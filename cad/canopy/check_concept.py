"""Read-only CAD interference audit of the canopy in original V3 coordinates.

Only JSON/text audit reports are written. No STEP geometry, source document, or
Fusion state is changed. Run with the CadQuery environment, for example:

    python cad/canopy/check_concept.py
    python cad/canopy/check_concept.py --manifest path/to/manifest.json

The manifest must contain ``parts: [{name, step, color, ...}, ...]``. Relative
STEP paths are resolved beside the manifest first, then against the repo root.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
from itertools import combinations
import json
from pathlib import Path
import re
import sys
import traceback

import cadquery as cq
from OCP.gp import gp_Trsf

ROOT = Path(__file__).resolve().parents[2]
# Reuse the converged mass integrator; no copy or change to source geometry.
sys.path.insert(0, str(ROOT / "cad/common"))
from accurate_volume import exact_volume, MASS_METHOD  # noqa: E402
from chassis_reference import CX, ZF, originals  # noqa: E402

SOURCE = ROOT / "outputs/chassis_reference"
DEFAULT_MANIFEST = ROOT / "outputs/canopy/concept/manifest.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def bounds(shape) -> list[float]:
    b = shape.BoundingBox()
    return [b.xmin, b.ymin, b.zmin, b.xmax, b.ymax, b.zmax]


def dimensions(bb: list[float]) -> dict:
    return {
        "world_min_mm": bb[:3],
        "world_max_mm": bb[3:],
        "size_mm": [bb[i + 3] - bb[i] for i in range(3)],
        "local_min_u_y_z_above_frame_mm": [bb[0] - CX, bb[1], bb[2] - ZF],
        "local_max_u_y_z_above_frame_mm": [bb[3] - CX, bb[4], bb[5] - ZF],
    }


def union_bounds(boxes: list[list[float]]) -> list[float]:
    return [min(b[i] for b in boxes) for i in range(3)] + [
        max(b[i] for b in boxes) for i in range(3, 6)
    ]


def boxes_overlap(a, b) -> bool:
    # Bounding boxes are only a broad-phase filter. Every overlapping pair
    # receives an exact CAD Boolean intersection below.
    return all(a[i] < b[i + 3] and b[i] < a[i + 3] for i in range(3))


def resolve_step(value: str, manifest: Path) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path.resolve(strict=True)
    for candidate in (manifest.parent / path, ROOT / path):
        if candidate.is_file():
            return candidate.resolve(strict=True)
    raise FileNotFoundError(f"STEP not found: {value!r} (manifest {manifest})")


def shape_record(name: str, shape, accurate=False) -> dict:
    bb = bounds(shape)
    record = {
        "name": name,
        "valid": bool(shape.isValid()),
        "solid_count": len(shape.Solids()),
        "volume_mm3": exact_volume(shape) if accurate else shape.Volume(),
        "volume_default_mm3": shape.Volume(),
        "mass_method": MASS_METHOD if accurate else "CadQuery default non-adaptive integration",
        "bounds": dimensions(bb),
    }
    if accurate:
        sizes = record["bounds"]["size_mm"]
        thin_axis = min(range(3), key=sizes.__getitem__)
        if thin_axis == 0:
            print_sizes = [sizes[2], sizes[1], sizes[0]]
            orientation = "Rotate 90 degrees about world Y to lay the thin X dimension vertically"
        elif thin_axis == 1:
            print_sizes = [sizes[0], sizes[2], sizes[1]]
            orientation = "Rotate 90 degrees about world X to lay the thin Y dimension vertically"
        else:
            print_sizes = sizes[:]
            orientation = "Keep world XY horizontal"
        record["candidate_print_bounds"] = {
            "orientation": orientation,
            "size_xyz_mm": print_sizes,
            "fits_240_mm_bed_xy": max(print_sizes[:2]) <= 240.00001,
            "within_230_mm_working_target_xy": max(print_sizes[:2]) <= 230.00001,
            "printer_height_verified": False,
            "final_print_setup_validated": False,
            "note": "Bounding-box screening only. No added joints, brims or support structures are included; actual print orientation and printer Z capacity require separate validation.",
        }
    return record


def inspect_pair(a_name, a, a_bb, b_name, b, b_bb, group, report, tolerance):
    bucket = report["checks"][group]
    bucket["pair_count"] += 1
    if not boxes_overlap(a_bb, b_bb):
        bucket["bbox_disjoint_count"] += 1
        return
    try:
        common = a.intersect(b)
        default_volume = common.Volume()
        # Integrate solid intersections adaptively too. Thin trimmed spline
        # solids can have materially inaccurate default mass integration.
        has_solids = bool(common.Solids())
        volume = exact_volume(common) if has_solids else 0.0
        # Empty intersections are valid. Check validity where a positive-volume
        # intersection is reported, so an invalid Boolean is not called clear.
        if volume > tolerance and not common.isValid():
            raise ValueError("Boolean intersection returned an invalid shape")
        row = {"part": a_name, "against": b_name, "volume_mm3": volume,
               "volume_default_mm3": default_volume,
               "mass_method": MASS_METHOD if has_solids else "No solid intersection"}
        bucket["exact_intersections"].append(row)
        if volume > tolerance:
            bucket["collisions"].append(row)
            print(f"  {group}: {b_name}: {volume:.6f} mm^3", flush=True)
    except Exception as exc:
        report["errors"].append({
            "stage": "intersection", "group": group,
            "part": a_name, "against": b_name,
            "error": f"{type(exc).__name__}: {exc}",
        })


def audit(manifest_path: Path, report: dict, tolerance: float):
    manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
    records = manifest.get("parts")
    if not isinstance(records, list) or not records:
        raise ValueError("Manifest must contain a nonempty parts array")
    names = [p["name"] for p in records]
    if any(not isinstance(n, str) or not n for n in names):
        raise ValueError("Each part needs a nonempty string name")
    if len(set(names)) != len(names):
        raise ValueError("Manifest contains duplicate part names")
    if manifest.get("units", "mm").lower() != "mm":
        raise ValueError("Manifest geometry must use millimetres")

    paths = [
        SOURCE / "original_v3_context.step", SOURCE / "source_fingerprint.json",
        SOURCE / "BladeSpace_local.step", SOURCE / "BladeSpace_transform.json",
        manifest_path,
    ]
    part_paths = [resolve_step(p["step"], manifest_path) for p in records]
    paths.extend(part_paths)
    report["input_files"] = [{"path": str(p), "sha256_before": sha256(p)} for p in dict.fromkeys(paths)]

    fingerprint = json.loads((SOURCE / "source_fingerprint.json").read_text())
    refs = {o["path"] + "/" + b["name"]: b
            for o in fingerprint["original_geometry"] for b in o["bodies"]}
    rail_names = {o["path"] + "/" + b["name"]
                  for o in fingerprint["original_geometry"]
                  if re.fullmatch(r"2020(?: \(\d+\))?", o["component"])
                  for b in o["bodies"]}
    source_shapes = originals()
    source_names = [name for name, _, _ in source_shapes]
    source_rows = []
    for name, shape, bb in source_shapes:
        ref = refs[name]
        row = shape_record(name, shape)
        row["fingerprint_bbox_max_error_mm"] = max(
            abs(a - b) for a, b in zip(bb, ref["min_mm"] + ref["max_mm"]))
        row["fingerprint_volume_difference_mm3"] = shape.Volume() - ref["volume_cm3"] * 1000
        source_rows.append(row)

    # Inspect and apply the saved rigid placement exactly once. The currently
    # supplied matrix is identity, and the stored world bounds verify this.
    blade_data = json.loads((SOURCE / "BladeSpace_transform.json").read_text())
    blade_local = cq.importers.importStep(str(SOURCE / "BladeSpace_local.step")).val()
    blade_local_bb = bounds(blade_local)
    matrix = blade_data["matrix4x4_mm"]
    transform = gp_Trsf()
    transform.SetValues(*[v for row in matrix[:3] for v in row])
    blade = blade_local.moved(cq.Location(transform))
    blade_bb = bounds(blade)
    expected = blade_data["world_bbox_mm"][0] + blade_data["world_bbox_mm"][1]
    blade_bbox_error = max(abs(a - b) for a, b in zip(blade_bb, expected))
    blade_volume_error = abs(blade.Volume() - blade_data["volume_mm3"])
    blade_name = blade_data["source_occurrence"] + "/Body1"
    report["blade_reference"] = {
        **shape_record(blade_name, blade),
        "exported": blade_data.get("exported"),
        "saved_matrix4x4_mm": matrix,
        "local_step_bounds": dimensions(blade_local_bb),
        "world_bbox_max_error_mm": blade_bbox_error,
        "reference_volume_error_mm3": blade_volume_error,
        "placement_verified": blade_bbox_error < 0.02 and blade_volume_error < 0.1,
    }
    if not report["blade_reference"]["placement_verified"]:
        raise ValueError("BladeSpace world placement/volume does not match its saved reference")

    actual = [(n, s, list(bb)) for n, s, bb in source_shapes
              if not n.endswith("/Space") and not n.startswith("BladeSpace:")]
    caster = [(n, s, list(bb)) for n, s, bb in source_shapes if n.endswith("/Space")]
    checked_names = set(source_names) | {blade_name}
    coverage = {
        "fingerprint_body_count": len(refs),
        "context_solid_count": len(source_shapes),
        "actual_component_body_count": len(actual),
        "caster_keepout_count": len(caster),
        "missing_bodies_after_separate_blade_reference": sorted(set(refs) - checked_names),
        "unexpected_bodies": sorted(checked_names - set(refs)),
        "duplicate_context_name_matches": sorted(n for n, count in Counter(source_names).items() if count > 1),
        "rail_names": sorted(rail_names),
        "rails_checked": sorted(rail_names & set(source_names)),
        "all_four_original_rails_present": len(rail_names) == 4 and rail_names <= set(source_names),
        "context_bounds_match_fingerprint": all(r["fingerprint_bbox_max_error_mm"] < 0.02 for r in source_rows),
        "source_bodies": source_rows,
    }
    report["source_coverage"] = coverage
    report["caster_keepouts"] = [shape_record(n, s) for n, s, _ in caster]
    allowance = cq.Solid.makeBox(190, 170, 200, cq.Vector(CX - 95, 70, ZF))
    allowance_bb = bounds(allowance)
    allowance_record = shape_record("190 x 170 x 200 provisional headroom allowance", allowance)
    allowance_record.update({
        "actual_electronics_geometry": False,
        "feasible_electronics_packing_verified": False,
        "existing_component_overlaps": [],
        "note": "Assumed headroom only. This box overlaps the existing mowing motor and is not an unoccupied or feasible electronics packing volume. Actual electronics and antenna geometry and installation positions are unmodelled.",
    })
    report["provisional_headroom_allowance"] = allowance_record
    # Document existing occupation independently of the new-part checks. Such
    # an overlap disproves an empty electronics box, but is not a new canopy
    # collision and must not be silently attributed to the proposed geometry.
    for original_name, original, original_bb in actual:
        if not boxes_overlap(allowance_bb, original_bb):
            continue
        try:
            common = allowance.intersect(original)
            volume = exact_volume(common) if common.Solids() else 0.0
            if volume > tolerance:
                if not common.isValid():
                    raise ValueError("Invalid headroom/original intersection")
                allowance_record["existing_component_overlaps"].append({
                    "original": original_name, "volume_mm3": volume,
                    "mass_method": MASS_METHOD,
                })
        except Exception as exc:
            report["errors"].append({"stage": "headroom_original_occupation",
                                     "original": original_name,
                                     "error": f"{type(exc).__name__}: {exc}"})

    loaded = []
    for record, path in zip(records, part_paths):
        name = record["name"]
        print(f"Checking {name}", flush=True)
        shape = cq.importers.importStep(str(path)).val()
        row = shape_record(name, shape, accurate=True)
        row.update({"step": str(path), "color": record.get("color")})
        report["parts"].append(row)
        bb = bounds(shape)
        loaded.append((name, shape, bb))
        for original_name, original, original_bb in actual:
            inspect_pair(name, shape, bb, original_name, original, original_bb,
                         "actual_original_components", report, tolerance)
        for keepout_name, keepout, keepout_bb in caster:
            inspect_pair(name, shape, bb, keepout_name, keepout, keepout_bb,
                         "caster_sweep_keepouts", report, tolerance)
        inspect_pair(name, shape, bb, blade_name, blade, blade_bb,
                     "blade_space_reference", report, tolerance)
        inspect_pair(name, shape, bb, "provisional_headroom_allowance", allowance, allowance_bb,
                     "provisional_headroom_allowance", report, tolerance)

    for (a_name, a, a_bb), (b_name, b, b_bb) in combinations(loaded, 2):
        print(f"Checking concept pair {a_name} / {b_name}", flush=True)
        inspect_pair(a_name, a, a_bb, b_name, b, b_bb,
                     "concept_part_pairs", report, tolerance)

    concept_bb = union_bounds([bb for _, _, bb in loaded])
    actual_bb = union_bounds([bb for _, _, bb in actual])
    report["concept_bounds"] = dimensions(concept_bb)
    report["mower_with_concept_bounds"] = dimensions(union_bounds([concept_bb, actual_bb]))
    report["caster_swept_mower_with_concept_bounds"] = dimensions(
        union_bounds([concept_bb, actual_bb] + [bb for _, _, bb in caster]))
    report["approximate_height_from_lowest_original_component_to_concept_top_mm"] = concept_bb[5] - actual_bb[2]

    for record in report["input_files"]:
        record["sha256_after"] = sha256(Path(record["path"]))
        record["unchanged_during_audit"] = record["sha256_before"] == record["sha256_after"]

    source_valid = all(r["valid"] and r["solid_count"] > 0 for r in source_rows)
    coverage_ok = (
        not coverage["missing_bodies_after_separate_blade_reference"]
        and not coverage["unexpected_bodies"]
        and not coverage["duplicate_context_name_matches"]
        and coverage["all_four_original_rails_present"]
        and coverage["context_bounds_match_fingerprint"]
        and len(caster) == 2
    )
    report["validation"] = {
        "all_concept_shapes_valid_solids": all(p["valid"] and p["solid_count"] > 0 and p["volume_mm3"] > 0 for p in report["parts"]),
        "original_context_shapes_valid": source_valid,
        "blade_reference_valid": bool(blade.isValid()) and len(blade.Solids()) > 0,
        "source_coverage_complete": bool(coverage_ok),
        "input_files_unchanged_during_audit": all(p["unchanged_during_audit"] for p in report["input_files"]),
        "no_boolean_errors": not report["errors"],
        "no_positive_volume_intersections_above_threshold": all(not v["collisions"] for v in report["checks"].values()),
    }
    report["passed"] = all(report["validation"].values())


def readable(report: dict) -> str:
    lines = [
        "GreenTitan canopy concept CAD interference audit",
        f"Result: {'PASS within the stated scope' if report['passed'] else 'FAIL / incomplete; see details'}",
        f"Manifest: {report['manifest']}",
        f"Intersection reporting threshold: {report['volume_tolerance_mm3']} mm^3",
        "",
    ]
    if "concept_bounds" in report:
        size = report["concept_bounds"]["size_mm"]
        lines.append("Concept world-axis width x length x height: " + " x ".join(f"{v:.3f}" for v in size) + " mm")
    for part in report["parts"]:
        size = part["bounds"]["size_mm"]
        lines.append(f"{part['name']}: valid={part['valid']}, solids={part['solid_count']}, "
                     + " x ".join(f"{v:.3f}" for v in size) + " mm")
        if "candidate_print_bounds" in part:
            candidate = part["candidate_print_bounds"]
            lines.append("  Candidate print XYZ: " + " x ".join(f"{v:.3f}" for v in candidate["size_xyz_mm"])
                         + " mm; " + candidate["orientation"] + ". Bounding-box screening only.")
    lines.append("")
    for name, check in report["checks"].items():
        lines.append(f"{name}: {len(check['collisions'])} collisions; {check['pair_count']} pairs screened; "
                     f"{len(check['exact_intersections'])} exact intersections computed")
        for collision in check["collisions"]:
            lines.append(f"  {collision['part']} / {collision['against']}: {collision['volume_mm3']:.6f} mm^3")
    if "provisional_headroom_allowance" in report:
        allowance = report["provisional_headroom_allowance"]
        lines.append("\nProvisional headroom allowance: " + allowance["note"])
        for overlap in allowance["existing_component_overlaps"]:
            lines.append(f"  Existing occupation by {overlap['original']}: {overlap['volume_mm3']:.6f} mm^3")
    if report["errors"]:
        lines.append("\nErrors:")
        lines.extend("  " + json.dumps(error) for error in report["errors"])
    lines.append("\nScope and limitations:")
    lines.extend("- " + note for note in report["scope"])
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--report", type=Path, help="Default: geometry_check.json beside manifest")
    parser.add_argument("--volume-tolerance", type=float, default=0.01,
                        help="Report collisions above this numerical volume threshold in mm^3 (default: 0.01)")
    args = parser.parse_args()
    if args.volume_tolerance < 0:
        parser.error("--volume-tolerance must be nonnegative")
    manifest = args.manifest.resolve()
    output = args.report.resolve() if args.report else manifest.parent / "geometry_check.json"
    if output.suffix.lower() != ".json":
        parser.error("--report must end in .json; a companion .txt report is also written")
    if output == manifest or output.with_suffix(".txt") == manifest:
        parser.error("Report may not overwrite the manifest")
    if output.is_relative_to(SOURCE.resolve()):
        parser.error("Report may not be written into the read-only chassis reference directory")
    if output.parent != manifest.parent and not output.parent.is_dir():
        parser.error("Custom report directory must already exist")
    report = {
        "audit": "read-only canopy concept final-position CAD interference",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "manifest": str(manifest), "units": "mm",
        "world_origin_reference": {"frame_centre_x": CX, "frame_top_z": ZF},
        "volume_tolerance_mm3": args.volume_tolerance,
        "parts": [], "errors": [], "passed": False,
        "checks": {name: {"pair_count": 0, "bbox_disjoint_count": 0,
                           "exact_intersections": [], "collisions": []}
                   for name in ("actual_original_components", "caster_sweep_keepouts",
                                "blade_space_reference", "provisional_headroom_allowance", "concept_part_pairs")},
        "scope": [
            "Only report files are written. Source geometry, concept STEP geometry, and Fusion are not modified.",
            "Exact Boolean intersections are computed after a bounding-box broad-phase screen. Absence of intersection does not establish a positive manufacturing clearance.",
            "All four original aluminium 2020 rails and other modelled original component bodies are checked in their saved world positions; hidden caster Space keepouts are reported separately.",
            "BladeSpace is the existing hidden reference, with saved placement and volume checked. Unmodelled extended blades, the complete blade motion envelope, blade flex and deflection remain unverified.",
            "The provisional headroom allowance is U=-95..95, Y=70..240, Z=0..200 above the frame. It overlaps the original mowing motor and is not a feasible electronics packing volume. The check only measures whether new canopy geometry enters that assumed allowance; actual electronics geometry and positions remain unmodelled.",
            "Antenna size, clearance, RF performance and openings remain unverified because no actual antenna geometry was supplied.",
            "This is a final-position concept audit. Mount design, insertion/removal, retention, print orientation and printer fit, strength, fatigue, weather protection and charging interfaces require separate validation.",
            "World-axis dimensions are packaging measurements, not approved print envelopes. Input hashes establish only that this audit did not change the files; no live Fusion preservation claim is made.",
        ],
    }
    try:
        audit(manifest, report, args.volume_tolerance)
    except Exception as exc:
        report["errors"].append({"stage": "audit", "error": f"{type(exc).__name__}: {exc}",
                                 "traceback": traceback.format_exc()})
    input_paths = {Path(p["path"]).resolve() for p in report.get("input_files", [])}
    if output in input_paths or output.with_suffix(".txt") in input_paths:
        parser.error("Report may not overwrite an input file")
    if not output.parent.is_dir():
        print(json.dumps(report, indent=2), flush=True)
        print(f"Report directory does not exist: {output.parent}", file=sys.stderr)
        return 2
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    output.with_suffix(".txt").write_text(readable(report), encoding="utf-8")
    print(readable(report), flush=True)
    print(f"Wrote {output}", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
