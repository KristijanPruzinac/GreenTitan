"""Present a compact cowl in a new direct design using only neutral STEP geometry.

Run inside Fusion's Python console after obtaining the shared Fusion lease.
``present()`` creates an unsaved review document. ``archive_and_verify()``
exports a local F3D and reimports it for verification. No cloud save is used.
Neither function edits the V3 source.
All STEP parts must already use the original world coordinates, in millimetres.
"""
import hashlib
import json
import math
import re
import traceback
import uuid
from pathlib import Path

import adsk.core
import adsk.fusion


WORK = Path(r'C:\Users\kikom\Documents\GitHub\GreenTitan')
OUT = WORK / 'outputs/cowl/concept'
MANIFEST = OUT / 'manifest.json'
SOURCE_RECORD = WORK / 'outputs/chassis_reference/source_fingerprint.json'
REFERENCE_STEP = WORK / 'outputs/chassis_reference/original_v3_context.step'
SOURCE_ID = 'urn:adsk.wipprod:dm.lineage:_LQv_QGCR_qQgKkpnr3AIg'
NAME = 'Mower - Compact Cowl Study'
REFERENCE_GROUP = 'REFERENCE ONLY - V3 static STEP chassis'
COWL_GROUP = 'Compact cowl - exterior study'
ATTRIBUTE = 'GreenTitanCompactCowl'
RESULT = OUT / 'fusion_presentation.json'


def _write(record):
    RESULT.write_text(json.dumps(record, indent=2), encoding='utf-8')


def _sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _design(document):
    result = adsk.fusion.Design.cast(
        document.products.itemByProductType('DesignProductType'))
    if not result:
        raise RuntimeError('Document has no Fusion design: ' + document.name)
    return result


def _body(body, accurate=False):
    result = {'name': body.name, 'volume_cm3': body.volume,
            'min_mm': [v * 10 for v in body.boundingBox.minPoint.asArray()],
            'max_mm': [v * 10 for v in body.boundingBox.maxPoint.asArray()]}
    if accurate:
        result['accurate_volume_cm3']=body.getPhysicalProperties(
            adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy).volume
    return result


def _fingerprint(design, prefix=None, accurate=False):
    result = []
    for occurrence in design.rootComponent.allOccurrences:
        if prefix and not (occurrence.fullPathName == prefix or
                           occurrence.fullPathName.startswith(prefix + '+')):
            continue
        result.append({'path': occurrence.fullPathName,
                       'component': occurrence.component.name,
                       'transform': list(occurrence.transform2.asArray()),
                       'visible': occurrence.isVisible,
                       'bodies': [_body(b,accurate) for b in occurrence.bRepBodies]})
    return result


def _near(a, b):
    if isinstance(a, (int, float)) and not isinstance(a, bool):
        return isinstance(b, (int, float)) and abs(a - b) <= 1e-6
    if type(a) is not type(b):
        return False
    if isinstance(a, dict):
        return a.keys() == b.keys() and all(_near(a[k], b[k]) for k in a)
    if isinstance(a, list):
        return len(a) == len(b) and all(_near(x, y) for x, y in zip(a, b))
    return a == b


def _source_guard(app, baseline):
    if baseline['source_id'] != SOURCE_ID or baseline['source_modified']:
        raise RuntimeError('The stored source baseline is not clean V3.')
    doc=None
    for candidate in app.documents:
        try:
            data_file=candidate.dataFile
        except RuntimeError:
            # A newly submitted cloud document may not have a temporary DataFile
            # yet. It is not the known source lineage and must not block lookup.
            continue
        if data_file and data_file.id==SOURCE_ID:
            doc=candidate
            break
    if not doc or doc.isModified:
        raise RuntimeError('V3 must already be open and unmodified.')
    if doc.dataFile.versionNumber != baseline['source_version']:
        raise RuntimeError('V3 version changed since the neutral STEP export.')
    if not _near(_fingerprint(_design(doc)), baseline['original_geometry']):
        raise RuntimeError('V3 geometry differs from the recorded source baseline.')
    return doc


def _rgb(value):
    if isinstance(value, str):
        value = value.lstrip('#')
        if not re.fullmatch(r'[0-9a-fA-F]{6}', value):
            raise ValueError('Colour must be #RRGGBB or three RGB numbers.')
        return tuple(int(value[i:i + 2], 16) for i in (0, 2, 4))
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('Colour must contain three RGB numbers.')
    if any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError('Invalid RGB value.')
    if all(0 <= v <= 1 for v in value):
        value = [v * 255 for v in value]
    if any(v < 0 or v > 255 for v in value):
        raise ValueError('RGB values out of range.')
    return tuple(round(v) for v in value)


def _appearance(app, design, name, rgb):
    library = app.materialLibraries.itemByName('Fusion Appearance Library')
    libraries = ([library] if library else []) + [
        item for item in app.materialLibraries if not library or item.id != library.id]
    candidates = ('Plastic - Matte (Green)', 'Plastic - Matte (Black)',
                  'Plastic - Matte (White)', 'Plastic - Matte (Gray)',
                  'Plastic - Matte (Grey)', 'ABS (White)', 'ABS (Black)',
                  'ABS Plastic (White)', 'ABS Plastic (Black)')
    source = None
    for candidate in candidates:
        source = design.appearances.itemByName(candidate)
        if not source:
            source = next((found for item in libraries
                           if (found := item.appearances.itemByName(candidate))), None)
        if source:
            break
    if not source:
        raise RuntimeError('No installed matte plastic appearance found.')
    appearance = design.appearances.addByCopy(source, name)
    changed = False
    for prop in appearance.appearanceProperties:
        color = adsk.core.ColorProperty.cast(prop)
        if color and ('color' in prop.name.lower() or 'albedo' in prop.id.lower()):
            color.value = adsk.core.Color.create(*rgb, 255)
            changed = True
    if not changed:
        raise RuntimeError('The selected appearance has no editable colour.')
    return appearance


def _new_group(component, name):
    occurrence = component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occurrence.component.name = name
    occurrence.component.isOriginFolderLightBulbOn = False
    return occurrence


def _import(app, path, component):
    options = app.importManager.createSTEPImportOptions(str(path))
    options.isViewFit = False
    if not app.importManager.importToTarget(options, component):
        raise RuntimeError('STEP import failed: ' + str(path))


def _same_solid(a, b):
    # STEP translation can introduce tiny tolerance differences, but no shifts.
    return (abs(a['volume_cm3'] - b['volume_cm3']) <=
            max(0.0002, abs(a['volume_cm3']) * 1e-4) and
            all(abs(x - y) <= 0.02 for key in ('min_mm', 'max_mm')
                for x, y in zip(a[key], b[key])))


def _verify_neutral_geometry(baseline, imported):
    remaining = [b for occurrence in imported for b in occurrence['bodies']]
    count = 0
    volume_deltas = []
    for occurrence in baseline['original_geometry']:
        # The original STEP export omitted hidden occurrences such as BladeSpace.
        if not occurrence['visible']:
            continue
        for body in occurrence['bodies']:
            match = next((i for i, item in enumerate(remaining)
                          if _same_solid(body, item)), None)
            if match is None:
                raise RuntimeError('Neutral STEP lost or moved source solid: ' +
                                   occurrence['path'] + '/' + body['name'])
            item=remaining.pop(match)
            volume_deltas.append({'source':occurrence['path']+'/'+body['name'],
                                  'volume_delta_mm3':(item['volume_cm3']-body['volume_cm3'])*1000})
            count += 1
    return {'source_visible_solids_matched': count,
            'additional_imported_solids': len(remaining),
            'bbox_tolerance_mm': 0.02,
            'neutral_step_volume_tolerance_relative':0.0001,
            'note':'STEP import can change volume integration/representation slightly; source V3 itself is guarded exactly.',
            'volume_deltas':volume_deltas}


def _verify_part(part, imported):
    bodies = [body for occurrence in imported for body in occurrence['bodies']]
    if not bodies:
        raise RuntimeError('Concept import contains no BRep bodies: ' + part['name'])
    result = {'body_count': len(bodies),
              'volume_mm3': sum(body.get('accurate_volume_cm3',body['volume_cm3']) * 1000 for body in bodies),
              'volume_accuracy':'VeryHighCalculationAccuracy',
              'bbox_mm': ([min(b['min_mm'][i] for b in bodies) for i in range(3)] +
                          [max(b['max_mm'][i] for b in bodies) for i in range(3)])}
    if 'bbox_mm' in part:
        result['bbox_error_mm'] = max(abs(a - b) for a, b in
                                      zip(result['bbox_mm'], part['bbox_mm']))
        if result['bbox_error_mm'] > 0.02:
            raise RuntimeError('Concept import moved or resized: ' + part['name'])
    if 'volume_mm3' in part:
        result['volume_delta_mm3']=result['volume_mm3']-part['volume_mm3']
        result['volume_relative_error']=abs(result['volume_delta_mm3'])/part['volume_mm3']
        # Fusion and OCP integrate the trimmed loft skin differently. Retain a
        # recorded 0.005% transfer tolerance plus the independent 0.02 mm bounds
        # check; this does not relax the exact original V3 preservation guard.
        result['volume_relative_tolerance']=0.00005
        if abs(result['volume_delta_mm3']) > max(0.2, part['volume_mm3'] * .00005):
            raise RuntimeError('Concept import changed volume: ' + part['name'])
    return result


def _hide_reserves(design, reference):
    hidden = []
    baseline=json.loads(SOURCE_RECORD.read_text(encoding='utf-8'))
    reference_bodies=[body for occ in baseline['original_geometry'] for body in occ['bodies']]
    prefix = reference.fullPathName
    for occurrence in design.rootComponent.allOccurrences:
        if not (occurrence.fullPathName == prefix or
                occurrence.fullPathName.startswith(prefix + '+')):
            continue
        occurrence.component.isOriginFolderLightBulbOn = False
        component_name = re.sub(r':\d+$', '', occurrence.component.name).strip().lower()
        if component_name in ('space', 'bladespace'):
            occurrence.isLightBulbOn = False
            hidden.append(occurrence.fullPathName)
        for proxy in occurrence.bRepBodies:
            body=proxy.nativeObject or proxy
            match=next((b for b in reference_bodies if _same_solid(_body(proxy),b)),None)
            if match:body.name=match['name']
            if body.name.strip().lower() in ('space', 'bladespace'):
                body.isLightBulbOn = False
                hidden.append(occurrence.fullPathName + '/' + body.name)
    return hidden


def _review_image(app, design):
    design.rootComponent.isOriginFolderLightBulbOn = False
    viewport = app.activeViewport
    camera = viewport.camera
    camera.eye=adsk.core.Point3D.create((-50.982847+850)/10,120,
                                      (1002.105249+660)/10)
    camera.target=adsk.core.Point3D.create(-50.982847/10,25,(1002.105249+35)/10)
    camera.upVector=adsk.core.Vector3D.create(0,0,1)
    camera.isPerspective = False
    camera.isFitView = True
    viewport.camera = camera
    viewport.visualStyle = adsk.core.VisualStyles.ShadedVisualStyle
    viewport.refresh()
    path = OUT / 'fusion_concept_review.png'
    if not viewport.saveAsImageFile(str(path), 1400, 1000):
        raise RuntimeError('Could not save the Fusion review image.')
    return str(path)


def present(archive=False):
    """Create a blank-derived study. Inspect it before archive_and_verify()."""
    record = {'phase': 'preflight', 'source_id': SOURCE_ID, 'target_name': NAME,
              'run_id': uuid.uuid4().hex, 'reference_kind': 'static neutral STEP',
              'manifest': str(MANIFEST), 'cloud_save_requested': False}
    try:
        # OUT must exist already; never create a new non-junction output tree on C:.
        manifest = json.loads(MANIFEST.read_text(encoding='utf-8'))
        baseline = json.loads(SOURCE_RECORD.read_text(encoding='utf-8'))
        if manifest.get('units', 'mm') != 'mm':
            raise RuntimeError('The concept manifest must use millimetres.')
        parts = manifest['parts']
        if not parts or len({p['name'] for p in parts}) != len(parts):
            raise RuntimeError('Manifest is empty or contains duplicate part names.')
        prepared = []
        for part in parts:
            path = Path(part['step'])
            if not path.is_absolute():
                path = MANIFEST.parent / path
            if not path.is_file() or path.suffix.lower() not in ('.step', '.stp'):
                raise RuntimeError('Missing STEP part: ' + str(path))
            prepared.append((part, path, _rgb(part['color'])))
        record['manifest_sha256'] = _sha256(MANIFEST)
        record['reference_step_sha256'] = _sha256(REFERENCE_STEP)
        app = adsk.core.Application.get()
        _source_guard(app, baseline)
        if any(d.name == NAME or _design(d).rootComponent.attributes.itemByName(
                ATTRIBUTE, 'run_id') for d in app.documents
                if d.products.itemByProductType('DesignProductType')):
            raise RuntimeError('A compact cowl review document is already open; inspect it first.')
        document = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        if document.isSaved:
            raise RuntimeError('New document unexpectedly has a saved lineage.')
        document.name = NAME
        design = _design(document)
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        root = design.rootComponent
        root.attributes.add(ATTRIBUTE, 'run_id', record['run_id'])
        root.attributes.add(ATTRIBUTE, 'source', 'Neutral STEP snapshot; no live V3 links')
        record['phase'] = 'blank_direct_document_created'
        _write(record)

        reference = _new_group(root, REFERENCE_GROUP)
        reference.component.attributes.add(ATTRIBUTE, 'purpose',
            'V3 neutral reference only; original parametric source is separate and unchanged')
        _import(app, REFERENCE_STEP, reference.component)
        neutral_geometry = _fingerprint(design, reference.fullPathName)
        record['reference_import_check'] = _verify_neutral_geometry(baseline, neutral_geometry)
        record['hidden_reserves'] = _hide_reserves(design, reference)
        record['reference_prefix'] = reference.fullPathName
        record['reference_fingerprint'] = _fingerprint(design, reference.fullPathName)
        record['reserve_state'] = _reserve_state(design, reference.fullPathName)
        record['phase'] = 'neutral_reference_imported_and_verified'
        _write(record)

        cowl = _new_group(root, COWL_GROUP)
        cowl.component.attributes.add(ATTRIBUTE, 'purpose', 'Generated cowl concept; inspect fit separately')
        record['parts'] = []
        for part, path, rgb in prepared:
            occurrence = _new_group(cowl.component, part['name'])
            _import(app, path, occurrence.component)
            appearance = _appearance(app, design, 'Cowl ' + part['name'], rgb)
            occurrence.appearance = appearance
            # addNewComponent on a nested component returns a native occurrence;
            # its path is relative to that component, not the design root.
            prefix = cowl.fullPathName + '+' + occurrence.name
            for child in root.allOccurrences:
                if child.fullPathName == prefix or child.fullPathName.startswith(prefix + '+'):
                    child.component.isOriginFolderLightBulbOn = False
                    child.appearance = appearance
                    for body in child.component.bRepBodies:
                        body.appearance = appearance
            imported = _fingerprint(design, prefix, accurate=True)
            import_check = _verify_part(part, imported)
            record['parts'].append({'name': part['name'], 'step': str(path),
                                    'prefix': prefix,
                                    'sha256': _sha256(path), 'rgb': rgb,
                                    'import_check': import_check,
                                    'imported_geometry': imported})
        if not _near(_fingerprint(design, reference.fullPathName),
                     record['reference_fingerprint']):
            raise RuntimeError('Neutral chassis changed while importing the concept.')
        _source_guard(app, baseline)
        record['source_v3_modified'] = False
        record['source_geometry_unchanged'] = True
        record['neutral_reference_unchanged'] = True
        record['review_image'] = _review_image(app, design)
        record['phase'] = 'ready_for_review_unsaved'
        _write(record)
        if archive:
            return archive_and_verify()
        return record
    except Exception:
        record['phase'] = 'error'
        record['error'] = traceback.format_exc()
        if OUT.exists():
            _write(record)
        raise


def _reserve_state(design, prefix):
    result = []
    for occurrence in design.rootComponent.allOccurrences:
        if not (occurrence.fullPathName == prefix or
                occurrence.fullPathName.startswith(prefix + '+')):
            continue
        for proxy in occurrence.bRepBodies:
            body = proxy.nativeObject or proxy
            if body.name.strip().lower() in ('space', 'bladespace'):
                result.append({'path': occurrence.fullPathName + '/' + body.name,
                               'lightbulb_on': body.isLightBulbOn})
    return sorted(result, key=lambda item: item['path'])


def _marked_document(app, record):
    matches = []
    for document in app.documents:
        if not document.products.itemByProductType('DesignProductType'):
            continue
        marker = _design(document).rootComponent.attributes.itemByName(ATTRIBUTE, 'run_id')
        if marker and marker.value == record['run_id']:
            matches.append(document)
    if len(matches) != 1:
        raise RuntimeError('Expected exactly one marked compact cowl study document.')
    return matches[0]


def _verify_study(design, record):
    marker = design.rootComponent.attributes.itemByName(ATTRIBUTE, 'run_id')
    if not marker or marker.value != record['run_id']:
        raise RuntimeError('compact cowl study identity marker is missing or different.')
    if design.designType != adsk.fusion.DesignTypes.DirectDesignType:
        raise RuntimeError('The study is no longer a direct design.')
    if not _near(_fingerprint(design, record['reference_prefix']),
                 record['reference_fingerprint']):
        raise RuntimeError('Neutral reference changed since the import check.')
    for part in record['parts']:
        if not _near(_fingerprint(design, part['prefix'], accurate=True),
                     part['imported_geometry']):
            raise RuntimeError('Concept geometry changed: ' + part['name'])
    reserve_state = _reserve_state(design, record['reference_prefix'])
    if reserve_state != record['reserve_state'] or any(
            body['lightbulb_on'] for body in reserve_state):
        raise RuntimeError('Neutral reserve visibility changed.')
    return {'reference_geometry_unchanged': True,
            'cowl_geometry_unchanged': True,
            'hidden_reserves_preserved': True,
            'direct_design_preserved': True,
            'cowl_part_count': len(record['parts'])}


def discard_staged():
    """Discard only our unchanged, marked, unsaved compact cowl review document."""
    record = json.loads(RESULT.read_text(encoding='utf-8'))
    if record['phase'] != 'ready_for_review_unsaved':
        raise RuntimeError('Only a completed unsaved compact cowl stage can be discarded.')
    app = adsk.core.Application.get()
    baseline = json.loads(SOURCE_RECORD.read_text(encoding='utf-8'))
    _source_guard(app, baseline)
    document = _marked_document(app, record)
    if document.isSaved:
        raise RuntimeError('Refusing to discard a saved document.')
    _verify_study(_design(document), record)
    if not document.close(False):
        raise RuntimeError('Could not close the verified unsaved compact cowl stage.')
    _source_guard(app, baseline)
    record['phase'] = 'discarded_unsaved_stage'
    record['source_v3_modified'] = False
    record['source_geometry_unchanged'] = True
    _write(record)
    return record


def archive_and_verify():
    """Export local F3D, reopen it for verification, then return to the study.

    Only the new archive's temporary verification document is closed. The V3
    source and staged compact cowl study stay open. No cloud document save is attempted.
    """
    record = json.loads(RESULT.read_text(encoding='utf-8'))
    if record['phase'] != 'ready_for_review_unsaved':
        raise RuntimeError('The compact cowl study has not completed its review import.')
    app = adsk.core.Application.get()
    baseline = json.loads(SOURCE_RECORD.read_text(encoding='utf-8'))
    _source_guard(app, baseline)
    if _sha256(MANIFEST) != record['manifest_sha256']:
        raise RuntimeError('The concept manifest changed after the review import.')
    if _sha256(REFERENCE_STEP) != record['reference_step_sha256']:
        raise RuntimeError('The neutral source STEP changed after import.')
    for part in record['parts']:
        if _sha256(Path(part['step'])) != part['sha256']:
            raise RuntimeError('A concept STEP changed after import: ' + part['name'])
    document = _marked_document(app, record)
    design = _design(document)
    _verify_study(design, record)
    archive_path = OUT / 'Mower_Compact_Cowl_Study.f3d'
    if archive_path.exists():
        raise RuntimeError('Local archive already exists; refusing to overwrite: ' + str(archive_path))
    roundtrip_document = None
    roundtrip_close_attempted = False
    try:
        record['phase'] = 'local_archive_exporting'
        _write(record)
        options = design.exportManager.createFusionArchiveExportOptions(str(archive_path))
        if not design.exportManager.execute(options):
            raise RuntimeError('Could not export the independent local F3D archive.')
        if not archive_path.is_file() or archive_path.stat().st_size == 0:
            raise RuntimeError('Fusion did not produce a nonempty F3D archive.')
        record['local_archive'] = str(archive_path)
        record['archive_bytes'] = archive_path.stat().st_size
        record['archive_sha256'] = _sha256(archive_path)
        record['phase'] = 'local_archive_exported_verifying'
        _write(record)
        import_options = app.importManager.createFusionArchiveImportOptions(str(archive_path))
        import_options.isViewFit = False
        roundtrip_document = app.importManager.importToNewDocument(import_options)
        if not roundtrip_document:
            raise RuntimeError('Could not reopen the local F3D archive.')
        if roundtrip_document == document:
            roundtrip_document = None
            raise RuntimeError('F3D verification did not create a separate document.')
        roundtrip_design = _design(roundtrip_document)
        record['archive_roundtrip'] = _verify_study(roundtrip_design, record)
        # Cross-check the reimport against the external source baseline as well.
        record['archive_roundtrip']['source_solid_check'] = _verify_neutral_geometry(
            baseline, _fingerprint(roundtrip_design, record['reference_prefix']))
        _source_guard(app, baseline)
        roundtrip_close_attempted = True
        if not roundtrip_document.close(False):
            raise RuntimeError('Could not close the temporary F3D verification document.')
        roundtrip_document = None
        document.activate()
        _verify_study(design, record)
        _source_guard(app, baseline)
        record['archive_roundtrip']['verification_document_closed'] = True
        record['source_v3_modified'] = False
        record['source_geometry_unchanged'] = True
        record['cloud_save_requested'] = False
        record['phase'] = 'local_archive_verified'
        _write(record)
        return record
    except Exception:
        record['phase'] = 'local_archive_verification_error'
        record['error'] = traceback.format_exc()
        _write(record)
        raise
    finally:
        if roundtrip_document and not roundtrip_close_attempted:
            # Close only our marker-bearing import, never select by name.
            try:
                marker = _design(roundtrip_document).rootComponent.attributes.itemByName(
                    ATTRIBUTE, 'run_id')
                if marker and marker.value == record['run_id']:
                    roundtrip_document.close(False)
            except Exception:
                pass
        try:
            document.activate()
        except Exception:
            pass


if __name__ == '__main__':
    present(archive=globals().get('ARCHIVE_CONCEPT', False))
