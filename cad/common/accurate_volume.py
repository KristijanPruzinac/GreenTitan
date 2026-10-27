"""Accurate numerical volume integration for the trimmed V5 B-spline solids.

CadQuery's default Shape.Volume() uses the non-adaptive BRepGProp overload.
For these thin shells it differs from converged adaptive integration by up to
0.18%. This helper integrates the existing geometry; it never changes a shape.
"""
from math import isfinite

from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

MASS_METHOD = (
    "OCP BRepGProp.VolumePropertiesGK_s; Eps=1e-12; OnlyClosed=True; "
    "IsUseSpan=True; CGFlag=False; IFlag=False; SkipShared=False"
)


def volume_details(shape, eps=1e-12):
    """Return GK volume and its reported error estimate for a closed shape.

    The requested tolerance is a numerical integration target, not a guaranteed
    final relative accuracy. The returned OCCT estimate can exceed that target.
    """
    props = GProp_GProps()
    wrapped = getattr(shape, "wrapped", shape)
    error = BRepGProp.VolumePropertiesGK_s(
        wrapped, props, float(eps), True, True, False, False, False
    )
    volume = props.Mass()
    if error < 0 or not isfinite(error) or not isfinite(volume):
        raise ValueError(f"Gauss-Kronrod volume integration failed: error={error}, volume={volume}")
    return {"volume_mm3": volume, "reported_relative_error": error, "requested_eps": eps}


def exact_volume(shape):
    """Numerically integrate volume using exact CAD surfaces and spline spans.

    'Exact' distinguishes the CAD surfaces from a triangulated approximation;
    the integral itself remains numerical. Input must represent closed solids.
    """
    return volume_details(shape)["volume_mm3"]
