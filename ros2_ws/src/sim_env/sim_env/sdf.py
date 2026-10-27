"""Tiny SDF/XML element helpers shared by the world and robot builders."""
import xml.etree.ElementTree as ET

SDF_VERSION = '1.9'


def el(tag, *children, text=None, **attrs):
    """``el('link', el('pose', text='0 0 0 0 0 0'), name='base')``. Lists of children are flattened."""
    node = ET.Element(tag, {k.rstrip('_'): str(v) for k, v in attrs.items() if v is not None})
    if text is not None:
        node.text = fmt(text)
    for child in children:
        for c in (child if isinstance(child, (list, tuple)) else [child]):
            if c is not None:
                node.append(c)
    return node


def fmt(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (list, tuple)):
        return ' '.join(fmt(v) for v in value)
    if isinstance(value, float):
        return f'{value:.6g}'
    return str(value)


def pose(values, relative_to=None):
    values = list(values) + [0.0] * (6 - len(values))
    return el('pose', text=[float(v) for v in values], relative_to=relative_to)


def plugin(filename, name, *children, **params):
    return el('plugin', *children, *[el(k, text=v) for k, v in params.items()],
              filename=filename, name=name)


def color(rgba):
    rgba = list(rgba) + [1.0] * (4 - len(rgba))
    return el('material', el('ambient', text=rgba), el('diffuse', text=rgba),
              el('specular', text=[0.1, 0.1, 0.1, 1.0]))


def geometry(shape):
    """Shape dict -> <geometry>. ``{box: [x,y,z]}``, ``{cylinder: [r, len]}``, ``{sphere: r}``,
    ``{mesh: uri, scale: [..]}``."""
    if 'box' in shape:
        g = el('box', el('size', text=shape['box']))
    elif 'cylinder' in shape:
        r, length = shape['cylinder']
        g = el('cylinder', el('radius', text=r), el('length', text=length))
    elif 'sphere' in shape:
        g = el('sphere', el('radius', text=shape['sphere']))
    elif 'mesh' in shape:
        g = el('mesh', el('uri', text=shape['mesh']),
               el('scale', text=shape.get('scale', [1, 1, 1])))
    else:
        raise ValueError(f'unknown geometry: {shape}')
    return el('geometry', g)


def surface(friction=None, contact=None):
    friction = friction or {}
    mu = friction.get('mu', 1.0)
    ode = el('ode', el('mu', text=mu), el('mu2', text=friction.get('mu2', mu)),
             el('fdir1', text=friction['fdir1']) if 'fdir1' in friction else None,
             el('slip1', text=friction.get('slip1', 0.0)), el('slip2', text=friction.get('slip2', 0.0)))
    children = [el('friction', ode)]
    if contact:
        children.append(el('contact', el('ode', *[el(k, text=v) for k, v in contact.items()])))
    return el('surface', *children)


def inertial(mass, shape=None, com=None, inertia=None):
    """Inertia from an explicit ``[ixx, iyy, izz]`` or computed from a primitive shape."""
    if inertia is None:
        if shape and 'box' in shape:
            x, y, z = shape['box']
            inertia = [mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12, mass * (x * x + y * y) / 12]
        elif shape and 'cylinder' in shape:  # axis along z
            r, length = shape['cylinder']
            side = mass * (3 * r * r + length * length) / 12
            inertia = [side, side, mass * r * r / 2]
        elif shape and 'sphere' in shape:
            r = shape['sphere']
            inertia = [2 * mass * r * r / 5] * 3
        else:
            inertia = [mass * 1e-3] * 3
    ixx, iyy, izz = inertia
    return el('inertial', pose(com or [0, 0, 0]), el('mass', text=float(mass)),
              el('inertia', el('ixx', text=ixx), el('iyy', text=iyy), el('izz', text=izz),
                 el('ixy', text=0.0), el('ixz', text=0.0), el('iyz', text=0.0)))


def document(*children):
    return el('sdf', *children, version=SDF_VERSION)


def write(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
