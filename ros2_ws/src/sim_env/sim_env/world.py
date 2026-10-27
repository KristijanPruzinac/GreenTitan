"""World SDF builder: physics, geodetic origin, lighting, terrain and static objects.

World file layout::

    world:
      name: field
      physics: {engine: dart, step: 0.001, real_time_factor: 1.0}
      origin: {latitude: 45.5, longitude: 18.0, altitude: 90.0}   # ENU origin of the world frame
      gravity: [0, 0, -9.80665]
      rendering: false           # enable when any robot carries a camera
    terrain: {...}               # see terrain.py (type: mesh); omit or ``type: flat`` for a ground plane
    objects:
      - {name: wall, shape: {box: [10, 0.2, 1]}, position: [0, 5], yaw: 0, on_ground: true, color: [.6,.6,.6]}
    gnss_zones:                  # sky-view obstructions, read by the GNSS device model
      - {center: [0, 4], radius: 3, max_state: float}
"""
import math
import os

from . import sdf, terrain
from .sdf import el

PHYSICS_ENGINES = {'dart': 'gz-physics-dartsim-plugin', 'bullet': 'gz-physics-bullet-featherstone-plugin'}


def _ground_plane(friction):
    plane =el('geometry', el('plane', el('normal', text=[0, 0, 1]), el('size', text=[500, 500])))
    return el('model',
              el('static', text=True),
              el('link',
                 el('collision', plane, sdf.surface(friction), name='collision'),
                 el('visual', el('geometry', el('plane', el('normal', text=[0, 0, 1]),
                                                el('size', text=[500, 500]))),
                    sdf.color([0.3, 0.5, 0.2]), name='visual'),
                 name='link'),
              name='ground')


def _terrain_mesh(t, friction, contact):
    parts = []
    for i, tile in enumerate(t['tiles']):
        geom = lambda: el('geometry', el('mesh', el('uri', text='file://' + tile)))  # noqa: E731
        parts.append(el('collision', geom(), sdf.surface(friction, contact), name=f'collision_{i}'))
        parts.append(el('visual', geom(), name=f'visual_{i}'))
    return el('model', el('static', text=True), el('link', parts, name='link'), name='terrain')


def ground_z(scene, x, y):
    t = scene.get('terrain_data')
    return terrain.height_at(t['heights'], t['side'], x, y) if t else 0.0


def _object(obj, scene):
    x, y = obj['position'][:2]
    z = obj['position'][2] if len(obj['position']) > 2 else 0.0
    if obj.get('on_ground', True):
        z += ground_z(scene, x, y)
        shape = obj['shape']
        if 'box' in shape:
            z += shape['box'][2] / 2
        elif 'cylinder' in shape:
            z += shape['cylinder'][1] / 2
        elif 'sphere' in shape:
            z += shape['sphere']
    geom = sdf.geometry(obj['shape'])
    link_children = [
        el('collision', geom, sdf.surface(obj.get('friction')), name='collision'),
        el('visual', sdf.geometry(obj['shape']), sdf.color(obj.get('color', [0.6, 0.6, 0.6])), name='visual'),
    ]
    static = obj.get('static', True)
    if not static:
        link_children.insert(0, sdf.inertial(obj.get('mass', 1.0), obj['shape']))
    return el('model', el('static', text=static),
              sdf.pose([x, y, z, obj.get('roll', 0), obj.get('pitch', 0), obj.get('yaw', 0)]),
              el('link', *link_children, name='link'), name=obj['name'])


def build(cfg, out_dir):
    """Write ``<out_dir>/world.sdf``. Returns a scene dict (paths, terrain data, origin)."""
    w = cfg.get('world', {})
    name = w.get('name', 'world')
    phys = w.get('physics', {})
    origin = w.get('origin', {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0})
    scene = {'world_name': name, 'origin': origin, 'out_dir': out_dir}

    tcfg = cfg.get('terrain') or {'type': 'flat'}
    friction = tcfg.get('friction', {'mu': 0.8})
    if tcfg.get('type', 'mesh') == 'flat':
        ground = _ground_plane(friction)
        scene['terrain_data'] = None
    else:
        scene['terrain_data'] = terrain.write(tcfg, os.path.join(out_dir, 'terrain'))
        ground = _terrain_mesh(scene['terrain_data'], friction, tcfg.get('contact'))

    systems = [
        sdf.plugin('gz-sim-physics-system', 'gz::sim::systems::Physics',
                   el('engine', el('filename', text=PHYSICS_ENGINES.get(phys.get('engine', 'dart'),
                                                                         phys.get('engine'))))),
        sdf.plugin('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
        sdf.plugin('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
        sdf.plugin('gz-sim-imu-system', 'gz::sim::systems::Imu'),
        sdf.plugin('gz-sim-contact-system', 'gz::sim::systems::Contact'),
    ]
    if w.get('rendering', False):
        systems.append(sdf.plugin('gz-sim-sensors-system', 'gz::sim::systems::Sensors',
                                  render_engine=w.get('render_engine', 'ogre2')))

    sun = w.get('sun', {'direction': [-0.5, 0.2, -0.9]})
    world = el('world',
               el('physics', el('max_step_size', text=float(phys.get('step', 0.001))),
                  el('real_time_factor', text=float(phys.get('real_time_factor', 1.0))),
                  el('real_time_update_rate', text=-1 if phys.get('real_time_factor', 1.0) <= 0 else
                     1.0 / float(phys.get('step', 0.001))),
                  name='physics', type='ignored'),
               systems,
               el('gravity', text=w.get('gravity', [0, 0, -9.80665])),
               el('spherical_coordinates',
                  el('surface_model', text='EARTH_WGS84'),
                  el('world_frame_orientation', text='ENU'),
                  el('latitude_deg', text=float(origin['latitude'])),
                  el('longitude_deg', text=float(origin['longitude'])),
                  el('elevation', text=float(origin.get('altitude', 0.0))),
                  el('heading_deg', text=0.0)),
               el('scene', el('ambient', text=[0.5, 0.5, 0.5, 1]), el('background', text=[0.7, 0.8, 0.9, 1]),
                  el('shadows', text=w.get('shadows', False))),
               el('light', el('cast_shadows', text=w.get('shadows', False)),
                  sdf.pose([0, 0, 50]), el('diffuse', text=[0.9, 0.9, 0.85, 1]),
                  el('specular', text=[0.2, 0.2, 0.2, 1]), el('direction', text=sun['direction']),
                  type='directional', name='sun'),
               ground,
               [_object(o, scene) for o in cfg.get('objects', [])],
               name=name)
    path = os.path.join(out_dir, 'world.sdf')
    sdf.write(sdf.document(world), path)
    scene['world_sdf'] = path
    return scene


def enu_to_geodetic(origin, e, n, u):
    """Exact WGS84 ENU -> (lat, lon, alt) via ECEF."""
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    lat0, lon0 = math.radians(origin['latitude']), math.radians(origin['longitude'])
    h0 = origin.get('altitude', 0.0)
    sl, cl, so, co = math.sin(lat0), math.cos(lat0), math.sin(lon0), math.cos(lon0)
    n0 = a / math.sqrt(1 - e2 * sl * sl)
    x0, y0, z0 = (n0 + h0) * cl * co, (n0 + h0) * cl * so, (n0 * (1 - e2) + h0) * sl
    x = x0 - so * e - sl * co * n + cl * co * u
    y = y0 + co * e - sl * so * n + cl * so * u
    z = z0 + cl * n + sl * u
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - e2))
    for _ in range(5):
        nn = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        alt = p / math.cos(lat) - nn
        lat = math.atan2(z, p * (1 - e2 * nn / (nn + alt)))
    return math.degrees(lat), math.degrees(lon), alt
