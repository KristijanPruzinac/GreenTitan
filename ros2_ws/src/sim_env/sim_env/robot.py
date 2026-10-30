"""Robot SDF builder.

``type: wheeled`` builds a rigid body with any number of driven wheels and casters (sphere or swivel) from a
YAML description. ``type: file`` uses an existing SDF model unchanged and only declares its
interface (joints and sensor topics) so the rest of the tooling can bridge it.

All positions are in the robot frame (x forward, y left, z up); its origin is the model origin.

    robot:
      name: rover
      type: wheeled
      body:                             # rigid parts of the base link; mass optional per part
        - {name: chassis, shape: {box: [0.5, 0.3, 0.1]}, position: [0, 0, 0.12], mass: 5}
        - {name: battery, shape: {box: [0.1, 0.1, 0.1]}, position: [0.1, 0, 0.1], mass: 1, collide: false}
      wheels:
        - {name: left,  position: [0,  0.18, 0.075], radius: 0.075, width: 0.04, mass: 0.4, driven: true}
        - {name: right, position: [0, -0.18, 0.075], radius: 0.075, width: 0.04, mass: 0.4, driven: true}
        - {name: caster, position: [-0.3, 0, 0.03], radius: 0.03, mass: 0.1, caster: true}
        # or a swivel caster: wheel centre trailing behind a free vertical axis at swivel.position
        # - {name: caster, position: [-0.33, 0, 0.03], radius: 0.03, width: 0.02, mass: 0.05, caster: true,
        #    caster_joint: swivel, swivel: {position: [-0.3, 0, 0.08], mass: 0.05, damping: 0.001}}
      actuators: {mode: servo, max_torque: 5.0}     # servo | pid (pid: {p, i, d})
      sensors:
        - {type: imu, name: imu, position: [0, 0, 0.1], rpy: [0, 0, 0], rate: 200}
        - {type: camera, name: front_cam, position: [0.25, 0, 0.3], rate: 15, width: 640, height: 480, hfov: 1.2}
      ground_truth: {rate: 50}
      spawn: {position: [0, 0], yaw: 0, clearance: 0.02}
"""
import math
import os
import shutil

import numpy as np

from . import sdf
from .sdf import el

DEFAULT_WHEEL_FRICTION = {'mu': 0.8, 'mu2': 0.8}
CASTER_FRICTION = {'mu': 0.05, 'mu2': 0.05}


def topics(name):
    """Every simulator-side topic name for a robot, used by the bridge and device nodes."""
    base = f'/sim/{name}'
    return {
        'ground_truth': f'{base}/ground_truth',
        'joint_states': f'{base}/joint_states',
        'joint_cmd': lambda joint: f'{base}/joint/{joint}/cmd_vel',
        'sensor': lambda sensor: f'{base}/{sensor}',
    }


def _combine_inertia(parts):
    """Mass, COM and diagonal inertia of the base link from its massive body parts."""
    massive = [p for p in parts if p.get('mass')]
    if not massive:
        return 1.0, [0, 0, 0], [1e-2, 1e-2, 1e-2]
    m = sum(float(p['mass']) for p in massive)
    com = sum(np.array(p['position'], float) * float(p['mass']) for p in massive) / m
    inertia = np.zeros(3)
    for p in massive:
        own = sdf.inertial(float(p['mass']), p.get('shape')).find('inertia')
        local = np.array([float(own.find(k).text) for k in ('ixx', 'iyy', 'izz')])
        d = np.array(p['position'], float) - com
        inertia += local + float(p['mass']) * np.array([d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2,
                                                        d[0] ** 2 + d[1] ** 2])
    return m, com.tolist(), inertia.tolist()


def _base_link(cfg):
    parts = cfg.get('body', [])
    mass, com, inertia = _combine_inertia(parts)
    children = [el('inertial', sdf.pose(com), el('mass', text=mass),
                   el('inertia', el('ixx', text=inertia[0]), el('iyy', text=inertia[1]),
                      el('izz', text=inertia[2]), el('ixy', text=0.0), el('ixz', text=0.0),
                      el('iyz', text=0.0)))]
    for part in parts:
        p = sdf.pose(list(part['position']) + list(part.get('rpy', [0, 0, 0])))
        if part.get('collide', True):
            children.append(el('collision', p, sdf.geometry(part['shape']),
                               sdf.surface(part.get('friction', {'mu': 0.5})), name=part['name']))
        if part.get('visible', True):
            children.append(el('visual', sdf.pose(list(part['position']) + list(part.get('rpy', [0, 0, 0]))),
                               sdf.geometry(part['shape']),
                               sdf.color(part.get('color', [0.3, 0.3, 0.3])), name=part['name']))
    return el('link', children, name='base_link')


def _swivel_caster(w):
    """Swivel caster: a fork that turns freely about a vertical axis at ``swivel.position`` and a
    wheel (cylinder, axle along y) that rolls freely in it. ``position`` is the wheel centre at
    rest, trailing behind the axis; bumps and side forces then act through the real lever arms."""
    sw = w['swivel']
    fork_name = f"{w['name']}_fork"
    fork_shape = {'box': sw.get('size', [0.02, 0.02, 0.02])}
    fork_mass = float(sw.get('mass', 0.05))
    fork = el('link', sdf.pose(sw['position']), sdf.inertial(fork_mass, fork_shape),
              el('visual', sdf.pose([0, 0, 0]), sdf.geometry(fork_shape),
                 sdf.color(w.get('color', [0.1, 0.1, 0.1])), name='visual'),
              name=fork_name)
    swivel = el('joint', el('parent', text='base_link'), el('child', text=fork_name),
                el('axis', el('xyz', text=[0, 0, 1], expressed_in='__model__'),
                   el('limit', el('lower', text=-1e16), el('upper', text=1e16)),
                   el('dynamics', el('damping', text=sw.get('damping', 0.0)),
                      el('friction', text=sw.get('friction', 0.0)))),
                type='revolute', name=f"{w['name']}_swivel_joint")
    shape = {'cylinder': [float(w['radius']), float(w['width'])]}
    axle_pose = sdf.pose([0, 0, 0, math.pi / 2, 0, 0])
    inertial = sdf.inertial(float(w.get('mass', 0.05)), shape)
    ine = inertial.find('inertia')  # cylinder inertia is about z; rotate to the y axle
    ine.find('iyy').text, ine.find('izz').text = ine.find('izz').text, ine.find('ixx').text
    contact_shape, contact_pose = shape, axle_pose
    if w.get('collision', 'cylinder') == 'sphere':
        contact_shape, contact_pose = {'sphere': float(w['radius'])}, sdf.pose([0, 0, 0])
    wheel = el('link', sdf.pose(w['position']), inertial,
               el('collision', contact_pose, sdf.geometry(contact_shape),
                  sdf.surface(w.get('friction', DEFAULT_WHEEL_FRICTION), w.get('contact')), name='collision'),
               el('visual', axle_pose, sdf.geometry(shape), sdf.color(w.get('color', [0.1, 0.1, 0.1])),
                  name='visual'),
               name=w['name'])
    dyn = w.get('dynamics', {})
    axle = el('joint', el('parent', text=fork_name), el('child', text=w['name']),
              el('axis', el('xyz', text=[0, 1, 0]),  # joint frame = wheel frame: turns with the fork
                 el('limit', el('lower', text=-1e16), el('upper', text=1e16)),
                 el('dynamics', el('damping', text=dyn.get('damping', 0.0)),
                    el('friction', text=dyn.get('friction', 0.0)))),
              type='revolute', name=f"{w['name']}_joint")
    return [fork, wheel], [swivel, axle]


def _wheel(w, max_torque):
    """Links and joints of one wheel: a driven or free wheel, a caster sphere or a swivel caster."""
    if w.get('caster') and w.get('caster_joint') == 'swivel':
        return _swivel_caster(w)
    link_pose = sdf.pose(w['position'])
    if w.get('caster'):
        shape = {'sphere': float(w['radius'])}
        geo_pose = sdf.pose([0, 0, 0])
        # 'fixed' is a rigid low-friction sphere (robust); 'ball' lets the sphere rotate freely.
        joint = el('joint', el('parent', text='base_link'), el('child', text=w['name']),
                   type=w.get('caster_joint', 'fixed'), name=f"{w['name']}_joint")
        friction = w.get('friction', CASTER_FRICTION)
    else:
        shape = {'cylinder': [float(w['radius']), float(w['width'])]}
        geo_pose = sdf.pose([0, 0, 0, math.pi / 2, 0, 0])
        dyn = w.get('dynamics', {})
        joint = el('joint', el('parent', text='base_link'), el('child', text=w['name']),
                   el('axis', el('xyz', text=[0, 1, 0], expressed_in='__model__'),
                      el('limit', el('lower', text=-1e16), el('upper', text=1e16),
                         el('effort', text=float(max_torque)), el('velocity', text=-1)),
                      el('dynamics', el('damping', text=dyn.get('damping', 0.0)),
                         el('friction', text=dyn.get('friction', 0.0)))),
                   type='revolute', name=f"{w['name']}_joint")
        friction = w.get('friction', DEFAULT_WHEEL_FRICTION)
    inertial = sdf.inertial(float(w.get('mass', 0.3)), shape)
    if not w.get('caster'):  # cylinder inertia is about z; rotate to the y axle
        ine = inertial.find('inertia')
        ixx, izz = ine.find('ixx').text, ine.find('izz').text
        ine.find('iyy').text, ine.find('izz').text = izz, ixx
    # Contact shape can differ from the visual: 'sphere' is the most robust against height fields.
    contact_shape, contact_pose = shape, geo_pose
    if not w.get('caster') and w.get('collision', 'cylinder') == 'sphere':
        contact_shape, contact_pose = {'sphere': float(w['radius'])}, sdf.pose([0, 0, 0])
    link = el('link', link_pose, inertial,
              el('collision', contact_pose, sdf.geometry(contact_shape), sdf.surface(friction, w.get('contact')),
                 name='collision'),
              el('visual', sdf.pose([0, 0, 0, math.pi / 2, 0, 0]) if not w.get('caster') else sdf.pose([0, 0, 0]),
                 sdf.geometry(shape), sdf.color(w.get('color', [0.1, 0.1, 0.1])), name='visual'),
              name=w['name'])
    return [link], [joint]


def _sensor(s, t):
    kind = s['type']
    p = sdf.pose(list(s.get('position', [0, 0, 0])) + list(s.get('rpy', [0, 0, 0])))
    common = [p, el('always_on', text=True), el('update_rate', text=float(s.get('rate', 30))),
              el('topic', text=t['sensor'](s['name'])), el('visualize', text=s.get('visualize', False))]
    if kind == 'imu':
        body = el('imu', el('enable_orientation', text=True))
    elif kind == 'camera':
        body = el('camera',
                  el('horizontal_fov', text=float(s.get('hfov', 1.2))),
                  el('image', el('width', text=int(s.get('width', 640))), el('height', text=int(s.get('height', 480))),
                     el('format', text=s.get('format', 'R8G8B8'))),
                  el('clip', el('near', text=s.get('near', 0.05)), el('far', text=s.get('far', 100.0))))
    elif kind == 'gpu_lidar':
        def scan_dim(tag, d):
            return el(tag, el('samples', text=int(d.get('samples', 1))), el('resolution', text=1),
                      el('min_angle', text=float(d.get('min_angle', 0))),
                      el('max_angle', text=float(d.get('max_angle', 0))))
        body = el('lidar',
                  el('scan', scan_dim('horizontal', s.get('horizontal', {'samples': 360, 'min_angle': -math.pi,
                                                                         'max_angle': math.pi})),
                     scan_dim('vertical', s.get('vertical', {}))),
                  el('range', el('min', text=s.get('min_range', 0.1)), el('max', text=s.get('max_range', 20.0)),
                     el('resolution', text=0.01)))
    else:
        raise ValueError(f'unknown sensor type: {kind}')
    return el('sensor', common, body, name=s['name'], type=kind)


def build(cfg, out_dir):
    """Write ``<out_dir>/robot.sdf`` and return the robot's interface description."""
    r = cfg['robot']
    name = r['name']
    t = topics(name)
    kind = r.get('type', 'wheeled')
    wheels = r.get('wheels', [])
    driven = [w['name'] for w in wheels if w.get('driven')]
    sensors = r.get('sensors', [])
    path = os.path.join(out_dir, 'robot.sdf')

    if kind == 'file':
        shutil.copyfile(r['path'], path)
        driven = r.get('driven_joints', [])
        joints = driven
    elif kind == 'wheeled':
        base = _base_link(r)
        base.extend(_sensor(s, t) for s in sensors)
        act = r.get('actuators', {})
        torque = float(act.get('max_torque', 5.0))
        links, joint_els = [], []
        for w in wheels:
            wheel_links, wheel_joints = _wheel(w, torque)
            links += wheel_links
            joint_els += wheel_joints
        joints = [f'{n}_joint' for n in driven]
        plugins = []
        for j in joints:
            if act.get('mode', 'servo') == 'servo':
                # Velocity servo: tracks the command exactly while the needed torque stays under
                # the joint effort limit, then gives way (stepper-like; stalls show as slip).
                plugins.append(sdf.plugin('gz-sim-joint-controller-system', 'gz::sim::systems::JointController',
                                          joint_name=j, topic=t['joint_cmd'](j)))
            else:  # 'pid': torque from a velocity PID, saturating at max_torque (DC-motor-like)
                pid = act.get('pid', {'p': 1.0})
                plugins.append(sdf.plugin('gz-sim-joint-controller-system', 'gz::sim::systems::JointController',
                                          joint_name=j, topic=t['joint_cmd'](j), use_force_commands=True,
                                          p_gain=pid.get('p', 1.0), i_gain=pid.get('i', 0.0),
                                          d_gain=pid.get('d', 0.0), i_max=torque, i_min=-torque,
                                          cmd_max=torque, cmd_min=-torque))
        gt = r.get('ground_truth', {})
        plugins += [
            sdf.plugin('gz-sim-odometry-publisher-system', 'gz::sim::systems::OdometryPublisher',
                       odom_frame='world', robot_base_frame=f'{name}/base_link',
                       odom_publish_frequency=float(gt.get('rate', 50)), odom_topic=t['ground_truth'],
                       dimensions=3, tf_topic=t['ground_truth'] + '/tf'),
            sdf.plugin('gz-sim-joint-state-publisher-system', 'gz::sim::systems::JointStatePublisher',
                       topic=t['joint_states']),
        ]
        model = el('model', base, links, joint_els, plugins, name=name)
        sdf.write(sdf.document(model), path)
    else:
        raise ValueError(f'unknown robot type: {kind}')

    return {
        'name': name,
        'sdf': path,
        'driven_wheels': driven,
        'joints': joints,
        'wheels': {w['name']: w for w in wheels},
        'sensors': sensors,
        'topics': t,
        'spawn': r.get('spawn', {}),
        'lowest_point': min([w['position'][2] - w['radius'] for w in wheels] or [0.0]),
    }
