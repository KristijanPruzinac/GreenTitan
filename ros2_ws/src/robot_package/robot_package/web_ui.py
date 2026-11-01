"""Drive page: joystick and live top-down view in a browser (phone or PC), at http://<host>:8080/.

    ros2 run robot_package web_ui                                   # real mower
    ros2 run robot_package web_ui --ros-args -p world:=<world.yaml> # HIL: also draw the true objects

Commands go to the firmware console (esp/console) as if typed there: the stick as "v<lin>,<ang>\\n"
at a steady 10 Hz from this node's own timer while the page reports it held (the page sends the
stick at 10 Hz; if that stops for STICK_TIMEOUT, e.g. Wi-Fi drops, the node sends one stop, as on
release). Buttons map to fixed keys (KEYS); nothing else is passed on.

State shown: the EKF pose (/odometry/filtered/global, base_link = antenna), esp/log (last lines,
the wall-assist state, manual/assist flags, the outline being recorded), esp/wall_map (typed
objects, format below), and with `world` the true objects (sim_env obstacles.expand) and the
true mower (/sim/greentitan/ground_truth), moved into the odom frame through the datum.

esp/wall_map v1 (Int16MultiArray): [1, n, then per object kind, x0, y0, x1, y1, height, hits],
cm in odom; kind 0 wall, 1 fence, 2 low edge, 3 hedge, 4 post (x0, y0 centre, x1 radius).

Only Python stdlib + rclpy: no rosbridge, no external JS for the 2D page, works offline (the optional
3D view loads three.js from cdnjs the first time it is opened).
"""
import collections
import json
import math
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Int16MultiArray, String

MAX_LIN, MAX_REV, MAX_ANG = 0.3, 0.1, 0.6   # m/s forward, m/s reverse, rad/s
STICK_TIMEOUT = 0.6                          # s without a stick report from the page -> stop (0.35 stopped on Wi-Fi jitter)
LOG_LINES = 30
KEYS = {'manual': 'm', 'assist': 'f', 'status': 'i', 'stop': ' '}
METERS_PER_LAT = 111111.0                    # the firmware's flat-earth conversion (gps_topic_callback)

RE_WALL_STATE = re.compile(r'\[WALL\] (\w+) -> (\w+): (.*)')
RE_POINT = re.compile(r'\[(?:WALL|DEBUG)\] Point added \((-?\d+), (-?\d+)\) cm( \(corner\))?')
RE_DATUM = re.compile(r'(?:Datum published|Datum set to): lat=(-?[\d.]+) lon=(-?[\d.]+)')


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _page_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('robot_package'), 'web', 'drive.html')
        if os.path.exists(path):
            return path
    except Exception:  # not installed: run from the source tree
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'web', 'drive.html')


class World:
    """True objects of a sim world YAML as 2D footprints with a height range, in the odom frame of a datum."""

    def __init__(self, path):
        from sim_env import config, obstacles  # only with a world: the phone has no sim_env
        from sim_env.world import enu_to_geodetic
        cfg = config.load(path)
        self.origin = cfg['world']['origin']
        self.enu_to_geodetic = enu_to_geodetic
        # (kind, ('poly', [(x, y)...]) | ('circle', (x, y), r), (z bottom, z top)) in world metres
        self.parts = []
        for obj in cfg.get('objects', []):
            parts, _ = obstacles.expand(obj, lambda x, y: 0.0)
            for p in parts:
                kind = p.get('kind') or obj.get('type') or obj.get('name', 'object')
                x, y, z, yaw = p['pose'][0], p['pose'][1], p['pose'][2], p['pose'][5]
                if 'box' in p:
                    hx, hy, hz = p['box'][0] / 2, p['box'][1] / 2, p['box'][2] / 2
                    c, s = math.cos(yaw), math.sin(yaw)
                    self.parts.append((kind, ('poly', [(x + c * u - s * v, y + s * u + c * v)
                                                       for u, v in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]),
                                       (z - hz, z + hz)))
                elif 'cylinder' in p or 'sphere' in p:
                    r = p['cylinder'][0] if 'cylinder' in p else p['sphere']
                    hz = p['cylinder'][1] / 2 if 'cylinder' in p else r
                    self.parts.append((kind, ('circle', (x, y), r), (z - hz, z + hz)))
        self.datum = None  # (lat, lon); None = assume the world origin

    def to_odom(self, e, n):
        """World ENU metres -> the firmware's odom metres (flat earth from the datum)."""
        lat, lon, _ = self.enu_to_geodetic(self.origin, e, n, 0.0)
        lat0, lon0 = self.datum or (self.origin['latitude'], self.origin['longitude'])
        return (lon - lon0) * METERS_PER_LAT * math.cos(math.radians(lat0)), (lat - lat0) * METERS_PER_LAT

    def footprints(self):
        out = []
        for kind, g, zr in self.parts:
            z = [round(v, 3) for v in zr]  # bottom, top (m above the world ground; 3D view)
            if g[0] == 'poly':
                out.append({'k': kind, 'p': [[round(v, 3) for v in self.to_odom(*q)] for q in g[1]], 'z': z})
            else:
                out.append({'k': kind, 'c': [round(v, 3) for v in self.to_odom(*g[1])], 'r': round(g[2], 3), 'z': z})
        return out


class WebUi(Node):
    def __init__(self):
        super().__init__('web_ui')
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('world', '')
        self.lock = threading.Lock()
        self.log = collections.deque(maxlen=LOG_LINES)
        self.log_id = 0
        self.log_tail = ''
        self.t_log = self.t_pose = self.t_imu = None
        self.pose = None
        self.truth = None
        self.sim_t = collections.deque(maxlen=150)  # (sim stamp, monotonic) of the last ~3 s of truth
        self.wall_map = []
        self.t_map = None
        self.manual = None   # None = not known yet (no log line seen)
        self.assist = None
        self.assist_state = 'FREE'
        self.assist_why = ''
        self.recording = False
        self.outlines = []   # [[[x, y, corner], ...], ...] cm
        self.stick = None    # (lin, ang, monotonic time of the last report) while held
        self.notice = ''

        self.world = None
        self.world_rev = 0
        path = self.get_parameter('world').value
        if path:
            try:
                self.world = World(path)
                self.world_rev = 1
                self.get_logger().info(f'world {path}: {len(self.world.parts)} parts')
            except Exception as e:  # the page must work without the overlay
                self.get_logger().error(f'world {path} not loaded: {e}')

        self.console = self.create_publisher(String, 'esp/console', 10)
        self.create_subscription(String, 'esp/log', self.on_log, 50)
        self.create_subscription(Odometry, '/odometry/filtered/global', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Int16MultiArray, 'esp/wall_map', self.on_map, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, 'esp/datum', self.on_datum, 10)
        if self.world:
            self.create_subscription(Odometry, '/sim/greentitan/ground_truth', self.on_truth, qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)

    # ------------------------------------------------------------------ ROS in
    def on_log(self, msg):
        now = time.monotonic()
        with self.lock:
            self.t_log = now
            text = self.log_tail + msg.data.replace('\r', '')
            lines = text.split('\n')
            self.log_tail = lines.pop()  # a line may continue in the next message
            for line in lines:
                if line:
                    self.parse(line)

    def parse(self, line):
        self.log_id += 1
        self.log.append((self.log_id, line))
        m = RE_WALL_STATE.search(line)
        if m:
            self.assist_state, self.assist_why = m.group(2), m.group(3)
        m = RE_POINT.search(line)
        if m:
            self.add_point(int(m.group(1)), int(m.group(2)), bool(m.group(3)))
        elif line.startswith('[DEBUG] Point added') and self.pose:  # 'p' at the mower position
            self.add_point(round(self.pose['x'] * 100), round(self.pose['y'] * 100), False)
        m = RE_DATUM.search(line)
        if m:
            self.set_datum(float(m.group(1)), float(m.group(2)))
        if 'Manual mode ON' in line:
            self.manual = True
        elif 'Manual mode OFF' in line or 'Not in manual mode' in line:
            self.manual = False
            self.stick = None
        if '[WALL] Assist ON' in line or 'Assist ready (on' in line:
            self.assist = True
        elif '[WALL] Assist OFF' in line or 'Assist ready (off' in line or 'Assist unavailable' in line:
            self.assist = False
        if 'Path capture started' in line:
            self.recording, self.outlines = True, []
        elif 'New outline started' in line:
            self.outlines.append([])
        elif 'Path capture completed' in line or 'Path capture failed' in line:
            self.recording = False

    def add_point(self, x, y, corner):
        if not self.outlines:
            self.outlines.append([])
        self.outlines[-1].append([x, y, corner])

    def on_datum(self, m):
        with self.lock:
            self.set_datum(m.latitude, m.longitude)

    def set_datum(self, lat, lon):
        # esp/datum has full precision, a log line 7 decimals (~1 cm); the latest one counts
        if self.world and (lat, lon) != self.world.datum:
            self.world.datum = (lat, lon)
            self.world_rev += 1

    def on_pose(self, m):
        p = m.pose.pose.position
        with self.lock:
            self.t_pose = time.monotonic()
            self.pose = {'x': p.x, 'y': p.y, 'yaw': _yaw(m.pose.pose.orientation),
                         'v': m.twist.twist.linear.x, 'w': m.twist.twist.angular.z}

    def on_imu(self, _):
        self.t_imu = time.monotonic()

    def on_truth(self, m):
        p = m.pose.pose.position
        x, y = self.world.to_odom(p.x, p.y)
        now = time.monotonic()
        with self.lock:
            self.truth = {'x': x, 'y': y, 'yaw': _yaw(m.pose.pose.orientation), 't': now}
            # Sim speed: sim stamp against wall time. The firmware runs on wall time, so below ~0.95x
            # the HIL loop is wrong (gyro turns over-counted, steppers slow): the page must show it
            self.sim_t.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, now))

    def rtf(self):
        if len(self.sim_t) < 2 or self.sim_t[-1][1] - self.sim_t[0][1] < 1.0:
            return None
        (s0, w0), (s1, w1) = self.sim_t[0], self.sim_t[-1]
        return round((s1 - s0) / (w1 - w0), 2)

    def on_map(self, m):
        d = list(m.data)
        if len(d) < 2 or d[0] != 1:
            return
        n = min(d[1], (len(d) - 2) // 7)
        objs = [d[2 + 7 * i: 9 + 7 * i] for i in range(n)]
        with self.lock:
            self.wall_map, self.t_map = objs, time.monotonic()

    # ------------------------------------------------------------------ console out
    def send(self, keys):
        self.console.publish(String(data=keys))

    def tick(self):
        """10 Hz: the held stick as a v command, one stop when it is released or goes silent."""
        with self.lock:
            stick = self.stick
            if stick and time.monotonic() - stick[2] > STICK_TIMEOUT:
                self.stick, stick = None, None
                self.notice = 'stick lost: stopped'
                release = True
            else:
                release = False
        if stick:
            self.send(f'v{stick[0]:.2f},{stick[1]:.2f}\n')
        elif release:
            self.send(' ')

    def drive(self, lin, ang, held):
        with self.lock:
            was = self.stick is not None
            if held and self.manual is not False:
                self.stick = (max(-MAX_REV, min(MAX_LIN, lin)), max(-MAX_ANG, min(MAX_ANG, ang)), time.monotonic())
                self.notice = ''
                return 'ok'
            self.stick = None
            if held:
                self.notice = 'manual is off'
                return 'manual off'
        if was:
            self.send(' ')
        return 'ok'

    def command(self, name):
        if name not in KEYS:
            return False
        if name == 'stop':
            with self.lock:
                self.stick = None
        self.send(KEYS[name])
        return True

    # ------------------------------------------------------------------ HTTP out
    def state(self, since):
        now = time.monotonic()
        age = lambda t: None if t is None else round(now - t, 2)  # noqa: E731
        with self.lock:
            return {
                'pose': self.pose, 'age_pose': age(self.t_pose), 'age_log': age(self.t_log), 'age_imu': age(self.t_imu),
                'truth': self.truth if self.truth and now - self.truth['t'] < 1.0 else None,
                'rtf': self.rtf() if self.truth and now - self.truth['t'] < 1.0 else None,
                'manual': self.manual, 'assist': self.assist, 'assist_state': self.assist_state,
                'assist_why': self.assist_why, 'recording': self.recording, 'outlines': self.outlines,
                'map': self.wall_map, 'age_map': age(self.t_map), 'world_rev': self.world_rev,
                'held': self.stick is not None, 'notice': self.notice,
                'log': [[i, s] for i, s in self.log if i > since], 'log_id': self.log_id,
            }

    def world_json(self):
        with self.lock:
            if not self.world:
                return {'rev': 0, 'parts': []}
            return {'rev': self.world_rev, 'parts': self.world.footprints(), 'datum': self.world.datum is not None}


def make_handler(ui, page_path):
    class Handler(BaseHTTPRequestHandler):
        # Keep-alive: with HTTP/1.0 every poll and stick report opened a new connection, and from Windows into
        # WSL each one can stall (2 s for "localhost": IPv6 first), long enough for the stick timeout to stop
        # the mower again and again (manual drive 2026-11-01). Every reply sets Content-Length.
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):  # quiet: the page polls 7x per second
            pass

        def reply(self, code, body, ctype='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body, separators=(',', ':')).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path, _, query = self.path.partition('?')
            if path in ('/', '/index.html'):
                with open(page_path, 'rb') as f:
                    self.reply(200, f.read(), 'text/html; charset=utf-8')
            elif path == '/state':
                since = 0
                for kv in query.split('&'):
                    if kv.startswith('since='):
                        since = int(kv[6:] or 0) if kv[6:].lstrip('-').isdigit() else 0
                self.reply(200, ui.state(since))
            elif path == '/world':
                self.reply(200, ui.world_json())
            else:
                self.reply(404, {'error': 'not found'})

        def do_POST(self):
            try:
                n = int(self.headers.get('Content-Length', 0))
                body = json.loads(self.rfile.read(n) or b'{}')
            except (ValueError, json.JSONDecodeError):
                return self.reply(400, {'error': 'bad json'})
            if self.path == '/drive':
                try:
                    r = ui.drive(float(body.get('lin', 0)), float(body.get('ang', 0)), bool(body.get('held')))
                except (TypeError, ValueError):
                    return self.reply(400, {'error': 'bad values'})
                self.reply(200, {'result': r})
            elif self.path == '/cmd':
                ok = ui.command(str(body.get('cmd', '')))
                self.reply(200 if ok else 400, {'result': 'ok' if ok else 'unknown command'})
            else:
                self.reply(404, {'error': 'not found'})
    return Handler


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl-C: one KeyboardInterrupt in spin
    ui = WebUi()
    host, port = ui.get_parameter('host').value, int(ui.get_parameter('port').value)
    server = ThreadingHTTPServer((host, port), make_handler(ui, _page_path()))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    ui.get_logger().info(f'drive page on http://{host}:{port}/')
    try:
        rclpy.spin(ui)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # the firmware stops by itself 500 ms after the last v command
    finally:
        server.server_close()  # the serving thread is a daemon; shutdown() would wait for its poll
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
