import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        self.frame_id = 'map'
        self.outlines_pub = self.create_publisher(MarkerArray, 'algo/markers/outlines', 10)
        self.infill_pub = self.create_publisher(MarkerArray, 'algo/markers/infill', 10)
        self.obstacles_pub = self.create_publisher(MarkerArray, 'algo/markers/obstacles', 10)
        self.create_subscription(Float64MultiArray, 'algo/path_data', self.on_data, 10)
        self.get_logger().info('Path visualizer ready')

    def on_data(self, msg):
        d = list(msg.data)
        i = 0

        outline_count = int(d[i]); i += 1
        outlines = []
        for _ in range(outline_count):
            n = int(d[i]); i += 1
            pts = [(d[i + 2 * k], d[i + 2 * k + 1]) for k in range(n)]
            i += 2 * n
            outlines.append(pts)

        scanline_count = int(d[i]); i += 1
        scanlines = []
        for _ in range(scanline_count):
            m = int(d[i]); i += 1
            pts = [(d[i + 2 * k], d[i + 2 * k + 1]) for k in range(m)]
            i += 2 * m
            scanlines.append(pts)

        self.outlines_pub.publish(self._outlines(outlines))
        self.infill_pub.publish(self._infill(scanlines))
        self.obstacles_pub.publish(self._obstacles(outlines))
        self.get_logger().info(f'Drew {len(outlines)} outlines, {len(scanlines)} scan lines')

    def _outlines(self, outlines):
        arr = MarkerArray()
        for idx, pts in enumerate(outlines):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.ns = 'outlines'
            m.id = idx
            m.type = Marker.LINE_STRIP
            m.scale.x = 0.02
            m.color = ColorRGBA(r=0.1, g=0.9, b=0.1, a=1.0)
            m.pose.orientation.w = 1.0
            m.pose.position.z = -0.02
            m.points = [Point(x=x, y=y, z=0.0) for x, y in pts]
            if pts:
                m.points.append(Point(x=pts[0][0], y=pts[0][1], z=0.0))
            arr.markers.append(m)
        return arr

    def _infill(self, scanlines):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.ns = 'infill'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.scale.x = 0.015
        m.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.8)
        m.pose.orientation.w = 1.0
        m.pose.position.z = -0.1
        for pts in scanlines:
            for k in range(0, len(pts) - 1, 2):
                m.points.append(Point(x=pts[k][0],     y=pts[k][1],     z=0.0))
                m.points.append(Point(x=pts[k + 1][0], y=pts[k + 1][1], z=0.0))
        arr = MarkerArray()
        arr.markers.append(m)
        return arr

    def _obstacles(self, outlines):
        arr = MarkerArray()
        for idx, pts in enumerate(outlines[1:], start=1):
            if len(pts) < 3:
                continue
            m = Marker()
            m.header.frame_id = self.frame_id
            m.ns = 'obstacles'
            m.id = idx - 1
            m.type = Marker.TRIANGLE_LIST
            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 1.0
            m.color = ColorRGBA(r=0.9, g=0.2, b=0.2, a=0.5)
            m.pose.orientation.w = 1.0
            m.pose.position.z = -0.05
            for t in range(len(pts) - 2):
                m.points.append(Point(x=pts[0][0],     y=pts[0][1],     z=0.0))
                m.points.append(Point(x=pts[t + 1][0], y=pts[t + 1][1], z=0.0))
                m.points.append(Point(x=pts[t + 2][0], y=pts[t + 2][1], z=0.0))
            arr.markers.append(m)
        return arr


def main():
    rclpy.init()
    try:
        rclpy.spin(PathVisualizer())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
