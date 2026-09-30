#!/usr/bin/env python3
"""
Ταινία stop-and-go κατά -X. Οι βαλίτσες κινούνται μαζί, σταματούν κοντά στον
βραχίονα (σε διαφορετικό σημείο κάθε φορά), περιμένουν, και συνεχίζουν.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import math
import random
import subprocess

ENTITIES = [
    ('luggage',   0.0561),
    ('luggage_2', 0.0679),
    ('luggage_3', 0.0933),
]

# Διαδοχικά σημεία στάσης κοντά στον βραχίονα (κυκλικά)
STOP_POINTS = [0.0]   # όλες σταματούν ΜΠΡΟΣΤΑ στον βραχίονα


class LuggageMover(Node):
    def __init__(self):
        super().__init__('luggage_mover')

        self.declare_parameter('world', 'empty')
        self.declare_parameter('y', 0.60)
        self.declare_parameter('x_start', 2.70)    # ξεκινούν δεξιά, εκτός πεδίου
        self.declare_parameter('x_end', -2.70)     # φεύγουν αριστερά
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('spacing', 1.80)
        self.declare_parameter('dwell', 90.0)      # δευτ. στάσης
        self.declare_parameter('pause', False)
        self.declare_parameter('random_stops', False)

        self.declare_parameter('yaw_1', 0.0)
        self.declare_parameter('yaw_2', 45.0)
        self.declare_parameter('yaw_3', -90.0)

        self.declare_parameter('face_arm_3', True)
        self.declare_parameter('handle_local_deg', 0.0)
        self.declare_parameter('jitter_deg', 35.0)

        self.dt = 0.05
        self.s = 0.0               # απόσταση που διένυσε η ταινία
        self.s_last_resume = 0.0
        self.stopped = False
        self.stop_t = 0.0
        self.stop_ptr = 0
        self.prev_x = {}
        self.laps = [-1, -1, -1]
        self.warned = False

        self.pub = self.create_publisher(PoseArray, '/luggage/true_poses', 10)
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'Ταινία stop-and-go. Στάσεις στο x={STOP_POINTS}, '
            f'αναμονή {self.p("dwell")}s')

    def p(self, n):
        return self.get_parameter(n).value

    def target_x(self):
        if self.p('random_stops'):
            return self.stop_target
        return STOP_POINTS[self.stop_ptr % len(STOP_POINTS)]

    def next_stop(self):
        self.stop_ptr += 1
        if self.p('random_stops'):
            self.stop_target = random.uniform(-0.35, 0.35)

    def facing_yaw(self):
        theta = math.atan2(-self.p('y'), 0.0)
        phi = math.radians(self.p('handle_local_deg'))
        j = self.p('jitter_deg')
        return math.degrees(theta - phi + math.radians(random.uniform(-j, j)))

    def tick(self):
        if not hasattr(self, 'stop_target'):
            self.stop_target = STOP_POINTS[0]

        xs, xe = self.p('x_start'), self.p('x_end')
        total = max(0.01, abs(xs - xe))
        y, spacing = self.p('y'), self.p('spacing')
        now = self.get_clock().now().nanoseconds / 1e9

        # --- χρονισμός ταινίας ---
        if self.stopped:
            if now - self.stop_t >= self.p('dwell'):
                self.stopped = False
                self.s_last_resume = self.s
                self.next_stop()
                self.get_logger().info(
                    f'ΞΕΚΙΝΑ ΞΑΝΑ - επόμενη στάση x={self.target_x():+.2f}')
        elif not self.p('pause'):
            self.s += self.p('speed') * self.dt

        arr = PoseArray()
        arr.header.frame_id = 'base_link'
        arr.header.stamp = self.get_clock().now().to_msg()
        report = []
        tx = self.target_x()
        guard_ok = (self.s - self.s_last_resume) > 0.4 * spacing

        for i, (name, z) in enumerate(ENTITIES):
            travelled = self.s + i * spacing
            lap = int(travelled // total)
            x = xs - (travelled % total)          # κίνηση προς -X

            if lap != self.laps[i]:
                self.laps[i] = lap
                if i == 2 and self.p('face_arm_3'):
                    ny = self.facing_yaw()
                    self.set_parameters([rclpy.parameter.Parameter(
                        'yaw_3', rclpy.Parameter.Type.DOUBLE, float(ny))])
                    self.get_logger().info(
                        f'{name}: νέο πέρασμα, χερούλι προς βραχίονα yaw={ny:.0f}°')

            # --- ανίχνευση άφιξης σε σημείο στάσης ---
            px = self.prev_x.get(name)
            if (not self.stopped and guard_ok and px is not None
                    and px > tx >= x and abs(px - x) < 0.5):
                self.stopped = True
                self.stop_t = now
                self.get_logger().info(
                    f'>>> ΣΤΑΣΗ: {name} στο x={x:+.2f} y={y:.2f} '
                    f'yaw={self.p(f"yaw_{i+1}"):+.0f}° - αναμονή {self.p("dwell")}s')
            self.prev_x[name] = x

            yaw = math.radians(self.p(f'yaw_{i+1}'))
            self.move_entity(name, x, y, z, yaw)

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            arr.poses.append(pose)

            report.append(f'{name}: x={x:+.2f} yaw={math.degrees(yaw):+.0f}° '
                          f'd={math.sqrt(x*x + y*y + z*z):.2f}m')

        self.pub.publish(arr)
        if self.stopped:
            state = f'[ΣΤΑΣΗ {self.p("dwell") - (now - self.stop_t):.0f}s]'
        elif self.p('pause'):
            state = '[ΠΑΥΣΗ]'
        else:
            state = '[ΚΙΝΗΣΗ]'
        self.get_logger().info(f'{state} ' + ' | '.join(report),
                               throttle_duration_sec=2.0)

    def move_entity(self, name, x, y, z, yaw):
        req = (f'name: "{name}" position: {{x: {x}, y: {y}, z: {z}}} '
               f'orientation: {{x: 0, y: 0, z: {math.sin(yaw/2):.6f}, '
               f'w: {math.cos(yaw/2):.6f}}}')
        try:
            r = subprocess.run(
                ['gz', 'service', '-s', f'/world/{self.p("world")}/set_pose',
                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '300', '--req', req],
                capture_output=True, timeout=1.0, text=True)
            if r.returncode != 0 and not self.warned:
                self.warned = True
                self.get_logger().error(f'gz service απέτυχε: {r.stderr.strip()}')
        except Exception as e:
            if not self.warned:
                self.warned = True
                self.get_logger().error(f'gz κλήση απέτυχε: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LuggageMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
