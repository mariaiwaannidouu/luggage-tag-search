#!/usr/bin/env python3
"""
Ανεξάρτητος κόμβος: ενημερώνει τα collision objects των βαλιτσών στο planning
scene με σταθερό ρυθμό, ώστε να μη "παγώνουν" όσο ο βραχίονας εκτελεί τροχιά.
Χρησιμοποιεί το /luggage/true_poses (ground truth από τον mover) αν υπάρχει,
αλλιώς πέφτει πίσω στο /luggage/tag_detections (μόνο ό,τι βλέπει η κάμερα).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import MarkerArray
import math

# σειρά ίδια με ENTITIES του luggage_mover
DIMS_BY_INDEX = [
    ('luggage',   (0.203, 0.300, 0.112)),
    ('luggage_2', (0.246, 0.363, 0.136)),
    ('luggage_3', (0.338, 0.499, 0.187)),
]
BODY_IDS = {0: 0, 1: 1, 2: 2}   # marker id -> index

PAD = 0.03


class CollisionPublisher(Node):
    def __init__(self):
        super().__init__('collision_publisher')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('pad', PAD)
        self.declare_parameter('use_truth', False)

        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.create_subscription(PoseArray, '/luggage/true_poses', self.truth_cb, 10)
        self.create_subscription(MarkerArray, '/luggage/tag_detections',
                                 self.det_cb, 10)

        self.truth = None
        self.det = {}
        self.ground_sent = False
        self.n = 0

        r = self.get_parameter('rate').value
        self.create_timer(1.0 / max(1.0, r), self.tick)
        self.get_logger().info(f'Collision publisher ξεκίνησε στα {r:.0f}Hz')

    def truth_cb(self, msg):
        self.truth = msg.poses

    def det_cb(self, msg):
        self.det = {}
        for m in msg.markers:
            if m.id in BODY_IDS:
                q = m.pose.orientation
                yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                                 1.0 - 2.0*(q.y*q.y + q.z*q.z))
                self.det[BODY_IDS[m.id]] = (m.pose.position.x, m.pose.position.y,
                                            m.pose.position.z, yaw, True)

    def make_box(self, oid, x, y, z_center, yaw, dims):
        pad = self.get_parameter('pad').value
        co = CollisionObject()
        co.header.frame_id = 'base_link'
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = oid
        b = SolidPrimitive()
        b.type = SolidPrimitive.BOX
        b.dimensions = [dims[0]+pad, dims[1]+pad, dims[2]+pad]
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z_center)
        p.orientation.z = math.sin(yaw/2.0)
        p.orientation.w = math.cos(yaw/2.0)
        co.primitives.append(b)
        co.primitive_poses.append(p)
        co.operation = CollisionObject.ADD
        return co

    def tick(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        if not self.ground_sent:
            co = CollisionObject()
            co.header.frame_id = 'base_link'
            co.id = 'ground_plane'
            b = SolidPrimitive()
            b.type = SolidPrimitive.BOX
            b.dimensions = [4.0, 4.0, 0.02]
            p = Pose()
            p.position.z = -0.011
            p.orientation.w = 1.0
            co.primitives.append(b)
            co.primitive_poses.append(p)
            co.operation = CollisionObject.ADD
            scene.world.collision_objects.append(co)
            self.ground_sent = True
            self.get_logger().info('Δάπεδο προστέθηκε')

        use_truth = self.get_parameter('use_truth').value
        src = 'truth'
        if use_truth and self.truth:
            for i, pose in enumerate(self.truth):
                if i >= len(DIMS_BY_INDEX):
                    break
                name, dims = DIMS_BY_INDEX[i]
                q = pose.orientation
                yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                                 1.0 - 2.0*(q.y*q.y + q.z*q.z))
                # ο mover δίνει το ΚΕΝΤΡΟ της βαλίτσας
                scene.world.collision_objects.append(
                    self.make_box(f'suitcase_{i}', pose.position.x,
                                  pose.position.y, pose.position.z, yaw, dims))
        elif self.det:
            src = 'detection'
            for i, (x, y, z, yaw, _) in self.det.items():
                name, dims = DIMS_BY_INDEX[i]
                # το tag είναι στην ΚΟΡΥΦΗ -> κέντρο μισό ύψος χαμηλότερα
                scene.world.collision_objects.append(
                    self.make_box(f'suitcase_{i}', x, y,
                                  max(dims[2]/2.0, z - dims[2]/2.0), yaw, dims))
        else:
            return

        self.scene_pub.publish(scene)
        self.n += 1
        if self.n % 50 == 0:
            self.get_logger().info(
                f'{len(scene.world.collision_objects)} κουτιά ({src})')


def main(args=None):
    rclpy.init(args=args)
    node = CollisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
