#!/usr/bin/env python3
"""
Πολλαπλή ανίχνευση ArUco με ΠΛΗΡΕΣ 6D pose (θέση + quaternion).
Θέση: από point cloud (επικυρωμένη). Προσανατολισμός: από solvePnP.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import numpy as np
import struct, math
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
import tf2_geometry_msgs  # noqa: F401
import cv2

ARUCO_DICT = cv2.aruco.DICT_4X4_50
TARGET_FRAME = 'base_link'
RAW_POINT_FRAME = 'gemini_camera_link'          # για το cloud (φυσική σύμβαση)
OPTICAL_FRAME = 'gemini_camera_link_optical'    # για solvePnP (optical σύμβαση)

# Εφεδρικά intrinsics από το URDF: hfov=1.64rad, 1280x800
FALLBACK_W, FALLBACK_H, FALLBACK_HFOV = 1280, 800, 1.64

TAG_REGISTRY = {
    # 'normal': πού δείχνει ΦΥΣΙΚΑ η κάθετος του tag. Λύνει την ασάφεια του
    #   solvePnP (βλ. pick_orientation).  'up' = κείτεται επίπεδο, κοιτάζει
    #   ψηλά.  'side' = κατακόρυφη επιφάνεια, κάθετος οριζόντια.
    #   Αν λείψει -> κρατάμε την επιλογή του OpenCV (παλιά συμπεριφορά).
    0: {'role': 'body', 'suitcase': 'luggage',   'tag_size': 0.0713, 'normal': 'up', 'dims': (0.203, 0.300, 0.112)},
    1: {'role': 'body', 'suitcase': 'luggage_2', 'tag_size': 0.0713, 'normal': 'up', 'dims': (0.246, 0.363, 0.136)},
    2: {'role': 'body', 'suitcase': 'luggage_3', 'tag_size': 0.0713, 'normal': 'up', 'dims': (0.338, 0.499, 0.187)},
    3: {'role': 'handle_front', 'tag_size': 0.0186},
    4: {'role': 'wheel_1',      'tag_size': 0.0180},
    5: {'role': 'wheel_2',      'tag_size': 0.0180},
    6: {'role': 'handle_side',  'tag_size': 0.0434},
}


def mat_to_quat(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s, 0.25*s)
    if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        return (0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s)
    if R[1,1] > R[2,2]:
        s = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        return ((R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s)
    s = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
    return ((R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s)


def quat_to_yaw(q):
    x, y, z, w = q
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))


def quat_mul(a, b):
    """a (x,y,z,w) ⊗ b (x,y,z,w). Περιστροφή του b εκφρασμένη μέσω του a."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def quat_norm(q):
    n = math.sqrt(sum(c*c for c in q))
    return tuple(c/n for c in q) if n > 1e-9 else (0.0, 0.0, 0.0, 1.0)


def orthonormal(ex, ey):
    """Δύο (θορυβώδεις) άξονες -> καθαρός πίνακας περιστροφής, Gram-Schmidt."""
    ex = np.asarray(ex, dtype=float)
    ey = np.asarray(ey, dtype=float)
    nx = float(np.linalg.norm(ex))
    if nx < 1e-6:
        return None
    ex = ex / nx
    ey = ey - ex * float(ex @ ey)
    ny = float(np.linalg.norm(ey))
    if ny < 1e-6:
        return None
    ey = ey / ny
    return np.column_stack((ex, ey, np.cross(ex, ey)))


def normal_z(q):
    """Z-συνιστώσα της καθέτου του tag (η τρίτη στήλη του R), στο base_link.
    +1 = κοιτάζει ίσια πάνω,  0 = κατακόρυφη επιφάνεια,  -1 = κάτω."""
    x, y, z, w = q
    return 1.0 - 2.0*(x*x + y*y)


def quat_to_rpy(q):
    """roll, pitch, yaw (ZYX) σε ΜΟΙΡΕΣ - διαγνωστικό.

    ΓΙΑΤΙ: για ένα tag που κείτεται ΕΠΙΠΕΔΟ στην πάνω επιφάνεια, roll και
    pitch πρέπει να είναι ~0. Μεγάλες τιμές σημαίνουν ότι ο solvePnP
    διάλεξε τη ΛΑΘΟΣ από τις δύο λύσεις-καθρέφτες (ασάφεια επίπεδου tag)
    - και τότε το σφάλμα κλίσης «περνάει» μέσα στον yaw."""
    x, y, z, w = q
    sr = 2.0*(w*x + y*z)
    cr = 1.0 - 2.0*(x*x + y*y)
    roll = math.atan2(sr, cr)
    sp = max(-1.0, min(1.0, 2.0*(w*y - z*x)))
    pitch = math.asin(sp)
    return (math.degrees(roll), math.degrees(pitch),
            math.degrees(quat_to_yaw(q)))


class ArucoMultiDetector(Node):
    def __init__(self):
        super().__init__('aruco_multi_detector')
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, cv2.aruco.DetectorParameters())

        self.declare_parameter('depth_min', 0.05)
        self.declare_parameter('depth_max', 1.9)
        self.declare_parameter('reach_min', 0.15)
        self.declare_parameter('reach_max', 2.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_cloud = None
        self.K = None
        self.dist = np.zeros((5, 1))

        self.create_subscription(PointCloud2, '/camera/points', self.cloud_cb, 1)
        self.create_subscription(Image, '/camera/image', self.image_cb, 1)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_cb, 1)

        self.markers_pub = self.create_publisher(MarkerArray, '/luggage/tag_detections', 10)
        self.get_logger().info('ArUco 6D detector started...')

    def info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(f'Intrinsics από camera_info: fx={self.K[0,0]:.1f}')

    def ensure_K(self, w, h):
        if self.K is None:
            fx = (FALLBACK_W / 2.0) / math.tan(FALLBACK_HFOV / 2.0)
            self.K = np.array([[fx, 0, w/2.0], [0, fx, h/2.0], [0, 0, 1]], dtype=np.float64)
            self.get_logger().warn(f'Χωρίς camera_info - υπολογισμένα intrinsics fx={fx:.1f}')
        return self.K

    def cloud_cb(self, msg):
        self.latest_cloud = msg

    def image_cb(self, msg):
        if self.latest_cloud is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        K = self.ensure_K(frame.shape[1], frame.shape[0])

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow('ArUco 6D Detection', frame)
        cv2.waitKey(1)

        if ids is None or len(ids) == 0:
            self.markers_pub.publish(MarkerArray())
            return

        dmin = self.get_parameter('depth_min').value
        dmax = self.get_parameter('depth_max').value
        arr, summary, rejected = MarkerArray(), [], []

        for i, mid in enumerate(ids.flatten()):
            mid = int(mid)
            info = TAG_REGISTRY.get(mid)
            if info is None:
                continue

            pt = self.get_3d_point(corners[i][0], dmin, dmax)
            if pt is None:
                rejected.append(f'id{mid}:no-depth')
                continue
            pose = self.to_base(pt, RAW_POINT_FRAME)
            if pose is None:
                rejected.append(f'id{mid}:no-tf')
                continue
            if not self.plausible(pose):
                p = pose.pose.position
                rejected.append(f'id{mid}:d={math.sqrt(p.x**2+p.y**2+p.z**2):.2f}')
                continue

            cands, pnp_pos = self.solve_orientation(corners[i][0], info['tag_size'], K)
            nz_note = ''
            if cands:
                # Ο solvePnP δίνει τον προσανατολισμό στο OPTICAL frame της
                # ΚΑΜΕΡΑΣ. Η θέση έχει ήδη μεταφερθεί στο base_link από το
                # to_base(), ο προσανατολισμός ΟΧΙ. Χωρίς αυτή τη μετατροπή
                # το μήνυμα δηλώνει frame_id=base_link αλλά κουβαλάει yaw
                # κάμερας - που ΠΕΡΙΣΤΡΕΦΕΤΑΙ κάθε φορά που κινείται ο
                # βραχίονας, ακόμα κι αν η βαλίτσα είναι ακίνητη.
                q_base, nz_note = self.pick_orientation(cands, info,
                                                        pose.header.stamp)
                # Το CLOUD έχει προτεραιότητα: δεν πάσχει από την ασάφεια.
                q_cloud = self.orientation_from_cloud(corners[i][0], dmin, dmax,
                                                      pose.header.stamp)
                if q_cloud is not None:
                    nz_note = (f'cloud={math.degrees(quat_to_yaw(q_cloud)):.1f}° '
                               f'pnp={math.degrees(quat_to_yaw(q_base)):.1f}° '
                               if q_base else
                               f'cloud={math.degrees(quat_to_yaw(q_cloud)):.1f}° ')
                    q_base = q_cloud
                else:
                    nz_note = 'ΧΩΡΙΣ-CLOUD ' + nz_note
                if q_base is not None:
                    pose.pose.orientation.x, pose.pose.orientation.y, \
                        pose.pose.orientation.z, pose.pose.orientation.w = q_base
                else:
                    # Χωρίς έγκυρο TF προτιμάμε ΚΑΝΕΝΑΝ προσανατολισμό από
                    # λάθος: ο move_arm3 ελέγχει για ταυτοτικό quaternion.
                    rejected.append(f'id{mid}:no-tf-optical')

            m = Marker()
            m.header.frame_id = TARGET_FRAME
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = info['role']
            m.id = mid
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = pose.pose
            m.scale.x = m.scale.y = info['tag_size']
            m.scale.z = 0.005
            m.color.a, m.color.g = 1.0, 1.0
            arr.markers.append(m)

            p = pose.pose.position
            q = (pose.pose.orientation.x, pose.pose.orientation.y,
                 pose.pose.orientation.z, pose.pose.orientation.w)
            _r, _p, _y = quat_to_rpy(q)
            txt = (f'{info["role"]}(id{mid}) pos=({p.x:.3f},{p.y:.3f},{p.z:.3f}) '
                   f'yaw={_y:.1f}° [{nz_note} επιλογή={normal_z(q):+.2f} '
                   f'roll={_r:.0f}° pitch={_p:.0f}°]')
            if pnp_pos is not None:
                # Τα δύο frames μοιράζονται origin (URDF joint xyz=0 0 0), οπότε
                # οι αποστάσεις από την κάμερα είναι άμεσα συγκρίσιμες.
                d_cloud = math.sqrt(pt[0]**2 + pt[1]**2 + pt[2]**2)
                d_pnp = math.sqrt(pnp_pos[0]**2 + pnp_pos[1]**2 + pnp_pos[2]**2)
                if d_pnp > 1e-6:
                    implied = info['tag_size'] * d_cloud / d_pnp
                    txt += (f' [size δηλ={info["tag_size"]*100:.1f} '
                            f'μετρ={implied*100:.1f}cm]')
            summary.append(txt)

        self.markers_pub.publish(arr)
        if summary:
            self.get_logger().info('ΒΡΕΘΗΚΑΝ: ' + ' | '.join(summary),
                                   throttle_duration_sec=1.0)
        if rejected:
            self.get_logger().warn('ΑΠΟΡΡΙΦΘΗΚΑΝ: ' + ' | '.join(rejected),
                                   throttle_duration_sec=3.0)

    def pick_orientation(self, cands, info, stamp):
        """Ποια από τις λύσεις του solvePnP είναι η σωστή;

        Ο OpenCV διαλέγει με βάση το reprojection error. Για μικρό επίπεδο
        tag σε απόσταση οι δύο λύσεις-καθρέφτες είναι σχεδόν ισοδύναμες,
        οπότε διαλέγει συχνά ΛΑΘΟΣ - και σταθερά για δεδομένη γωνία θέασης.
        Το σφάλμα βγαίνει ως κλίση (roll/pitch) και «περνάει» μέσα στον yaw.

        Όπου ξέρουμε πού δείχνει ΦΥΣΙΚΑ η κάθετος του tag (TAG_REGISTRY
        -> 'normal'), διαλέγουμε με ΑΥΤΟ αντί για το error. Το tag του
        σώματος κείτεται επίπεδο στην πάνω επιφάνεια: η κάθετός του δείχνει
        πάνω, άρα normal_z ~ +1. Η λύση-καθρέφτης δεν το ικανοποιεί.
        """
        hint = info.get('normal')
        scored = []
        for q_opt in cands:
            q_b = self.orientation_to_base(q_opt, stamp)
            if q_b is not None:
                scored.append((normal_z(q_b), q_b))
        if not scored:
            return None, ''
        # ΟΛΕΣ οι υποψήφιες στο log: αν η σωστή (nz~+1) δεν εμφανίζεται
        # καθόλου, το πρόβλημα ΔΕΝ είναι η επιλογή αλλά η εκτίμηση πόζας.
        note = 'nz=' + '/'.join(f'{z:+.2f}' for z, _ in scored)
        if hint is None:
            return scored[0][1], note
        if hint == 'up':
            pick = max(scored, key=lambda s: s[0])
        else:
            pick = min(scored, key=lambda s: abs(s[0]))
        return pick[1], note

    def orientation_to_base(self, quat, stamp, src=None):
        """Μεταφέρει προσανατολισμό από το OPTICAL frame της κάμερας στο
        base_link. Επιστρέφει (x,y,z,w) ή None αν λείπει το TF.

        ΓΙΑΤΙ ΥΠΑΡΧΕΙ: ο solvePnP δουλεύει σε συντεταγμένες ΚΑΜΕΡΑΣ. Η κάμερα
        είναι στο άκρο του βραχίονα, άρα το frame της περιστρέφεται συνεχώς.
        Ένας yaw σε camera frame που ερμηνεύεται ως base_link αλλάζει τιμή σε
        κάθε κίνηση - και τροφοδοτεί handle_face(), search_dest_angles(),
        half_extent() και τη στροφή του collision box."""
        src = src or OPTICAL_FRAME
        try:
            tf = self.tf_buffer.lookup_transform(
                TARGET_FRAME, src, stamp,
                timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(TARGET_FRAME, src, Time())
            except Exception as e:
                self.get_logger().warn(
                    f'TF {src}->{TARGET_FRAME} failed: {e}',
                    throttle_duration_sec=5.0)
                return None
        # ΟΧΙ do_transform_pose_stamped: φτιάχνει πίνακα 4x4 και μετά κάνει
        # ιδιοανάλυση (np.linalg.eigh) για να ξαναβγάλει quaternion. Είναι
        # και ακριβό και ασταθές - έσκαγε με 'Eigenvalues did not converge'.
        # Καθαρή περιστροφή = απλός πολλαπλασιασμός quaternion.
        r = tf.transform.rotation
        return quat_norm(quat_mul((r.x, r.y, r.z, r.w), quat))

    def solve_orientation(self, corners, size, K):
        h = size / 2.0
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
                       dtype=np.float64)
        img = np.array(corners, dtype=np.float64).reshape(4, 2)
        try:
            # ΓΕΝΙΚΗ μορφή: επιστρέφει ΟΛΕΣ τις λύσεις, όχι μόνο τη «νικήτρια».
            # Ένα επίπεδο tag έχει πάντα δύο λύσεις-καθρέφτες με σχεδόν ίδιο
            # reprojection error - η επιλογή με βάση το error είναι αναξιόπιστη.
            cnt, rvecs, tvecs, _err = cv2.solvePnPGeneric(
                obj, img, K, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not cnt:
                return [], None
            quats = []
            for rv in rvecs:
                R, _ = cv2.Rodrigues(rv)
                quats.append(mat_to_quat(R))
            t = tvecs[0]
            return quats, (float(t[0]), float(t[1]), float(t[2]))
        except Exception as e:
            self.get_logger().warn(f'solvePnP failed: {e}', throttle_duration_sec=5.0)
            return [], None

    def plausible(self, pose):
        p = pose.pose.position
        d = math.sqrt(p.x**2 + p.y**2 + p.z**2)
        return (self.get_parameter('reach_min').value <= d
                <= self.get_parameter('reach_max').value)

    def to_base(self, pt, src_frame):
        ps = PoseStamped()
        ps.header.stamp = self.latest_cloud.header.stamp
        ps.header.frame_id = src_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, pt)
        ps.pose.orientation.w = 1.0
        try:
            tf = self.tf_buffer.lookup_transform(
                TARGET_FRAME, src_frame, ps.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception:
            # fallback: το πιο πρόσφατο διαθέσιμο TF (ο βραχίονας είναι ακίνητος
            # κατά την παρατήρηση, οπότε το σφάλμα είναι αμελητέο)
            try:
                tf = self.tf_buffer.lookup_transform(TARGET_FRAME, src_frame, Time())
                ps.header.stamp = tf.header.stamp
            except Exception as e:
                self.get_logger().warn(f'TF failed: {e}', throttle_duration_sec=5.0)
                return None
        return tf2_geometry_msgs.do_transform_pose_stamped(ps, tf)

    def corner_3d(self, uv, ctr, dmin, dmax, shrink=0.18, win=2):
        """3D θέση ΜΙΑΣ γωνίας του tag από το point cloud.

        Δειγματοληπτεί λίγο ΜΕΣΑ από τη γωνία (shrink προς το κέντρο):
        ακριβώς πάνω στην ακμή το cloud έχει κενά και σημεία του φόντου.
        """
        u = uv[0] + (ctr[0] - uv[0]) * shrink
        v = uv[1] + (ctr[1] - uv[1]) * shrink
        c = self.latest_cloud
        xs, ys, zs = [], [], []
        for dv in range(-win, win + 1):
            for du in range(-win, win + 1):
                uu, vv = int(round(u)) + du, int(round(v)) + dv
                if not (0 <= uu < c.width and 0 <= vv < c.height):
                    continue
                off = vv * c.row_step + uu * c.point_step
                if off + 12 > len(c.data):
                    continue
                x, y, z = struct.unpack_from('fff', c.data, off)
                if np.isnan(x) or np.isnan(y) or np.isnan(z) or x < dmin or x > dmax:
                    continue
                xs.append(x); ys.append(y); zs.append(z)
        if len(xs) < 3:
            return None
        return (float(np.median(xs)), float(np.median(ys)), float(np.median(zs)))

    def orientation_from_cloud(self, corners, dmin, dmax, stamp):
        """Προσανατολισμός tag από το POINT CLOUD - ΧΩΡΙΣ solvePnP.

        ΓΙΑΤΙ ΥΠΑΡΧΕΙ: ο solvePnP σε επίπεδο tag έχει πάντα δύο λύσεις-
        καθρέφτες και διαλέγει συχνά λάθος. Το cloud δίνει τις 3D θέσεις
        των γωνιών ΑΠΕΥΘΕΙΑΣ: καμία ασάφεια, καμία εξάρτηση από το
        δηλωμένο tag_size. Είναι η ίδια πηγή που δίνει ήδη τη θέση.

        ΣΗΜΑΝΤΙΚΟ: το cloud είναι στη ΦΥΣΙΚΗ σύμβαση (RAW_POINT_FRAME),
        όχι στην optical.
        """
        if self.latest_cloud is None:
            return None
        ctr = (float(np.mean(corners[:, 0])), float(np.mean(corners[:, 1])))
        P = [self.corner_3d(corners[k], ctr, dmin, dmax) for k in range(4)]
        if any(p is None for p in P):
            return None
        # ArUco: 0=πάνω-αριστ  1=πάνω-δεξ  2=κάτω-δεξ  3=κάτω-αριστ
        # Τοπικοί άξονες του tag:  +X = 0->1,  +Y = 3->0  (μέσος όρος
        # και των δύο απέναντι ακμών, για λιγότερο θόρυβο)
        ex = [((P[1][i]-P[0][i]) + (P[2][i]-P[3][i])) * 0.5 for i in range(3)]
        ey = [((P[0][i]-P[3][i]) + (P[1][i]-P[2][i])) * 0.5 for i in range(3)]
        R = orthonormal(ex, ey)
        if R is None:
            return None
        return self.orientation_to_base(mat_to_quat(R), stamp,
                                        src=RAW_POINT_FRAME)

    def get_3d_point(self, corners, dmin, dmax):
        c = self.latest_cloud
        xs, ys, zs = [], [], []
        poly = corners.astype(np.float32)
        u0, u1 = int(np.min(poly[:,0])), int(np.max(poly[:,0]))
        v0, v1 = int(np.min(poly[:,1])), int(np.max(poly[:,1]))
        step = max(1, (u1-u0)//15)
        for v in range(v0, v1+1, step):
            for u in range(u0, u1+1, step):
                if cv2.pointPolygonTest(poly, (float(u), float(v)), False) < 0:
                    continue
                if not (0 <= u < c.width and 0 <= v < c.height):
                    continue
                off = v*c.row_step + u*c.point_step
                if off + 12 > len(c.data):
                    continue
                x, y, z = struct.unpack_from('fff', c.data, off)
                if np.isnan(x) or np.isnan(y) or np.isnan(z) or x < dmin or x > dmax:
                    continue
                xs.append(x); ys.append(y); zs.append(z)
        if len(zs) < 3:
            return None
        return (float(np.median(xs)), float(np.median(ys)), float(np.median(zs)))


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMultiDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
