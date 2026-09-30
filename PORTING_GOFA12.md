# Μεταφορά σε ABB GoFa 12 (CRB 15000-12/1.27)

Οδηγός για τον προγραμματιστή που θα περάσει το project από τον **UR5e (Gazebo)**
στον **ABB GoFa 12**. Προϋπόθεση: έχεις τρέξει την προσομοίωση UR5e με το
[`README.md`](README.md) και ξέρεις τι κάνει.

> **Κατάσταση:** τίποτα από τα παρακάτω δεν έχει δοκιμαστεί σε GoFa. Όπου γράφει
> **ΕΠΙΒΕΒΑΙΩΣΕ** ή **ΜΕΤΡΗΣΕ**, η τιμή είναι αφετηρία και όχι σωστή τιμή.

---

## 0. Η βασική ιδέα

Ο αλγόριθμος αναζήτησης (`move_arm3.py`) **δεν αλλάζει**. Ό,τι εξαρτάται από τον
βραχίονα διαβάζεται από ένα YAML (`config/gofa12.yaml`). Για τον GoFa χρειάζονται:

1. **URDF** του GoFa με την κάμερα στο άκρο → §2
2. **Πακέτο MoveIt** για αυτό το URDF → §3
3. **Launch** που τα φορτώνει → §4
4. **YAML ρυθμίσεων** με μετρημένες τιμές → §5
5. **Driver** του πραγματικού βραχίονα → §6
6. **Πραγματική κάμερα** στη θέση των γεφυρών του Gazebo → §7

Χαρακτηριστικά GoFa 12: ωφέλιμο φορτίο **12 kg**, εμβέλεια **1.27 m**
(ο UR5e: 5 kg, 0.85 m). ΕΠΙΒΕΒΑΙΩΣΕ με το datasheet της ABB.

---

## 1. Τι είναι φτιαγμένο ειδικά για τον UR5e

| Αρχείο | Τι είναι UR5e | Τι κάνεις για GoFa |
|---|---|---|
| `move_arm3.py` - μπλοκ «ΡΥΘΜΙΣΕΙΣ ΒΡΑΧΙΟΝΑ» (`PLANNING_GROUP`, `CAMERA_LINK`, `BASE_FRAME`, `SELF_LINKS`, `LINK_RADIUS`, `MAX_REACH`, `MIN_REACH`, `WORKABLE_DIST`, `IK_TIMEOUT`, `SMALL_MOVE`, `SETTLE`, `CAM_HFOV`/`CAM_W`/`CAM_H`, `WATCH_FORWARD`, `WATCH_RIGHT`, `FACE_VIEWS`, `ELEVATE_VIEWS`, `HANDLE_STANDOFFS`) | οι τιμές του UR5e ως προεπιλογές | **Καμία αλλαγή κώδικα.** Όλα μέσω `config/gofa12.yaml` (§5). |
| `move_arm3.py` - ονόματα (`self.ur5e`, κόμβος `ur5e_python_controller` στο launch) | μόνο ονόματα | τίποτα (προαιρετικά μετονομασία) |
| `launch/run_move_arm3.launch.py` | `MoveItConfigsBuilder("ur", package_name="my_ur5e_moveit_config")`, `ur5e_with_camera.xacro`, `mappings={"name": "ur", "ur_type": "ur5e"}`, `srdf/ur.srdf.xacro`, `kinematics.yaml`, `max_velocity_scaling_factor: 1.0`, `use_sim_time: True` | νέο launch (§4) |
| `urdf/ur5e_with_camera.xacro` | κάνει include το `ur_simulation_gz/urdf/ur_gz.urdf.xacro`, κάμερα στο `tool0`, αισθητήρας Gazebo | νέο xacro (§2) |
| `my_ur5e_moveit_config/` | group `ur_manipulator` (αλυσίδα `base_link` → `gripper_link`), αρθρώσεις UR, controller `scaled_joint_trajectory_controller`, KDL | νέο πακέτο MoveIt (§3) |
| `aruco_multi_detector.py` | frames `gemini_camera_link` / `gemini_camera_link_optical`, topics `/camera/*`, point cloud με **x μπροστά**, μεγέθη tags | §7 - προσοχή στο φίλτρο βάθους |
| `package.xml` | εξαρτήσεις `ur_description`, `ur_simulation_gz` | πρόσθεσε τα πακέτα του GoFa / driver |
| `luggage_mover.py`, γέφυρες `ros_gz_bridge`, `DONE_CYCLES` | μόνο προσομοίωση | δες README §9 |

---

## 2. URDF του GoFa με την κάμερα

### 2.1 Από πού παίρνεις τον GoFa 12

- **ROS-Industrial `abb`** - πακέτο `abb_crb15000_support`:
  <https://github.com/ros-industrial/abb> (φάκελος `abb_crb15000_support`).
  Περιέχει το `urdf/crb15000_12_127_macro.xacro` (macro `abb_crb15000_12_127`),
  το `urdf/crb15000_12_127.xacro` και τα meshes. Η παραλλαγή 12/1.27 προστέθηκε
  μέσω του `abb_experimental` και μεταφέρθηκε εκεί
  ([issue #202](https://github.com/ros-industrial/abb/issues/202)).
  - **Είναι πακέτο ROS 1 (catkin).** Το xacro και τα meshes είναι όμως απλά αρχεία:
    φτιάξε ένα πακέτο ROS 2 `ament_cmake` με το **ίδιο όνομα**
    (`abb_crb15000_support`) που εγκαθιστά τους φακέλους `urdf/` και `meshes/`
    στο `share/`. Έτσι τα `package://abb_crb15000_support/meshes/...` δουλεύουν
    όπως είναι.
  - Το macro κάνει include το `$(find abb_resources)/urdf/common_materials.xacro`
    (ίδιο repo) → αντέγραψέ το κι αυτό ή βάλε δικά σου υλικά.
  - Σύνδεσμοι: `base_link`, `link_1` … `link_6`, `flange`, `tool0` (και `base`).
    Αρθρώσεις: `joint_1` … `joint_6`.
  - Όρια αρθρώσεων στο macro (ΕΠΙΒΕΒΑΙΩΣΕ με το datasheet): j1 ±270°, j2 ±180°,
    j3 −225°…+85°, j4 ±180°, j5 ±180°, j6 ±270°.
- **PickNik `abb_ros2`** (<https://github.com/PickNikRobotics/abb_ros2>): drivers
  για ROS 2 (§6). Περιγραφές ρομπότ έχει μόνο για IRB1200, όχι για GoFa.

### 2.2 Νέο xacro: `urdf/gofa12_with_camera.xacro`

Αντέγραψε από το `ur5e_with_camera.xacro` **ό,τι αφορά την κάμερα και το gripper**
και άλλαξε μόνο το include του βραχίονα:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="gofa12">
  <xacro:include filename="$(find abb_crb15000_support)/urdf/crb15000_12_127_macro.xacro"/>
  <xacro:abb_crb15000_12_127 prefix=""/>

  <!-- ΚΡΑΤΑ ΤΑ ΙΔΙΑ ΟΝΟΜΑΤΑ: gemini_camera_link / gemini_camera_link_optical
       -> ο ανιχνευτής δουλεύει χωρίς αλλαγή (δες §7). -->
  <link name="gemini_camera_link"> ... ίδιο με το ur5e_with_camera.xacro ... </link>
  <joint name="camera_joint" type="fixed">
    <parent link="tool0"/>
    <child link="gemini_camera_link"/>
    <origin xyz="ΜΕΤΡΗΣΕ" rpy="ΜΕΤΡΗΣΕ"/>   <!-- UR5e: xyz="0 0.049 -0.025" rpy="0 0 1.5708" -->
  </joint>
  <link name="gemini_camera_link_optical"/>
  <joint name="gemini_camera_optical_joint" type="fixed">
    <parent link="gemini_camera_link"/>
    <child link="gemini_camera_link_optical"/>
    <origin xyz="0 0 0" rpy="-1.5707963 0 -1.5707963"/>
  </joint>
  <!-- gripper_link: ίδιο mesh, αν η βάση/gripper είναι ίδια -->
</robot>
```

- **Σύμβαση του `gemini_camera_link`:** +X = κατεύθυνση θέασης, +Z = «πάνω» της
  εικόνας. Το `move_arm3` κινεί ΑΥΤΟ το frame.
- **Θέση κάμερας:** το `origin` του `camera_joint` είναι η φυσική βάση της κάμερας
  ως προς το `tool0`. Στο ABB το `tool0` έχει Z προς τα έξω από τη φλάντζα (όπως
  και στον UR), αλλά η στροφή γύρω από το Z μπορεί να διαφέρει. **ΕΠΙΒΕΒΑΙΩΣΕ στο
  RViz** (`ros2 launch urdf_tutorial display.launch.py model:=...` ή το `demo.launch.py`
  του MoveIt) ότι ο άξονας X του `gemini_camera_link` δείχνει εκεί που κοιτά η κάμερα.
- **Προσομοίωση GoFa στο Gazebo (συνιστάται πριν τον πραγματικό):** ο UR5e έπαιρνε
  `ros2_control` + plugin Gazebo έτοιμα από το `ur_gz.urdf.xacro`. Για τον GoFa
  πρόσθεσε μόνος σου μπλοκ `<ros2_control>` με το `gz_ros2_control/GazeboSimSystem`,
  το plugin `gz_ros2_control` και ένα YAML controllers (`joint_state_broadcaster`
  + `joint_trajectory_controller` για `joint_1`…`joint_6`). Τον αισθητήρα κάμερας
  (`<gazebo reference="gemini_camera_link">`) τον αντιγράφεις όπως είναι.

Έλεγχος: `xacro gofa12_with_camera.xacro > /tmp/gofa.urdf && check_urdf /tmp/gofa.urdf`

---

## 3. Πακέτο MoveIt για τον GoFa

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

1. Φόρτωσε το `gofa12_with_camera.xacro`.
2. *Self-Collisions* → Generate.
3. *Planning Groups*: ένα group (π.χ. `manipulator`) ως **αλυσίδα** από `base_link`
   ως το `tool0` (ή ως το `gripper_link`, όπως στον UR5e). Kinematic solver:
   `kdl_kinematics_plugin/KDLKinematicsPlugin` (ίδιο με τον UR5e).
   - **Γιατί έτσι:** το `move_arm3` λύνει IK για το `gemini_camera_link`
     (`set_from_ik(PLANNING_GROUP, pose, CAMERA_LINK)`). Η κάμερα πρέπει να είναι
     **σταθερά** δεμένη στο άκρο της αλυσίδας.
4. *Controllers*: `FollowJointTrajectory` με όνομα **ίδιο** με τον controller του
   driver (§6) ή της προσομοίωσης.
5. Όνομα πακέτου π.χ. `gofa12_moveit_config`.

Έλεγχος του ονόματος group (μπαίνει στο YAML):
```bash
grep -o 'group name="[^"]*"' <gofa12_moveit_config>/config/*.srdf
```

---

## 4. Launch: `launch/run_move_arm3_gofa12.launch.py`

Αντέγραψε το `run_move_arm3.launch.py` και άλλαξε:

```python
moveit_config = (
    MoveItConfigsBuilder("gofa12", package_name="gofa12_moveit_config")      # <- 1
    .robot_description(
        file_path=os.path.join(my_pkg_path, "urdf", "gofa12_with_camera.xacro"))  # <- 2 (χωρίς mappings UR)
    .robot_description_semantic(
        file_path=os.path.join(gofa_cfg_path, "config", "gofa12.srdf"))      # <- 3 (όνομα από το Setup Assistant)
    .planning_pipelines(pipelines=["ompl"])
    .robot_description_kinematics(
        file_path=os.path.join(gofa_cfg_path, "config", "kinematics.yaml"))  # <- 4
    .to_moveit_configs()
)
...
"max_velocity_scaling_factor": 0.1,        # <- 5  ΠΡΑΓΜΑΤΙΚΟΣ ΒΡΑΧΙΟΝΑΣ: 0.1 - 0.2
"max_acceleration_scaling_factor": 0.1,
...
{"use_sim_time": False},                   # <- 6  True μόνο σε Gazebo
```

και βάλε ως προεπιλογή του `robot_config` το `config/gofa12.yaml`
(πλήρες path, π.χ. με `get_package_share_directory`).

---

## 5. Συμπλήρωση του `config/gofa12.yaml`

Το αρχείο υπάρχει ήδη ως πρότυπο με σχόλια ανά κλειδί. Στην εκκίνηση το
`move_arm3` τυπώνει `=== Ρυθμίσεις βραχίονα: ABB GoFa 12 ... ===` με ό,τι
φόρτωσε - **έλεγξέ το κάθε φορά**. Άγνωστο κλειδί = σταματά με σφάλμα.

| Κλειδί | Πώς το βρίσκεις |
|---|---|
| `planning_group` | από το SRDF (§3) |
| `camera_link` | `gemini_camera_link` αν κράτησες τα ονόματα (§2.2) - ΟΧΙ το optical |
| `base_frame` | `base_link` |
| `self_links` | `link_1`, …, `link_6` και στο τέλος ο σύνδεσμος που κρατά την κάμερα (`tool0` ή `gripper_link`). Με τη σειρά από τον ώμο προς την κάμερα. |
| `link_radius` | ΜΕΤΡΗΣΕ: μισό πάχος του χοντρότερου συνδέσμου (ο GoFa είναι πιο χοντρός από τον UR5e - το πρότυπο έχει 0.08) |
| `max_reach` | ΜΕΤΡΗΣΕ τη μέγιστη απόσταση **κάμερας** από τη βάση που φτάνει ο IK. Στον UR5e: 0.85 ονομαστική → 0.93 μετρημένη. Τρόπος: στο RViz (MotionPlanning) σύρε τον δείκτη προς τα έξω σε διάφορες κατευθύνσεις/ύψη ως εκεί που αποτυγχάνει ο IK, και διάβασε `ros2 run tf2_ros tf2_echo base_link gemini_camera_link`. Ξεκίνα συντηρητικά. |
| `min_reach` | πιο κοντά από αυτό ο βραχίονας «διπλώνει» πάνω του - ΜΕΤΡΗΣΕ με τον ίδιο τρόπο |
| `workable_dist` | πόσο μακριά από τη βάση δέχεσαι βαλίτσα ως στόχο - εξαρτάται από τη θέση της ταινίας |
| `ik_timeout` | 0.03-0.05 s ανά δοκιμή (γίνονται έως 12 δοκιμές ανά θέση) |
| `small_move_deg` | 30 (όπως UR5e) |
| `settle` | αναμονή μετά την κίνηση ώστε η εικόνα να είναι καθαρή: πραγματική κάμερα ~1.0-1.5 s |
| `camera_hfov`, `camera_width`, `camera_height` | από το `camera_info` της πραγματικής κάμερας (`hfov = 2·atan(width / (2·fx))`) |
| `watch_forward`, `watch_right` | **ΞΑΝΑΜΕΤΡΗΣΕ**: βάλε τον βραχίονα (teach pendant / RViz) σε θέση που βλέπει τον διάδρομο και διάβασε τη θέση κάμερας με `tf2_echo`. Μορφή: `[[θέση κάμερας], [σημείο που κοιτάζει]]` στο `base_link`. |
| `face_views`, `elevate_views`, `handle_standoffs` | αποστάσεις θέασης: εξαρτώνται από κάμερα και μέγεθος tag, **όχι** από τον βραχίονα - μένουν ίδιες αν η κάμερα είναι ίδια |

Εκτέλεση:
```bash
ros2 launch my_ur5e_controller run_move_arm3_gofa12.launch.py \
  robot_config:=$(ros2 pkg prefix my_ur5e_controller)/share/my_ur5e_controller/config/gofa12.yaml
```

---

## 6. Driver του πραγματικού βραχίονα

- Ο GoFa ελέγχεται από controller **OmniCore**. Για ROS 2 υπάρχει το
  [`abb_ros2`](https://github.com/PickNikRobotics/abb_ros2): `abb_hardware_interface`
  (ros2_control μέσω **EGM** - Externally Guided Motion) και `abb_rws_client`
  (**RWS** - Robot Web Services). ΕΠΙΒΕΒΑΙΩΣΕ: (α) ότι ο controller έχει την
  επιλογή EGM, (β) ότι το repo χτίζεται σε Jazzy (ο ενεργός κλάδος είναι `rolling`).
- Το όνομα του `FollowJointTrajectory` controller στο MoveIt (§3) πρέπει να είναι
  **ίδιο** με αυτό που φορτώνει ο driver.
- **Ασφάλεια:** ο έλεγχος σύγκρουσης του `move_arm3` **δεν** είναι σύστημα ασφαλείας.
  Ρύθμισε ζώνες/όρια ταχύτητας στον controller (SafeMove), χαμηλή ταχύτητα στο
  launch (0.1), και έχε πάντα κάποιον στο emergency stop.

---

## 7. Πραγματική κάμερα (στη θέση των γεφυρών του Gazebo)

Ο ανιχνευτής (`aruco_multi_detector.py`) περιμένει:

| Topic | Τύπος |
|---|---|
| `/camera/image` | `sensor_msgs/Image` (έγχρωμη) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` |
| `/camera/points` | `sensor_msgs/PointCloud2` |

Με πραγματικό driver κάνεις remap:
```bash
ros2 run my_ur5e_controller aruco_multi_detector --ros-args \
  -r /camera/image:=<topic_χρώματος> \
  -r /camera/camera_info:=<topic_camera_info_χρώματος> \
  -r /camera/points:=<topic_point_cloud>
```

**Τρεις απαιτήσεις του ανιχνευτή που ΠΡΕΠΕΙ να ελεγχθούν:**

1. **Οργανωμένο point cloud, ευθυγραμμισμένο με την έγχρωμη εικόνα.** Ο ανιχνευτής
   διαβάζει το cloud στο **ίδιο pixel (u, v)** όπου βρήκε το tag στην εικόνα
   (`get_3d_point`, `corner_3d`: `off = v*row_step + u*point_step`). Άρα το cloud
   πρέπει να έχει `width × height` ίσα με την εικόνα και να είναι καταχωρημένο
   (registered / aligned) στην έγχρωμη κάμερα. Στον driver ενεργοποίησε την
   ευθυγράμμιση βάθους→χρώματος.
2. **Άξονας βάθους.** Το φίλτρο βάθους είναι `x < dmin or x > dmax`
   (στα `get_3d_point` και `corner_3d`), δηλαδή υποθέτει cloud με **x μπροστά**
   (όπως το Gazebo, frame `gemini_camera_link`). Οι πραγματικοί drivers συνήθως
   δίνουν το cloud σε **optical** frame (**z μπροστά**). Τότε: είτε άλλαξε τη
   συνθήκη σε `z < dmin or z > dmax` (και στα δύο σημεία), είτε μετάτρεψε το cloud
   σε frame με x μπροστά πριν τον ανιχνευτή.
   Frame του cloud: `ros2 topic echo --once /camera/points --field header.frame_id`
3. **Frames.** Στην αρχή του αρχείου:
   `RAW_POINT_FRAME` = το frame του point cloud (το παραπάνω `header.frame_id`),
   `OPTICAL_FRAME` = το optical frame της έγχρωμης κάμερας (`header.frame_id` του
   `camera_info`), `TARGET_FRAME = 'base_link'`. Ο μετασχηματισμός γίνεται με TF,
   άρα πρέπει να υπάρχει αλυσίδα TF `base_link → … → tool0 → κάμερα`. Αν τρέχει
   και ο driver της κάμερας με δικά του TF, φρόντισε να **συνδέεται** με το
   `gemini_camera_link` του URDF (ένα από τα δύο δένει το άλλο - όχι δύο «ρίζες»).

**Επίσης:**
- **Μεγέθη tags:** `TAG_REGISTRY` → `tag_size` (m, μαύρο τετράγωνο χωρίς το λευκό
  περιθώριο). ΜΕΤΡΗΣΕ τα τυπωμένα tags.
- **Διαστάσεις βαλιτσών:** `BODY_IDS` στο `move_arm3.py` **και** `TAG_REGISTRY`
  στον ανιχνευτή **και** `DIMS_BY_INDEX` στον `collision_publisher` - να συμφωνούν.
- **Βαθμονόμηση:** αν θες ακρίβεια χιλιοστών, κάνε hand-eye calibration και βάλε
  το αποτέλεσμα στο `origin` του `camera_joint`.
- **Παράθυρο OpenCV:** ο ανιχνευτής ανοίγει παράθυρο (`cv2.imshow`). Σε υπολογιστή
  χωρίς οθόνη σχολίασε τις γραμμές `cv2.imshow` / `cv2.waitKey`.

---

## 8. Σειρά δοκιμών (προτεινόμενη)

1. **URDF:** `check_urdf` + προβολή στο RViz. Ο άξονας X της κάμερας κοιτάζει σωστά;
2. **MoveIt μόνο (χωρίς βραχίονα):** `demo.launch.py` του νέου πακέτου - σχεδίαση
   κινήσεων για το `gemini_camera_link`.
3. **Προσομοίωση GoFa στο Gazebo** (αν έφτιαξες το §2.2): ολόκληρη η διαδικασία
   του README με `run_move_arm3_gofa12.launch.py` και `gofa12.yaml`.
4. **Πραγματικός βραχίονας, χωρίς βαλίτσες:** ταχύτητα 0.1. Χωρίς βαλίτσα στο
   κάδρο ο κώδικας πάει **μόνο** στις θέσεις επιτήρησης → έλεγξε ότι
   `watch_forward` / `watch_right` είναι ασφαλείς.
5. **Μία ακίνητη βαλίτσα**, μετά κανονική λειτουργία.

---

## 9. Χρήσιμες εντολές

```bash
ros2 run tf2_tools view_frames                       # δέντρο TF (PDF)
ros2 run tf2_ros tf2_echo base_link gemini_camera_link
ros2 topic echo --once /camera/points --field header.frame_id
ros2 topic echo /luggage/tag_detections              # τι βλέπει ο ανιχνευτής
ros2 topic echo /luggage/handle_pose_final           # το αποτέλεσμα
```
