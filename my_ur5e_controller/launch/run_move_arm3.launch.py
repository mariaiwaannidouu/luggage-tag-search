import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Προαιρετικό YAML με τις ρυθμίσεις του βραχίονα (δες config/ur5e.yaml,
    # config/gofa12.yaml). Κενό = οι προεπιλογές UR5e μέσα στο move_arm3.py.
    #   ros2 launch my_ur5e_controller run_move_arm3.launch.py robot_config:=/path/x.yaml
    robot_config_arg = DeclareLaunchArgument(
        "robot_config", default_value="",
        description="πλήρες path σε YAML ρυθμίσεων βραχίονα (κενό = UR5e)")

    # 1. Ο φάκελος με τον κώδικα και το URDF
    my_pkg_path = get_package_share_directory("my_ur5e_controller")
    # 2. Ο νέος φάκελος με τις ρυθμίσεις και το ΣΩΣΤΟ SRDF
    my_config_path = get_package_share_directory("my_ur5e_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="my_ur5e_moveit_config")
        .robot_description(
            file_path=os.path.join(my_pkg_path, "urdf", "ur5e_with_camera.xacro"),
            mappings={"name": "ur", "ur_type": "ur5e"}
        )
        .robot_description_semantic(
            # ΤΩΡΑ ΔΙΑΒΑΖΕΙ ΤΟ ΔΙΚΟ ΜΑΣ SRDF ΜΕ ΤΙΣ ΕΞΑΙΡΕΣΕΙΣ!
            file_path=os.path.join(my_config_path, "srdf", "ur.srdf.xacro"),
            mappings={"name": "ur", "ur_type": "ur5e"}
        )
        .planning_pipelines(pipelines=["ompl"])
        .robot_description_kinematics(
            file_path=os.path.join(my_config_path, "config", "kinematics.yaml")
        )
        .to_moveit_configs()
    )

    # Αυτές είναι οι ρυθμίσεις που απαιτεί αυστηρά το MoveItPy για να τρέξει
    moveit_py_params = {
        "planning_pipelines": {
            "pipeline_names": ["ompl"],
        },
        "plan_request_params": {
            "planning_attempts": 2,
            "planning_time": 2.5,
            "planning_pipeline": "ompl",
            "max_velocity_scaling_factor": 1.0,
            "max_acceleration_scaling_factor": 1.0,
        },
    }

    move_arm_node = Node(
        package="my_ur5e_controller",
        executable="move_arm3",
        name="ur5e_python_controller",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_py_params,
            {"use_sim_time": True},
        ],
        # Το path φτάνει στον κόμβο ως μεταβλητή περιβάλλοντος ROBOT_CONFIG,
        # ώστε οι παράμετροι του MoveIt να μένουν ακριβώς ίδιες.
        additional_env={"ROBOT_CONFIG": LaunchConfiguration("robot_config")},
    )

    return LaunchDescription([robot_config_arg, move_arm_node])