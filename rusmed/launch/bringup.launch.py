from launch import LaunchDescription
from launch_ros.actions import  PushRosNamespace, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    imu_node = Node(
        package='rusmed_sensors',
        executable='imu',
        name='imu_node',
        output='screen',
        namespace="bmi160"
        # parameters=['params/imu.yaml']  # Si tienes un YAML de params, opcional
    )

    imu_broadcaster = Node(
        package='rusmed_sensors',
        executable='imu_tf_broadcaster',
        name='imu_tf_broadcaster',
        output='screen',
        namespace="bmi160"
        # parameters=['params/imu.yaml']  # Si tienes un YAML de params, opcional
    )


    axial_joint_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rusmed_motors'),
            '/launch/rusmed_control.launch.py'
        ]),
    )

    axial_joint_group  = GroupAction([
        PushRosNamespace('axial_joint'),
        axial_joint_launch
    ])



    return LaunchDescription([
        imu_node,
        imu_broadcaster ,
        axial_joint_group 
    ])
