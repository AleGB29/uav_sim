import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
  
def generate_launch_description():
    # Define the nodes to launch
    uav1_node = Node(
            # Set the namespace for the node
            namespace='uav1',
            # Specify the package name containing the node
            package='px4_ros_api',
            # Specify the name of the node
            name='px4_ros_api',
            # Specify the executable to launch
            executable='px4_ros_api',
            # Pass parameters to the node
            parameters=[os.path.join(
                get_package_share_directory('px4_ros_api'),'params','params.yaml')
                ],
            # Remap topic names
            remappings=[
                # Services
                ("arm_disarm", "px4_ros_api/srv/arm_disarm"),  # Remap arm/disarm service
                ("land", "px4_ros_api/srv/land"),  # Remap land service
                ("takeoff", "px4_ros_api/srv/takeoff"),  # Remap takeoff service
                # Subscribers
                ("timesync/px4Msgs/in", "fmu/timesync/out"),  # Remap timesync messages
                ("status/px4Msgs/in", "fmu/vehicle_status/out"),  # Remap vehicle status messages
                ("imu/px4Msgs/in", "fmu/sensor_combined/out"),  # Remap IMU messages
                ("odometry/px4Msgs/in", "fmu/vehicle_odometry/out"),  # Remap odometry messages
                ("trajectory/px4RosExtra/in", "px4_ros_api/trajectory/in"),  # Remap trajectory messages
                # Publishers
                ("imu/stdMsgs/out", "px4_ros_api/imu/out"),  # Remap IMU output messages
                ("odometry/stdMsgs/out", "px4_ros_api/odometry/out"),  # Remap odometry output messages
                ("poseStamped/stdMsgs/out", "px4_ros_api/pose_stamped/out"),  # Remap pose stamped output messages
                ("state/px4RosExtra/out", "px4_ros_api/state/out"),  # Remap UAV state output messages
                ("controlMode/px4Msgs/out", "fmu/offboard_control_mode/in"),  # Remap control mode messages
                ("trajectory/px4Msgs/out", "fmu/trajectory_setpoint/in"),  # Remap trajectory output messages
                ("vehicleCommand/px4Msgs/out", "fmu/vehicle_command/in")  # Remap vehicle command messages
                ],
            # Specify the output location of the node's output
            output='screen',
        )
    uav2_node = Node(
            # Set the namespace for the node
            namespace='uav2',
            # Specify the package name containing the node
            package='px4_ros_api',
            # Specify the name of the node
            name='px4_ros_api',
            # Specify the executable to launch
            executable='px4_ros_api',
            # Pass parameters to the node
            parameters=[os.path.join(
                get_package_share_directory('px4_ros_api'),'params','params.yaml')
                ],
            # Remap topic names
            remappings=[
                # Services
                ("arm_disarm", "px4_ros_api/srv/arm_disarm"),  # Remap arm/disarm service
                ("land", "px4_ros_api/srv/land"),  # Remap land service
                ("takeoff", "px4_ros_api/srv/takeoff"),  # Remap takeoff service
                # Subscribers
                ("timesync/px4Msgs/in", "fmu/timesync/out"),  # Remap timesync messages
                ("status/px4Msgs/in", "fmu/vehicle_status/out"),  # Remap vehicle status messages
                ("imu/px4Msgs/in", "fmu/sensor_combined/out"),  # Remap IMU messages
                ("odometry/px4Msgs/in", "fmu/vehicle_odometry/out"),  # Remap odometry messages
                ("trajectory/px4RosExtra/in", "px4_ros_api/trajectory/in"),  # Remap trajectory messages
                # Publishers
                ("imu/stdMsgs/out", "px4_ros_api/imu/out"),  # Remap IMU output messages
                ("odometry/stdMsgs/out", "px4_ros_api/odometry/out"),  # Remap odometry output messages
                ("poseStamped/stdMsgs/out", "px4_ros_api/pose_stamped/out"),  # Remap pose stamped output messages
                ("state/px4RosExtra/out", "px4_ros_api/state/out"),  # Remap UAV state output messages
                ("controlMode/px4Msgs/out", "fmu/offboard_control_mode/in"),  # Remap control mode messages
                ("trajectory/px4Msgs/out", "fmu/trajectory_setpoint/in"),  # Remap trajectory output messages
                ("vehicleCommand/px4Msgs/out", "fmu/vehicle_command/in")  # Remap vehicle command messages
                ],
            # Specify the output location of the node's output
            output='screen',
        )
    uav3_node = Node(
            # Set the namespace for the node
            namespace='uav3',
            # Specify the package name containing the node
            package='px4_ros_api',
            # Specify the name of the node
            name='px4_ros_api',
            # Specify the executable to launch
            executable='px4_ros_api',
            # Pass parameters to the node
            parameters=[os.path.join(
                get_package_share_directory('px4_ros_api'),'params','params.yaml')
                ],
            # Remap topic names
            remappings=[
                # Services
                ("arm_disarm", "px4_ros_api/srv/arm_disarm"),  # Remap arm/disarm service
                ("land", "px4_ros_api/srv/land"),  # Remap land service
                ("takeoff", "px4_ros_api/srv/takeoff"),  # Remap takeoff service
                # Subscribers
                ("timesync/px4Msgs/in", "fmu/timesync/out"),  # Remap timesync messages
                ("status/px4Msgs/in", "fmu/vehicle_status/out"),  # Remap vehicle status messages
                ("imu/px4Msgs/in", "fmu/sensor_combined/out"),  # Remap IMU messages
                ("odometry/px4Msgs/in", "fmu/vehicle_odometry/out"),  # Remap odometry messages
                ("trajectory/px4RosExtra/in", "px4_ros_api/trajectory/in"),  # Remap trajectory messages
                # Publishers
                ("imu/stdMsgs/out", "px4_ros_api/imu/out"),  # Remap IMU output messages
                ("odometry/stdMsgs/out", "px4_ros_api/odometry/out"),  # Remap odometry output messages
                ("poseStamped/stdMsgs/out", "px4_ros_api/pose_stamped/out"),  # Remap pose stamped output messages
                ("state/px4RosExtra/out", "px4_ros_api/state/out"),  # Remap UAV state output messages
                ("controlMode/px4Msgs/out", "fmu/offboard_control_mode/in"),  # Remap control mode messages
                ("trajectory/px4Msgs/out", "fmu/trajectory_setpoint/in"),  # Remap trajectory output messages
                ("vehicleCommand/px4Msgs/out", "fmu/vehicle_command/in")  # Remap vehicle command messages
                ],
            # Specify the output location of the node's output
            output='screen',
        )
    # Define the launch description
    return LaunchDescription([uav1_node, uav2_node, uav3_node])