import ament_index_python.packages
import launch 
from launch_ros.actions import Node
def generate_launch_description():
    ld = launch.LaunchDescription()
    motor1_pid_node = Node(
        package="pid_controller",
        executable="motor1_pid",
    )
    motor2_pid_node = Node(
        package="pid_controller",
        executable="motor2_pid",
    )
    set_cmd_node = Node(
        package="command",
        executable="set_cmd"
    )

    
    ugv_fmu_description = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ament_index_python.packages.get_package_share_directory(
                'ugv') + '/launch/ugv_fmu.launch.py'))
                
    ld.add_action(motor1_pid_node)
    ld.add_action(motor2_pid_node)
    ld.add_action(set_cmd_node)
    ld.add_action(ugv_fmu_description)
    return ld
