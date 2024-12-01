from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition,UnlessCondition

def generate_launch_description():
    
    using_python_arg=DeclareLaunchArgument(
        "using_python",
        default_value="True"
    )
    
    wheel_radius_arg=DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.056"
    )
    
    wheel_separation_arg=DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.281"
    )
    
    using_controlbot_simple_controller_arg=DeclareLaunchArgument(
        "using_simple_controller",
        default_value="True"
    )
    
    using_controlbot_simple_controller=LaunchConfiguration("using_simple_controller")
    using_python=LaunchConfiguration("using_python")
    wheel_radius=LaunchConfiguration("wheel_radius")
    wheel_separation=LaunchConfiguration("wheel_separation")
    
    joint_state_broadcaster_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    
    simple_controller=GroupAction(
        condition=IfCondition(using_controlbot_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            ),
            Node(
                package="controlbot_controllers",
                executable="simple_controller.py",
                parameters=[{"wheel_radius":wheel_radius},
                            {"wheel_separation":wheel_separation}],
                condition=IfCondition(using_python)
            )
    
        ]
    )
    
    
    controlbot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["controlbot_diff_drive_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
        condition=UnlessCondition(using_controlbot_simple_controller),
    )
    
    
    
    return LaunchDescription([
        using_python_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        joint_state_broadcaster_spawner,
        simple_controller,
        using_controlbot_simple_controller_arg,
        controlbot_controller_spawner
    ]
    )