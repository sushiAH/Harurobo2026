from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():

    ld = LaunchDescription()

    zenoh_bridge = ExecuteProcess(
        cmd=[
            "zenoh-bridge-ros2dds",
            "-d",
            "1",
            "--rest-http-port",
            "8000",
        ],
        name="zenoh_bridge",
    )

    publish_twist_node = Node(
        package="harurobo2026",
        executable="publish_twist_node",
    )

    subscribe_twist_node = Node(
        package="harurobo2026",
        executable="subscribe_twist_node",
    )

    control_ring_node = Node(package="harurobo2026",
                             executable="control_ring_node")

    control_yagura_node = Node(package="harurobo2026",
                               executable="control_yagura_node")

    publish_feedback_node = Node(package="harurobo2026",
                                 executable="publish_feedback_node")

    dyna_handler_node = Node(package="ah_ros2_dynamixel",
                             executable="dyna_handler_node",
                             parameters=[{
                                 "port_name": "/dev/ttyUSB-Dynamixel",
                             }])

    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
    )

    ld.add_action(publish_feedback_node)
    ld.add_action(joy_linux_node)
    ld.add_action(dyna_handler_node)

    ld.add_action(publish_twist_node)
    ld.add_action(subscribe_twist_node)
    #ld.add_action(zenoh_bridge)
    ld.add_action(control_ring_node)
    ld.add_action(control_yagura_node)

    return ld
