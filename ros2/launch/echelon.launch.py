from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file to test the echelon3 position
    Parameters :
        - rate : rate of the ROS2 node
        - numberOfSensors : number of sensor connected
        - initPose : initial Position of the arm
    """
    ############## create node ##############


    polhemus = Node(
        package="polhemus_liberty",
        executable="echelon3",
        name = "PolhemusRosNode",
        namespace= "Polhemus",
        parameters=[
            {'rate': 100.0,
            'numberOfSensors': 1,
            }
        ],
    )

    plot = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name = "rqt_plot",
        arguments= (
            "/Polhemus/polhemus/sensor0/pose/position",
        ),
    )

    ############## create launch object ##############
    launch_description = LaunchDescription()

    launch_description.add_action(polhemus)
    launch_description.add_action(plot)


    return launch_description