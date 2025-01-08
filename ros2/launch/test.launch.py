from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ############## create node ##############


    polhemus = Node(
        package="polhemus_liberty",
        executable="polhemus_ros",
        name = "PolhemusRosNode",
        namespace= "Polhemus",
        parameters=[
            {'rate': 100.0,
            'numberOfSensors': 2,
            }        
        ],
    )

    plot = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name = "rqt_plot",
        arguments= (
            # "/Polhemus/polhemus/sensor0/pose/position/x",
            # "/Polhemus/polhemus/sensor0/pose/position/y",
            # "/Polhemus/polhemus/sensor0/pose/position/z",
            "/Polhemus/polhemus/sensor1/pose/position/x",
            "/Polhemus/polhemus/sensor1/pose/position/y",
            "/Polhemus/polhemus/sensor1/pose/position/z",
        ),
    )

    ############## create launch object ##############
    launch_description = LaunchDescription()

    launch_description.add_action(polhemus)
    launch_description.add_action(plot)


    return launch_description