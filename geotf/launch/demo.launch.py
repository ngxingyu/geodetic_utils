
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='enubodybroadcaster',
            arguments=[
                "--x", "1.25",
                "--y", "1.25",
                "--z", "0",
                "--qx", "0",
                "--qy", "0",
                "--qz", "0",
                "--qw", "1",
                "--frame-id", "enu",
                "--child-frame-id", "body"
            ]
        ),
        Node(
            package='geotf',
            executable='demo_node',
            name='demo_node',
            output="screen",
            parameters=[
                {
                    "geotf": """Frames:
    ENU_LEE:
        Type: ENUOrigin
        LonOrigin:  8.54582
        LatOrigin:  47.37842
        AltOrigin:  489.619
    GPS:
        Type: GCSCode
        Code: WGS84
    UTM:
        Type: UTM
        Zone: 32
        Hemisphere: N
    CH1903+:
        Type: EPSGCode
        Code: 2056
TF_Mapping:
    GEO_TF: ENU_LEE
    TF: enu"""
                }]
        )])