from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='csi_publisher',
            namespace='csi_objDetct_ligStere_ffdirewolf_smbus',
            executable='csi_L_publisher',
            name='csi_L_publisher_node'
        ),
        Node(
            package='csi_publisher',
            namespace='csi_objDetct_ligStere_ffdirewolf_smbus',
            executable='csi_R_publisher',
            name='csi_R_publisher_node'
        ),
        Node(
            package='yolo_detect',
            namespace='csi_objDetct_ligStere_ffdirewolf_smbus',
            executable='yolo_detect_puber',
            name='yolo_detect_puber_node'
        ),
        Node(
            package='lightStereo_disp',
            namespace='csi_objDetct_ligStere_ffdirewolf_smbus',
            executable='lightStereo_disp',
            name='lightStereo_disp_node'
        ),
        Node(
            package='tcp_disp',
            namespace='csi_objDetct_ligStere_ffdirewolf_smbus',
            executable='tcp_disp',
            name='tcp_disp_node'
        ),
        Node(
            package='tcp_objectDetection',
            namespace='csi_objDetct_ligStere_ffdirewolf_smbus',
            executable='tcp_objectDetection',
            name='tcp_objectDetection_node'
        )
    ])
