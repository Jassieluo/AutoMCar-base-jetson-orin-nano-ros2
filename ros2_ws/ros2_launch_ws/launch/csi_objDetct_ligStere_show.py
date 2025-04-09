from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='csi_publisher',
            namespace='csi_objDetct_ligStere_show',
            executable='csi_L_publisher',
            name='csi_L_publisher_node'
        ),
        Node(
            package='csi_publisher',
            namespace='csi_objDetct_ligStere_show',
            executable='csi_R_publisher',
            name='csi_R_publisher_node'
        ),
        Node(
            package='yolo_detect',
            namespace='csi_objDetct_ligStere_show',
            executable='yolo_detect_puber',
            name='yolo_detect_puber_node'
        ),
        Node(
            package='yolo_detect',
            namespace='csi_objDetct_ligStere_show',
            executable='detect_msg_draw_box',
            name='detect_msg_draw_box_node'
        ),
        Node(
            package='lightStereo_disp',
            namespace='csi_objDetct_ligStere_show',
            executable='lightStereo_disp',
            name='lightStereo_disp_node'
        ),
        Node(
            package='lightStereo_disp',
            namespace='csi_objDetct_ligStere_show',
            executable='disp_show',
            name='disp_show_node'
        )
    ])
