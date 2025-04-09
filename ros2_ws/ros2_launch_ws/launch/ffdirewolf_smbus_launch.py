from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ffdirewolf_publisher',
            namespace='csi_objDetct_ligStere_ffdirewolf_smbus',
            executable='ffdirewolf_publisher',
            name='ffdirewolf_publisher_node'
        ),
        Node(
            package='smbus_cpp_car_pub',
            namespace='csi_objDetct_ligStere_ffdirewolf_smbus',
            executable='ffdirewolf_msg2smbus_base_msg_pub',
            name='ffdirewolf_msg2smbus_base_msg_pub_node'
        ),
        Node(
            package='smbus_cpp_base_msg_sub',
            namespace='csi_objDetct_ligStere_ffdirewolf_smbus',
            executable='smbus_cpp_base_msg_sub',
            name='smbus_cpp_base_msg_sub_node'
        ),
        Node(
            package='points3d_vector_control',
            namespace='csi_objDetct_ligStere_ffdirewolf_smbus',
            executable='point3D_Vector_control',
            name='point3D_Vector_control_node'
        ),
        Node(
            package='detect_msg_control',
            namespace='csi_objDetct_ligStere_ffdirewolf_smbus',
            executable='detect_msg_control',
            name='detect_msg_control_node'
        )
    ])
