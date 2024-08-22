# launch_my_cpp_node.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_pubsub',        
            executable='talker',            
            name='publisher_py',                       
            #output='screen'        
        ),
        Node(
            package='py_pubsub',        
            executable='listener',            
            name='subscriber_py',                       
            #output='screen'                    
        )
    ])
