# launch_my_cpp_node.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pubsub',        
            executable='talker',            
            name='publisher_cpp',                       
            #output='screen'        
        ),
        Node(
            package='cpp_pubsub',        
            executable='listener',            
            name='subscriber_cpp',                       
            #output='screen'                    
        )
    ])
