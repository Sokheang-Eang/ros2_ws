# launch_my_cpp_node.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pubsub',        
            executable='talker',            
            name='publisher',                       
            #output='screen'        
        ),
        Node(
            package='cpp_pubsub',        
            executable='listener',            
            name='subscriber',                       
            #output='screen'                    
        ),
        Node(
            package='py_pubsub',        
            executable='talker',            
            name='publisher',                       
            #output='screen'        
        ),
        Node(
            package='py_pubsub',        
            executable='listener',            
            name='subscriber',                       
            #output='screen'                    
        )
    ])
