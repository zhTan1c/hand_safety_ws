from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    estop_topic = LaunchConfiguration('estop_topic')
    squat_lock_topic = LaunchConfiguration('squat_lock_topic')
    lowstate_topic = LaunchConfiguration('lowstate_topic')
    squat_safe_publish_frames = LaunchConfiguration('squat_safe_publish_frames')
    enable_robot_state_monitor = LaunchConfiguration('enable_robot_state_monitor')
    sport_request_topic = LaunchConfiguration('sport_request_topic')
    sport_response_topic = LaunchConfiguration('sport_response_topic')
    robot_state_poll_hz = LaunchConfiguration('robot_state_poll_hz')
    robot_state_response_timeout_sec = LaunchConfiguration(
        'robot_state_response_timeout_sec')
    auto_clear_squat_lock = LaunchConfiguration('auto_clear_squat_lock')
    enable_record = LaunchConfiguration('enable_record')
    record_log_dir = LaunchConfiguration('record_log_dir')
    # enable_voice = LaunchConfiguration('enable_voice')
    # speaker_id = LaunchConfiguration('speaker_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'estop_topic',
            default_value='/safe/inspire_hand/estop',
            description='Bool topic used to latch or clear hand emergency stop.',
        ),
        DeclareLaunchArgument(
            'squat_lock_topic',
            default_value='/safe/inspire_hand/squat_lock',
            description='Bool topic used to latch or clear squat hand posture lock.',
        ),
        DeclareLaunchArgument(
            'lowstate_topic',
            default_value='/lowstate',
            description='Unitree lowstate topic that carries wireless_remote data.',
        ),
        DeclareLaunchArgument(
            'squat_safe_publish_frames',
            default_value='10',
            description='Number of 50 Hz squat-safe posture command frames.',
        ),
        DeclareLaunchArgument(
            'enable_robot_state_monitor',
            default_value='true',
            description='Start FSM-based robot posture monitor.',
        ),
        DeclareLaunchArgument(
            'sport_request_topic',
            default_value='/api/sport/request',
            description='Unitree sport API request topic.',
        ),
        DeclareLaunchArgument(
            'sport_response_topic',
            default_value='/api/sport/response',
            description='Unitree sport API response topic.',
        ),
        DeclareLaunchArgument(
            'robot_state_poll_hz',
            default_value='5.0',
            description='Polling rate for GetFsmId.',
        ),
        DeclareLaunchArgument(
            'robot_state_response_timeout_sec',
            default_value='1.0',
            description='Timeout for GetFsmId response.',
        ),
        DeclareLaunchArgument(
            'auto_clear_squat_lock',
            default_value='false',
            description='Automatically clear squat lock when FSM returns to a normal motion id.',
        ),
        DeclareLaunchArgument(
            'enable_record',
            default_value='true',
            description='Start hand_safety_record_node.',
        ),
        DeclareLaunchArgument(
            'record_log_dir',
            default_value='/tmp/hand_safety',
            description='Directory for safety trigger logs.',
        ),
        # DeclareLaunchArgument(
        #     'enable_voice',
        #     default_value='true',
        #     description='Start hand_safety_voice_node for G1 TTS prompts.',
        # ),
        # DeclareLaunchArgument(
        #     'speaker_id',
        #     default_value='0',
        #     description='Unitree G1 TTS speaker id.',
        # ),

        Node(
            package='hand_safety_pkg',
            executable='hand_safety_node',
            name='hand_safety_node',
            output='screen',
            parameters=[{
                'squat_lock_topic': squat_lock_topic,
                'squat_safe_publish_frames': ParameterValue(
                    squat_safe_publish_frames, value_type=int),
            }],
        ),
        Node(
            package='hand_safety_pkg',
            executable='hand_gamepad_estop',
            name='hand_gamepad_estop',
            output='screen',
            parameters=[{
                'estop_topic': estop_topic,
                'squat_lock_topic': squat_lock_topic,
                'lowstate_topic': lowstate_topic,
            }],
        ),
        Node(
            package='hand_safety_pkg',
            executable='hand_robot_state_monitor',
            name='hand_robot_state_monitor',
            output='screen',
            condition=IfCondition(enable_robot_state_monitor),
            parameters=[{
                'squat_lock_topic': squat_lock_topic,
                'sport_request_topic': sport_request_topic,
                'sport_response_topic': sport_response_topic,
                'poll_hz': ParameterValue(robot_state_poll_hz, value_type=float),
                'response_timeout_sec': ParameterValue(
                    robot_state_response_timeout_sec, value_type=float),
                'auto_clear_when_safe': ParameterValue(
                    auto_clear_squat_lock, value_type=bool),
            }],
        ),
        Node(
            package='hand_safety_pkg',
            executable='hand_safety_record_node',
            name='hand_safety_record_node',
            output='screen',
            condition=IfCondition(enable_record),
            parameters=[{
                'log_dir': record_log_dir,
            }],
        ),
        # Node(
        #     package='hand_safety_pkg',
        #     executable='hand_safety_voice_node',
        #     name='hand_safety_voice_node',
        #     output='screen',
        #     condition=IfCondition(enable_voice),
        #     parameters=[{
        #         'estop_topic': estop_topic,
        #         'speaker_id': speaker_id,
        #     }],
        # ),
    ])
