from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    estop_topic = LaunchConfiguration('estop_topic')
    lowstate_topic = LaunchConfiguration('lowstate_topic')
    squat_safe_hold_seconds = LaunchConfiguration('squat_safe_hold_seconds')
    squat_safe_publish_frames = LaunchConfiguration('squat_safe_publish_frames')
    left_raw_cmd_topic = LaunchConfiguration('left_raw_cmd_topic')
    right_raw_cmd_topic = LaunchConfiguration('right_raw_cmd_topic')
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
            'lowstate_topic',
            default_value='/lowstate',
            description='Unitree lowstate topic that carries wireless_remote data.',
        ),
        DeclareLaunchArgument(
            'squat_safe_hold_seconds',
            default_value='2.0',
            description='Hold time for L2+A to publish squat-safe hand posture.',
        ),
        DeclareLaunchArgument(
            'squat_safe_publish_frames',
            default_value='10',
            description='Number of 50 Hz squat-safe posture command frames.',
        ),
        DeclareLaunchArgument(
            'left_raw_cmd_topic',
            default_value='/safe/inspire_hand/raw/cmd/l',
            description='Left raw command topic for squat-safe hand posture.',
        ),
        DeclareLaunchArgument(
            'right_raw_cmd_topic',
            default_value='/safe/inspire_hand/raw/cmd/r',
            description='Right raw command topic for squat-safe hand posture.',
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
        ),
        Node(
            package='hand_safety_pkg',
            executable='hand_gamepad_estop',
            name='hand_gamepad_estop',
            output='screen',
            parameters=[{
                'estop_topic': estop_topic,
                'lowstate_topic': lowstate_topic,
                'squat_safe_hold_seconds': ParameterValue(
                    squat_safe_hold_seconds, value_type=float),
                'squat_safe_publish_frames': ParameterValue(
                    squat_safe_publish_frames, value_type=int),
                'left_raw_cmd_topic': left_raw_cmd_topic,
                'right_raw_cmd_topic': right_raw_cmd_topic,
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
