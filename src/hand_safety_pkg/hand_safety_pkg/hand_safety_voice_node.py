#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from unitree_api.msg import Request, Response


class HandSafetyVoiceNode(Node):
    """Speak G1 voice prompts when hand estop state changes."""

    AUDIO_TTS_API_ID = 1001

    def __init__(self):
        super().__init__('hand_safety_voice_node')

        self.estop_topic = self.declare_parameter(
            'estop_topic', '/safe/inspire_hand/estop'
        ).value
        self.voice_request_topic = self.declare_parameter(
            'voice_request_topic', '/api/voice/request'
        ).value
        self.voice_response_topic = self.declare_parameter(
            'voice_response_topic', '/api/voice/response'
        ).value
        self.speaker_id = self.declare_parameter('speaker_id', 0).value

        self.tts_index = 0
        self.last_estop_state = None
        self.pending_requests = {}

        self.estop_sub = self.create_subscription(
            Bool, self.estop_topic, self.estop_callback, 10
        )
        self.voice_req_pub = self.create_publisher(
            Request, self.voice_request_topic, 10
        )
        self.voice_res_sub = self.create_subscription(
            Response, self.voice_response_topic, self.voice_response_callback, 10
        )

        self.get_logger().info(
            f'Voice node ready: {self.estop_topic} -> {self.voice_request_topic}'
        )

    def estop_callback(self, msg: Bool):
        if self.last_estop_state == msg.data:
            return

        self.last_estop_state = msg.data
        if msg.data:
            self.send_tts('灵巧手急停已触发')
        else:
            self.send_tts('灵巧手急停已解除')

    def send_tts(self, text: str):
        req = Request()
        request_id = self.get_clock().now().nanoseconds
        req.header.identity.id = request_id
        req.header.identity.api_id = self.AUDIO_TTS_API_ID
        req.parameter = json.dumps(
            {
                'index': self.tts_index,
                'text': text,
                'speaker_id': int(self.speaker_id),
            },
            ensure_ascii=False,
        )

        self.tts_index += 1
        self.pending_requests[request_id] = text
        self.voice_req_pub.publish(req)
        self.get_logger().info(f'TTS request sent: {text}')

    def voice_response_callback(self, msg: Response):
        request_id = msg.header.identity.id
        text = self.pending_requests.pop(request_id, None)
        if text is None:
            return

        if msg.header.status.code == 0:
            self.get_logger().info(f'TTS request succeeded: {text}')
        else:
            self.get_logger().warn(
                f'TTS request failed: {text}, code={msg.header.status.code}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = HandSafetyVoiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
