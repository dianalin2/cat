import struct

START_BYTE = 0xFF
TELEOP_HEADER = 0x00
FEEDBACK_HEADER = 0x01

FEEDBACK_PACKETS = [
    {
        'msg_type': 'EncoderFeedback',
        'header': 0x01,
        'length': 32,
        'parser': lambda data: {
            'name': ['front_left_hip', 'front_left_knee', 'front_right_hip', 'front_right_knee', 'rear_left_hip', 'rear_left_knee', 'rear_right_hip', 'rear_right_knee'],
            'position': list(struct.unpack('<8f', data))
        }
    },
]
