"""Exercise the generated interface type support, not Python-only substitutes."""
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from ridgeback_interfaces.msg import PolarBeams, TargetDetections, TargetMeasurements


@pytest.mark.parametrize('message_type', [PolarBeams, TargetDetections, TargetMeasurements])
def test_generated_interface_round_trip(message_type):
    message = message_type()
    message.header.frame_id = 'camera_0_color_optical_frame'
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456789123
    if message_type is PolarBeams:
        message.scan_stamp.sec = 122
        message.scan_stamp.nanosec = 987654321
        message.scan_frame_id = 'lidar2d_0_laser'
        message.beam_count = 1080
        message.selected = [0, 99, 1079]
        message.merged = [99]
    else:
        message.detected = True
        message.count = 1
        message.bbox_xyxy = [1.0, 2.0, 3.0, 4.0]
        message.labels = ['humanoid robot']
        message.scores = [0.5]
        message.image_width = 640
        message.image_height = 480
        if message_type is TargetMeasurements:
            for field, field_type in message.get_fields_and_field_types().items():
                if field.endswith('_m'):
                    setattr(message, field, [1.25])
                elif field.endswith('_status'):
                    setattr(message, field, [255])
    restored = deserialize_message(serialize_message(message), message_type)
    assert restored == message
