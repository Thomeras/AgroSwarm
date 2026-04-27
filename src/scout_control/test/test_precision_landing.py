# flake8: noqa
"""Tests for PrecisionLanding ROS2 node."""

import json
import pytest
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from scout_control.vision.precision_landing import PrecisionLanding

class CapturingPublisher:
    def __init__(self):
        self.messages = []
    def publish(self, msg):
        self.messages.append(msg.data)

@pytest.fixture
def rclpy_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    # We don't shutdown to avoid interfering with other tests if they run in the same process
    # rclpy.shutdown()

@pytest.mark.unit
def test_status_callback_parsing(rclpy_context):
    node = PrecisionLanding()
    # Test phase parsing
    node._on_status(String(data=json.dumps({"phase": "MAPPING", "position_ned": [0, 0, -10]})))
    assert node._runtime_phase == "MAPPING"
    assert node._altitude_m == 10.0

    # Test altitude_m direct field
    node._on_status(String(data=json.dumps({"phase": "RETURN_HOME", "altitude_m": 4.5})))
    assert node._runtime_phase == "RETURN_HOME"
    assert node._altitude_m == 4.5

    # Test drone_ned field
    node._on_status(String(data=json.dumps({"drone_ned": [1, 2, -3.2]})))
    assert node._altitude_m == 3.2
    node.destroy_node()

@pytest.mark.unit
def test_active_logic(rclpy_context):
    node = PrecisionLanding()
    node.set_parameters([
        rclpy.parameter.Parameter("active_phase", rclpy.Parameter.Type.STRING, "RETURN_HOME"),
        rclpy.parameter.Parameter("max_active_altitude_m", rclpy.Parameter.Type.DOUBLE, 5.0)
    ])
    
    # Wrong phase
    node._runtime_phase = "MAPPING"
    node._altitude_m = 3.0
    assert not node._active()
    
    # Altitude too high
    node._runtime_phase = "RETURN_HOME"
    node._altitude_m = 6.0
    assert not node._active()
    
    # OK
    node._runtime_phase = "RETURN_HOME"
    node._altitude_m = 4.0
    assert node._active()
    
    # Wildcard phase
    node.set_parameters([rclpy.parameter.Parameter("active_phase", rclpy.Parameter.Type.STRING, "*")])
    node._runtime_phase = "ANYTHING"
    assert node._active()
    node.destroy_node()

@pytest.mark.unit
def test_offset_not_published_when_inactive(rclpy_context, monkeypatch):
    node = PrecisionLanding()
    node._offset_pub = CapturingPublisher()
    
    # Set inactive
    node._runtime_phase = "MAPPING"
    node._altitude_m = 10.0
    
    # Mock image callback
    node._on_image(Image())
    assert len(node._offset_pub.messages) == 0
    node.destroy_node()

@pytest.mark.unit
def test_offset_published_when_active(rclpy_context, monkeypatch):
    import cv2
    import numpy as np
    from scout_control.vision import precision_landing
    
    if not hasattr(cv2, "aruco"):
        pytest.skip("cv2.aruco is unavailable")

    node = PrecisionLanding()
    node._offset_pub = CapturingPublisher()
    
    # Set active
    node._runtime_phase = "RETURN_HOME"
    node._altitude_m = 3.0
    
    # Set intrinsics
    node._intrinsics = precision_landing.CameraIntrinsics(fx=500, fy=500, cx=160, cy=160)
    
    # Generate synthetic image with marker
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, 7, 100)
    else:
        marker = cv2.aruco.drawMarker(dictionary, 7, 100)
    
    frame = np.full((320, 320, 3), 255, dtype=np.uint8)
    frame[110:210, 110:210] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    
    img_msg = Image()
    img_msg.height = 320
    img_msg.width = 320
    img_msg.encoding = "bgr8"
    img_msg.data = frame.tobytes()
    img_msg.step = 320 * 3
    
    # We need to monkeypatch cv_bridge because it might not be installed or might fail in test env
    # But PrecisionLanding uses it. Let's try to use real bridge first.
    
    node._on_image(img_msg)
    
    assert len(node._offset_pub.messages) == 1
    data = json.loads(node._offset_pub.messages[0])
    assert data["marker_id"] == 7
    assert "dx_m" in data
    assert "dy_m" in data
    node.destroy_node()

@pytest.mark.unit
def test_no_px4_output(rclpy_context):
    node = PrecisionLanding()
    # Check that it doesn't publish to PX4 topics
    # We can inspect the node's publishers
    for pub in node.publishers:
        topic = pub.topic_name
        assert "fmu/in" not in topic
        assert "offboard" not in topic
        assert "setpoint" not in topic
    node.destroy_node()
