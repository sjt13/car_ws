from car_yolo.uav_target_bridge_node import UavTargetBridgeNode


def test_detection_publish_rate_limit_allows_first_and_due_updates():
    assert UavTargetBridgeNode._publish_due(None, 10.0, 5.0)
    assert not UavTargetBridgeNode._publish_due(10.0, 10.1, 5.0)
    assert UavTargetBridgeNode._publish_due(10.0, 10.2, 5.0)


def test_detection_publish_rate_limit_can_be_disabled():
    assert UavTargetBridgeNode._publish_due(10.0, 10.01, 0.0)
