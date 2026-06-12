import importlib.util
from pathlib import Path
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped


SCRIPT = Path(__file__).parents[1] / "scripts" / "goal_slam_navigator_node.py"
SPEC = importlib.util.spec_from_file_location("goal_slam_navigator_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
GoalSlamNavigator = MODULE.GoalSlamNavigator


class _Future:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value


class _Harness:
    def __init__(self):
        self.nav_request_id = 2
        self.nav_goal_pending = True
        self.nav_goal_handle = object()
        self.final_goal = PoseStamped()
        self.final_goal.pose.position.x = 2.0
        self.state = "NAV_TO_GOAL"
        self.active_goal_is_final = True
        self.nav_failure_count = 0
        self.max_nav_failures = 5
        self.published_statuses = []
        self.reached = []

    def _publish_status(self, text):
        self.published_statuses.append(text)

    def _publish_reached_goal(self, goal):
        self.reached.append(goal)

    def _time_after(self, _seconds):
        return None


class _Clock:
    @staticmethod
    def now():
        return 1


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _TickHarness:
    def __init__(self):
        self.final_goal = PoseStamped()
        self.final_goal.pose.position.x = 2.0
        self.nav_goal_pending = False
        self.nav_goal_handle = None
        self.map_msg = object()
        self.next_retry_time = 0
        self.state = "RECEIVE_GOAL"
        self.nav_failure_count = 0
        self.max_nav_failures = 5
        self.temporary_goal_count = 0
        self.max_temporary_goals = 10
        self.minimum_temporary_goal_distance_m = 0.60
        self.retry_period_sec = 2.0
        self.map_frame = "map"
        self.base_frame = "base_footprint"
        self.nav_client = SimpleNamespace(wait_for_server=lambda timeout_sec: True)
        self.temporary_goal_pub = _Publisher()
        self.published_statuses = []
        self.sent_goals = []

    def get_clock(self):
        return _Clock()

    def _lookup_robot_pose(self):
        return PoseStamped()

    def _finish_if_final_goal_is_close(self, **_kwargs):
        return False

    def _pose_is_known_free(self, _pose):
        return False

    def _choose_temporary_goal(self, _robot_pose, _final_goal):
        goal = PoseStamped()
        goal.pose.position.x = 0.4
        return goal

    def _distance(self, a, b):
        return GoalSlamNavigator._distance(a, b)

    def _time_after(self, seconds):
        return seconds

    def _publish_status(self, text):
        self.published_statuses.append(text)

    def _send_nav_goal(self, pose, is_final):
        self.sent_goals.append((pose, is_final))


def test_stale_success_result_cannot_mark_new_goal_reached():
    harness = _Harness()
    stale_success = _Future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))

    GoalSlamNavigator._on_nav_result(harness, stale_success, request_id=1)

    assert harness.final_goal is not None
    assert harness.reached == []
    assert harness.published_statuses == []


def test_stale_failure_result_cannot_increment_new_goal_failures():
    harness = _Harness()
    stale_failure = _Future(SimpleNamespace(status=GoalStatus.STATUS_ABORTED))

    GoalSlamNavigator._on_nav_result(harness, stale_failure, request_id=1)

    assert harness.nav_failure_count == 0
    assert harness.final_goal is not None
    assert harness.published_statuses == []


def test_stale_goal_response_cannot_clear_current_pending_state():
    harness = _Harness()
    stale_response = _Future(None)

    GoalSlamNavigator._on_goal_response(harness, stale_response, request_id=1)

    assert harness.nav_goal_pending is True
    assert harness.nav_goal_handle is not None


def test_too_close_temporary_goal_waits_without_dispatch_or_failure():
    harness = _TickHarness()

    GoalSlamNavigator._tick(harness)

    assert harness.state == "RETRY_PLAN"
    assert harness.next_retry_time == 2.0
    assert harness.nav_failure_count == 0
    assert harness.temporary_goal_count == 0
    assert harness.temporary_goal_pub.messages == []
    assert harness.sent_goals == []
    assert harness.published_statuses == [
        "RETRY_PLAN temporary goal too close; waiting for map growth"
    ]
