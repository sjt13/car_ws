from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from car_yolo.openclaw_goal_decision_node import OpenClawGoalDecisionNode


class _MissionHarness:
    def __init__(self):
        self._mission_active = True
        self._mission_current = PoseStamped()
        self._mission_current.pose.position.x = 1.25
        self._mission_current.pose.position.y = -0.40
        self._mission_total = 3
        self._mission_done = 0
        self._failure_handled_for_current = False
        self._continuous_auto_active = True
        self.goal_reached_match_radius_m = 0.60
        self.statuses = []
        self.next_goal_calls = 0
        self.reorder_calls = 0

    def _publish_mission_status(self, text):
        self.statuses.append(text)

    def _publish_next_mission_goal(self):
        self.next_goal_calls += 1

    def _reorder_remaining_mission_goals(self):
        self.reorder_calls += 1

    @staticmethod
    def _pose_distance_2d(a, b):
        return OpenClawGoalDecisionNode._pose_distance_2d(a, b)


def test_navigation_failure_skips_current_goal_without_stopping_continuous_mode():
    harness = _MissionHarness()

    OpenClawGoalDecisionNode.nav_status_callback(
        harness,
        String(data="FAILED_OR_FALLBACK planner failed"),
    )

    assert harness._continuous_auto_active is True
    assert harness._mission_done == 1
    assert harness._mission_current is None
    assert harness.reorder_calls == 1
    assert harness.next_goal_calls == 1
    assert harness.statuses[0].startswith("MISSION_GOAL_FAILED 1/3")


def test_repeated_failure_status_is_handled_only_once():
    harness = _MissionHarness()
    failed = String(data="FAILED_OR_FALLBACK planner failed")

    OpenClawGoalDecisionNode.nav_status_callback(harness, failed)
    OpenClawGoalDecisionNode.nav_status_callback(harness, failed)

    assert harness._mission_done == 1
    assert harness.next_goal_calls == 1


def test_reached_goal_replans_remaining_queue_before_sending_next_goal():
    harness = _MissionHarness()

    OpenClawGoalDecisionNode.reached_goal_callback(
        harness,
        harness._mission_current,
    )

    assert harness._mission_current is None
    assert harness.reorder_calls == 1
    assert harness.next_goal_calls == 1


class _FallbackHarness:
    class_priority_map = {'red_ball': 0, 'red_cube': 1}


def test_fallback_prefers_significantly_closer_target_over_class_priority():
    metadata = [
        {
            'index': 0,
            'label': 'red_ball',
            'score': 0.9,
            'class_priority_rank': 0,
            'distance_m': 2.0,
            'visited': False,
        },
        {
            'index': 1,
            'label': 'red_cube',
            'score': 0.9,
            'class_priority_rank': 1,
            'distance_m': 0.5,
            'visited': False,
        },
    ]

    selected, ordered, _ = OpenClawGoalDecisionNode._fallback_decision(
        _FallbackHarness(),
        metadata,
    )

    assert selected == 1
    assert ordered == [1, 0]


class _ReplanHarness:
    target_frame = 'map'

    def __init__(self):
        self._mission_queue = []
        for x in (4.0, 1.0, 2.0):
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = x
            self._mission_queue.append(goal)
        self.statuses = []

    @staticmethod
    def _lookup_robot_pose(_frame_id):
        return 0.0, 0.0, 0.0

    @staticmethod
    def _distance_to_robot(pose, robot_pose):
        return OpenClawGoalDecisionNode._distance_to_robot(pose, robot_pose)

    def _publish_mission_status(self, text):
        self.statuses.append(text)


def test_replan_orders_remaining_goals_from_current_robot_pose():
    harness = _ReplanHarness()

    OpenClawGoalDecisionNode._reorder_remaining_mission_goals(harness)

    assert [goal.pose.position.x for goal in harness._mission_queue] == [1.0, 2.0, 4.0]
    assert harness.statuses[0].startswith('MISSION_REPLANNED')
