from rclpy.lifecycle import LifecycleNode

from krt_task.routine_runner import RoutineRunnerNode


def test_routine_runner_is_lifecycle_managed():
    assert issubclass(RoutineRunnerNode, LifecycleNode)
