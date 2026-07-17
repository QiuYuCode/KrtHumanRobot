from rclpy.lifecycle import LifecycleNode

from agx_action_group_runner.runner_node import ActionGroupRunnerNode
from agx_action_group_runner.teach_action_group_node import TeachActionGroupNode


def test_action_group_nodes_are_lifecycle_managed():
    assert issubclass(ActionGroupRunnerNode, LifecycleNode)
    assert issubclass(TeachActionGroupNode, LifecycleNode)
