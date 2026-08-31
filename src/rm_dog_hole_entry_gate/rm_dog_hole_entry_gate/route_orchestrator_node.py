"""Public NavigateToPose proxy for a map-bound dog-hole route."""

from __future__ import annotations

from copy import deepcopy
import math
from threading import Lock

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rm_dog_hole_entry_gate.route import (
    RoutePose,
    load_route,
    validate_route_map_binding,
)
from rm_path_annotations.core import RegionContractError


class DogHoleRouteOrchestratorNode(Node):
    """Stage selected goals through stop and exit poses using Nav2 actions."""

    def __init__(self) -> None:
        super().__init__("dog_hole_route_orchestrator")
        route_file = self._required_string("route_file")
        expected_map_id = self._required_string("expected_map_id")
        expected_map_revision = self._required_string("expected_map_revision")
        expected_manifest_sha256 = self._required_string(
            "expected_manifest_sha256"
        )
        public_action = self.declare_parameter(
            "public_navigate_to_pose_action", "/navigate_to_pose"
        ).value
        direct_action = self.declare_parameter(
            "direct_navigate_to_pose_action", "/navigate_to_pose_direct"
        ).value
        through_action = self.declare_parameter(
            "navigate_through_poses_action", "/navigate_through_poses"
        ).value
        public_goal_topic = self.declare_parameter(
            "public_goal_pose_topic", "/goal_pose"
        ).value
        self._server_wait_sec = self._finite_positive("server_wait_sec", 10.0)
        for name, value in (
            ("public_navigate_to_pose_action", public_action),
            ("direct_navigate_to_pose_action", direct_action),
            ("navigate_through_poses_action", through_action),
            ("public_goal_pose_topic", public_goal_topic),
        ):
            if not isinstance(value, str) or not value.strip():
                raise RegionContractError(f"{name} must be a non-empty string")
        if public_action == direct_action:
            raise RegionContractError(
                "public and direct NavigateToPose action names must differ"
            )

        self._route = load_route(route_file)
        validate_route_map_binding(
            self._route,
            expected_map_id=expected_map_id,
            expected_map_revision=expected_map_revision,
            expected_manifest_sha256=expected_manifest_sha256,
        )
        self._callback_group = ReentrantCallbackGroup()
        self._to_pose_client = ActionClient(
            self,
            NavigateToPose,
            direct_action,
            callback_group=self._callback_group,
        )
        self._through_client = ActionClient(
            self,
            NavigateThroughPoses,
            through_action,
            callback_group=self._callback_group,
        )
        self._public_client = ActionClient(
            self,
            NavigateToPose,
            public_action,
            callback_group=self._callback_group,
        )
        self._state_publisher = self.create_publisher(
            String, "/dog_hole/route_state", 10
        )
        self._active_publisher = self.create_publisher(
            Bool, "/dog_hole/route_active", 10
        )
        self._lock = Lock()
        self._goal_reserved = False
        self._direct_goal_handle = None
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            public_action,
            execute_callback=self._execute,
            goal_callback=self._accept_goal,
            cancel_callback=self._cancel_goal,
            callback_group=self._callback_group,
        )
        self._goal_pose_subscription = self.create_subscription(
            PoseStamped,
            public_goal_topic,
            self._handle_goal_pose,
            10,
            callback_group=self._callback_group,
        )
        self._publish_state("idle", active=False)
        self.get_logger().warning(
            "DOG-HOLE ROUTE ORCHESTRATOR ACTIVE: "
            f"route={self._route.route_id}@{self._route.revision}, "
            f"public={public_action}, direct={direct_action}. Goals in the "
            "trigger polygon are forced through stop -> exit -> original goal."
        )

    def _required_string(self, name: str) -> str:
        value = self.declare_parameter(name, "").value
        if not isinstance(value, str) or not value.strip():
            raise RegionContractError(f"{name} must be a non-empty string")
        return value.strip()

    def _finite_positive(self, name: str, default: float) -> float:
        value = self.declare_parameter(name, default).value
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RegionContractError(f"{name} must be numeric")
        result = float(value)
        if not math.isfinite(result) or result <= 0.0:
            raise RegionContractError(f"{name} must be finite and positive")
        return result

    def _accept_goal(self, request: NavigateToPose.Goal) -> GoalResponse:
        pose = request.pose
        route_selected = self._route.stages_goal(
            pose.header.frame_id,
            pose.pose.position.x,
            pose.pose.position.y,
        )
        if not self._valid_goal_pose(pose):
            self.get_logger().error(
                "Rejecting navigation goal with invalid frame or non-finite pose"
            )
            return GoalResponse.REJECT
        if route_selected and request.behavior_tree:
            self.get_logger().error(
                "Rejecting staged dog-hole goal with a custom NavigateToPose "
                "behavior tree; NavigateThroughPoses requires its own tree"
            )
            return GoalResponse.REJECT
        with self._lock:
            if self._goal_reserved:
                self.get_logger().warning(
                    "Rejecting concurrent navigation goal; cancel the active goal first"
                )
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _cancel_goal(self, unused_goal_handle) -> CancelResponse:
        del unused_goal_handle
        with self._lock:
            direct = self._direct_goal_handle
        if direct is not None:
            direct.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _handle_goal_pose(self, pose: PoseStamped) -> None:
        """Route RViz's goal topic through the same public action boundary."""
        goal = NavigateToPose.Goal()
        goal.pose = self._fresh_copy(pose)
        future = self._public_client.send_goal_async(goal)
        future.add_done_callback(self._observe_topic_goal_acceptance)

    def _observe_topic_goal_acceptance(self, future) -> None:
        try:
            handle = future.result()
        except Exception as error:
            self.get_logger().error(f"Failed to submit /goal_pose: {error}")
            return
        if not handle.accepted:
            self.get_logger().warning(
                "RViz /goal_pose was rejected because another goal is active "
                "or the pose violates the route contract"
            )

    async def _execute(self, goal_handle) -> NavigateToPose.Result:
        request = goal_handle.request
        route_selected = self._route.stages_goal(
            request.pose.header.frame_id,
            request.pose.pose.position.x,
            request.pose.pose.position.y,
        )
        try:
            if route_selected:
                status = await self._execute_staged_route(goal_handle)
            else:
                self._publish_state("direct_navigation", active=False)
                direct_goal = NavigateToPose.Goal()
                direct_goal.pose = self._fresh_copy(request.pose)
                direct_goal.behavior_tree = request.behavior_tree
                status = await self._run_to_pose(
                    goal_handle, direct_goal, "direct_navigation"
                )
            self._finish_public_goal(goal_handle, status)
        except Exception as error:  # ROS action failures must terminate the goal.
            self.get_logger().error(f"Dog-hole route execution failed: {error}")
            if goal_handle.is_active:
                goal_handle.abort()
        finally:
            with self._lock:
                self._direct_goal_handle = None
                self._goal_reserved = False
            self._publish_state("idle", active=False)
        return NavigateToPose.Result()

    async def _execute_staged_route(self, goal_handle) -> int:
        self.get_logger().warning(
            "Goal is inside dog-hole trigger polygon; enforcing stop -> "
            "transition hold -> exit -> original goal"
        )
        stop_goal = NavigateToPose.Goal()
        stop_goal.pose = self._pose_stamped(self._route.stop_pose)
        self._publish_state("navigating_to_stop", active=True)
        status = await self._run_to_pose(
            goal_handle, stop_goal, "navigating_to_stop"
        )
        if status != GoalStatus.STATUS_SUCCEEDED:
            return status
        if goal_handle.is_cancel_requested:
            return GoalStatus.STATUS_CANCELED

        # Sending the through-route creates a path crossing the committed
        # corridor. The separate final-velocity gate sees that path while the
        # robot is at stop_pose and owns the 5 s transition hold. A future
        # lower-controller request/ack replaces that gate timer, not this
        # action-routing layer.
        through_goal = NavigateThroughPoses.Goal()
        through_goal.poses = [
            self._pose_stamped(self._route.exit_pose),
            self._fresh_copy(goal_handle.request.pose),
        ]
        through_goal.behavior_tree = ""
        self._publish_state("transition_then_traverse", active=True)
        return await self._run_through(
            goal_handle, through_goal, "transition_then_traverse"
        )

    async def _run_to_pose(self, outer, direct_goal, stage: str) -> int:
        if not self._to_pose_client.wait_for_server(
            timeout_sec=self._server_wait_sec
        ):
            self.get_logger().error("Direct NavigateToPose action is unavailable")
            return GoalStatus.STATUS_ABORTED
        future = self._to_pose_client.send_goal_async(
            direct_goal,
            feedback_callback=lambda message: self._relay_feedback(
                outer, message.feedback, stage
            ),
        )
        direct = await future
        return await self._await_direct_result(outer, direct, stage)

    async def _run_through(self, outer, direct_goal, stage: str) -> int:
        if not self._through_client.wait_for_server(
            timeout_sec=self._server_wait_sec
        ):
            self.get_logger().error("NavigateThroughPoses action is unavailable")
            return GoalStatus.STATUS_ABORTED
        future = self._through_client.send_goal_async(
            direct_goal,
            feedback_callback=lambda message: self._relay_feedback(
                outer, message.feedback, stage
            ),
        )
        direct = await future
        return await self._await_direct_result(outer, direct, stage)

    async def _await_direct_result(self, outer, direct, stage: str) -> int:
        if not direct.accepted:
            self.get_logger().error(f"Nav2 rejected route stage {stage}")
            return GoalStatus.STATUS_ABORTED
        with self._lock:
            self._direct_goal_handle = direct
        if outer.is_cancel_requested:
            await direct.cancel_goal_async()
        wrapped_result = await direct.get_result_async()
        with self._lock:
            if self._direct_goal_handle is direct:
                self._direct_goal_handle = None
        return wrapped_result.status

    def _relay_feedback(self, outer, source, unused_stage: str) -> None:
        del unused_stage
        if not outer.is_active:
            return
        feedback = NavigateToPose.Feedback()
        feedback.current_pose = source.current_pose
        feedback.navigation_time = source.navigation_time
        feedback.estimated_time_remaining = source.estimated_time_remaining
        feedback.number_of_recoveries = source.number_of_recoveries
        feedback.distance_remaining = source.distance_remaining
        outer.publish_feedback(feedback)

    def _finish_public_goal(self, goal_handle, status: int) -> None:
        if status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        elif status == GoalStatus.STATUS_CANCELED or goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()

    def _fresh_copy(self, pose: PoseStamped) -> PoseStamped:
        result = deepcopy(pose)
        result.header.stamp = self.get_clock().now().to_msg()
        return result

    def _pose_stamped(self, pose: RoutePose) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = self._route.map_binding.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = pose.x
        message.pose.position.y = pose.y
        message.pose.orientation.z = math.sin(pose.yaw / 2.0)
        message.pose.orientation.w = math.cos(pose.yaw / 2.0)
        return message

    def _valid_goal_pose(self, pose: PoseStamped) -> bool:
        if pose.header.frame_id != self._route.map_binding.frame_id:
            return False
        values = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        return all(math.isfinite(value) for value in values)

    def _publish_state(self, value: str, *, active: bool) -> None:
        state = String()
        state.data = value
        self._state_publisher.publish(state)
        active_message = Bool()
        active_message.data = active
        self._active_publisher.publish(active_message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = DogHoleRouteOrchestratorNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
