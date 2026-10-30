#!/usr/bin/env python3

"""
Adaptive Nav2 Path Replanner for ROS 2 Humble.

Monitors:
    - Robot odometry (/odom)
    - Global path (/plan)
    - NavigateToPose action status

When the robot makes no positional progress for `stuck_timeout` seconds,
the node starts a recovery event:
    1. Increases the global costmap inflation radius.
    2. Optionally clears the global costmap (`clear_global_costmap`).
       This is off by default, because clearing also erases obstacles the
       lidar has seen that are not in the static map.
    3. Waits for Nav2 to publish a new path.
    4. Compares the new path with the path from before the change.
         - Different enough: the event succeeds; the higher inflation stays.
         - Too similar: raises inflation again, up to
           `max_similar_path_retries` times.
         - Still too similar: no alternative route exists at these
           inflation levels. The node restores the inflation from before
           the event and leaves the robot to Nav2's own recovery behaviours.
           This is reported as a warning, not as a failure.

`max_adaptations_per_goal` limits the number of recovery events per goal.
When the goal finishes, the inflation radius is restored to
`initial_inflation`.

Nav2's Behavior Tree remains responsible for replanning, so the active BT
must call ComputePathToPose periodically (e.g. inside a RateController).
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from nav_msgs.msg import Odometry, Path
from action_msgs.msg import GoalStatusArray, GoalStatus

from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.srv import SetParameters


INFLATION_PARAMETER = "inflation_layer.inflation_radius"


# ---------------------------------------------------------
# PATH GEOMETRY FUNCTIONS
# ---------------------------------------------------------

def point_to_segment_distance(px, py, ax, ay, bx, by):

    dx = bx - ax
    dy = by - ay

    denom = dx * dx + dy * dy

    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)

    t = max(
        0.0,
        min(
            1.0,
            ((px - ax) * dx + (py - ay) * dy) / denom
        )
    )

    qx = ax + t * dx
    qy = ay + t * dy

    return math.hypot(px - qx, py - qy)


def path_points(path: Path) -> List[Tuple[float, float]]:

    return [
        (
            pose.pose.position.x,
            pose.pose.position.y
        )
        for pose in path.poses
    ]


def directed_path_deviation(source, target):

    if not source or not target:
        return float("inf")

    if len(target) == 1:

        return sum(
            math.hypot(
                x - target[0][0],
                y - target[0][1]
            )
            for x, y in source
        ) / len(source)

    total_distance = 0.0

    for x, y in source:

        distance = min(
            point_to_segment_distance(
                x,
                y,
                *target[i],
                *target[i + 1]
            )
            for i in range(len(target) - 1)
        )

        total_distance += distance

    return total_distance / len(source)


def symmetric_path_deviation(path_a, path_b):

    return 0.5 * (
        directed_path_deviation(path_a, path_b)
        +
        directed_path_deviation(path_b, path_a)
    )


# ---------------------------------------------------------
# ADAPTIVE PATH REPLANNER NODE
# ---------------------------------------------------------

class AdaptivePathReplanner(Node):

    def __init__(self):

        super().__init__("adaptive_path_replanner")

        # ---------------- PARAMETERS ----------------

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("plan_topic", "/plan")

        self.declare_parameter(
            "status_topic",
            "/navigate_to_pose/_action/status"
        )

        self.declare_parameter(
            "global_costmap_node",
            "/global_costmap/global_costmap"
        )

        # Clearing the global costmap also erases obstacles seen by the
        # lidar that are not in the static map (e.g. spawned walls), so the
        # planner may return the same blocked route. Off by default.
        self.declare_parameter("clear_global_costmap", False)

        self.declare_parameter(
            "clear_global_costmap_service",
            "/global_costmap/clear_entirely_global_costmap"
        )

        # Must match global_costmap inflation_layer.inflation_radius
        # in turtlebot3_navigation2/param/humble/burger.yaml.
        self.declare_parameter("initial_inflation", 0.50)
        self.declare_parameter("inflation_step", 0.10)
        self.declare_parameter("max_inflation", 0.80)

        self.declare_parameter("stuck_timeout", 8.0)

        # The robot counts as progressing only after it moves this far
        # (in the odom frame) from the last progress point. Rotating in
        # place or oscillating does not count as progress.
        self.declare_parameter("min_progress_distance", 0.10)

        self.declare_parameter("min_path_length", 0.25)
        self.declare_parameter("min_path_deviation", 0.15)

        # Recovery events (stuck detections) handled per goal.
        self.declare_parameter("max_adaptations_per_goal", 3)

        # Extra inflation raises within one event when the new path is
        # too similar. After that, the node concludes there is no
        # alternative route.
        self.declare_parameter("max_similar_path_retries", 1)

        self.declare_parameter("recovery_cooldown", 5.0)

        # Plans received sooner than this after an inflation change are
        # ignored for comparison, because they may still come from the
        # old costmap.
        self.declare_parameter("comparison_delay", 2.0)

        # Give up waiting for a new plan after this long.
        self.declare_parameter("comparison_timeout", 10.0)

        # ---------------- READ PARAMETERS ----------------

        def param(name):
            return self.get_parameter(name).value

        self.odom_topic = param("odom_topic")
        self.plan_topic = param("plan_topic")
        self.status_topic = param("status_topic")
        self.costmap_node = param("global_costmap_node")

        self.clear_costmap = bool(param("clear_global_costmap"))
        self.clear_service_name = param("clear_global_costmap_service")

        self.initial_inflation = float(param("initial_inflation"))
        self.inflation_step = float(param("inflation_step"))
        self.max_inflation = float(param("max_inflation"))

        self.stuck_timeout = float(param("stuck_timeout"))
        self.min_progress_distance = float(param("min_progress_distance"))

        self.min_path_length = float(param("min_path_length"))
        self.min_path_deviation = float(param("min_path_deviation"))

        self.max_adaptations = int(param("max_adaptations_per_goal"))
        self.max_similar_retries = int(param("max_similar_path_retries"))

        self.cooldown = float(param("recovery_cooldown"))
        self.comparison_delay = float(param("comparison_delay"))
        self.comparison_timeout = float(param("comparison_timeout"))

        # ---------------- ROS SERVICE CLIENTS ----------------

        self.parameter_service = (
            self.costmap_node.rstrip("/") + "/set_parameters"
        )

        self.parameter_client = self.create_client(
            SetParameters,
            self.parameter_service
        )

        self.clear_client = self.create_client(
            ClearEntireCostmap,
            self.clear_service_name
        )

        # ---------------- SUBSCRIBERS ----------------

        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )

        self.create_subscription(
            Path, self.plan_topic, self.plan_callback, 10
        )

        self.create_subscription(
            GoalStatusArray, self.status_topic, self.status_callback, 10
        )

        # ---------------- INTERNAL STATE ----------------

        self.active_goal = False

        self.linear_speed = 0.0
        self.progress_anchor = None
        self.last_progress_time = self.now_seconds()

        # -inf so the cooldown never blocks the first recovery,
        # even when sim time starts near zero.
        self.last_recovery_time = -math.inf

        self.current_path = []
        self.previous_path = []
        self.path_length = 0.0

        # Recovery events started for the current goal.
        self.adaptations = 0
        self.exhausted_reported = False

        # State of the current recovery event.
        self.event_active = False
        self.event_start_inflation = self.initial_inflation
        self.similar_retries_left = 0
        self.event_raises = 0

        self.pending_path_comparison = False
        self.comparison_not_before = 0.0
        self.comparison_deadline = 0.0

        # Inflation value currently applied on the global costmap
        # (as far as this node knows).
        self.current_inflation = self.initial_inflation

        # True while a SetParameters / clear request is outstanding.
        self.request_in_flight = False

        # ---------------- MONITORING TIMER ----------------

        self.timer = self.create_timer(0.5, self.monitor_progress)

        self.get_logger().info("Adaptive Path Replanner started.")

        self.get_logger().info(
            f"Parameter service: {self.parameter_service}"
        )

        self.get_logger().info(
            f"Initial inflation radius: {self.initial_inflation:.2f} m, "
            f"clear global costmap: {self.clear_costmap}"
        )

    # -----------------------------------------------------
    # TIME
    # -----------------------------------------------------

    def now_seconds(self):

        return self.get_clock().now().nanoseconds / 1e9

    # -----------------------------------------------------
    # ODOMETRY CALLBACK
    # -----------------------------------------------------

    def odom_callback(self, msg: Odometry):

        self.linear_speed = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        )

        position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        if self.progress_anchor is None:
            self.progress_anchor = position
            self.last_progress_time = self.now_seconds()
            return

        moved = math.hypot(
            position[0] - self.progress_anchor[0],
            position[1] - self.progress_anchor[1]
        )

        if moved >= self.min_progress_distance:
            self.progress_anchor = position
            self.last_progress_time = self.now_seconds()

    # -----------------------------------------------------
    # NAVIGATION STATUS CALLBACK
    # -----------------------------------------------------

    def status_callback(self, msg: GoalStatusArray):

        active_states = (
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING
        )

        is_active = any(
            status.status in active_states
            for status in msg.status_list
        )

        if is_active and not self.active_goal:

            self.adaptations = 0
            self.exhausted_reported = False
            self.previous_path = []
            self.end_event()
            self.progress_anchor = None
            self.last_progress_time = self.now_seconds()
            self.last_recovery_time = -math.inf

            self.get_logger().info(
                "Navigation goal became active. "
                "Recovery counter reset."
            )

        if not is_active and self.active_goal:

            self.get_logger().info(
                "Navigation goal is no longer active."
            )

            self.end_event()

        self.active_goal = is_active

        # Make sure the costmap is back at its initial inflation
        # (when a goal ends, or if an earlier reset failed).
        self.restore_initial_inflation_if_needed()

    # -----------------------------------------------------
    # GLOBAL PLAN CALLBACK
    # -----------------------------------------------------

    def plan_callback(self, msg: Path):

        new_points = path_points(msg)

        if not new_points:
            return

        self.path_length = sum(
            math.hypot(
                new_points[i + 1][0] - new_points[i][0],
                new_points[i + 1][1] - new_points[i][1]
            )
            for i in range(len(new_points) - 1)
        )

        self.current_path = new_points

        if not self.pending_path_comparison:
            if not self.event_active:
                self.previous_path = new_points
            return

        # Ignore plans that may still come from the old costmap.
        if self.now_seconds() < self.comparison_not_before:
            return

        self.pending_path_comparison = False

        if not self.previous_path:
            self.get_logger().info(
                "No earlier path to compare with. "
                "Keeping the current inflation."
            )
            self.previous_path = new_points
            self.end_event()
            return

        deviation = symmetric_path_deviation(
            self.previous_path,
            new_points
        )

        self.get_logger().info(
            f"Replanned path deviation: {deviation:.3f} m "
            f"(inflation {self.current_inflation:.2f} m)"
        )

        if deviation >= self.min_path_deviation:

            self.get_logger().info(
                "New path differs sufficiently from previous path. "
                f"Keeping inflation at {self.current_inflation:.2f} m."
            )

            self.previous_path = new_points
            self.end_event()
            return

        if self.similar_retries_left > 0:

            self.similar_retries_left -= 1

            self.get_logger().warn(
                "New path is too similar to the previous path. "
                "Increasing inflation again."
            )

            self.raise_inflation()
            return

        self.no_alternative_route(deviation)

    # -----------------------------------------------------
    # STUCK DETECTION
    # -----------------------------------------------------

    def monitor_progress(self):

        if not self.active_goal:
            return

        now = self.now_seconds()

        if self.event_active:

            # Don't wait forever if Nav2 stops publishing plans.
            if (
                self.pending_path_comparison
                and now > self.comparison_deadline
            ):
                self.get_logger().warn(
                    f"No new plan within {self.comparison_timeout:.1f} s "
                    "after the inflation change. Ending this recovery."
                )
                self.end_event()

            return

        if self.request_in_flight:
            return

        if self.path_length < self.min_path_length:
            return

        time_without_progress = now - self.last_progress_time

        if time_without_progress < self.stuck_timeout:
            return

        if now - self.last_recovery_time < self.cooldown:
            return

        # Restart the stuck timer so the checks below aren't repeated
        # every timer tick.
        self.last_progress_time = now

        if self.adaptations >= self.max_adaptations:
            self.report_adaptations_exhausted()
            return

        self.get_logger().warn(
            f"Robot appears stuck. "
            f"Speed: {self.linear_speed:.3f} m/s, "
            f"no {self.min_progress_distance:.2f} m progress for "
            f"{time_without_progress:.1f} seconds."
        )

        self.start_event()

    # -----------------------------------------------------
    # RECOVERY EVENTS
    # -----------------------------------------------------

    def start_event(self):

        self.adaptations += 1
        self.last_recovery_time = self.now_seconds()

        self.event_active = True
        self.event_start_inflation = self.current_inflation
        self.similar_retries_left = self.max_similar_retries
        self.event_raises = 0
        self.previous_path = list(self.current_path)

        self.get_logger().info(
            f"Recovery {self.adaptations}/{self.max_adaptations} started."
        )

        self.raise_inflation()

    def raise_inflation(self):

        new_value = min(
            self.current_inflation + self.inflation_step,
            self.max_inflation
        )

        if new_value <= self.current_inflation + 1e-6:

            self.get_logger().warn(
                f"Inflation already at maximum "
                f"({self.max_inflation:.2f} m)."
            )

            self.no_alternative_route(None)
            return

        # Compare the next plan against the latest one.
        self.previous_path = list(self.current_path)

        if self.request_inflation(new_value, purpose="raise"):
            self.event_raises += 1
        else:
            self.end_event()

    def no_alternative_route(self, deviation):
        """The path doesn't change with more inflation: undo this event."""

        detail = (
            f"deviation {deviation:.3f} m < {self.min_path_deviation:.2f} m"
            if deviation is not None
            else "no room to raise inflation"
        )

        self.get_logger().warn(
            f"No alternative route found ({detail} after "
            f"{self.event_raises} raise(s)). The current route is the only "
            "one available. Restoring inflation to "
            f"{self.event_start_inflation:.2f} m and leaving the robot to "
            "Nav2's recovery behaviours."
        )

        restore_to = self.event_start_inflation
        self.end_event()

        if abs(self.current_inflation - restore_to) > 1e-6:
            self.request_inflation(restore_to, purpose="restore")

    def end_event(self):

        self.event_active = False
        self.pending_path_comparison = False

    def report_adaptations_exhausted(self):

        if self.exhausted_reported:
            return

        self.exhausted_reported = True

        self.get_logger().warn(
            f"All {self.max_adaptations} recoveries for this goal are used. "
            "Leaving Nav2 to its configured recovery handling."
        )

    def restore_initial_inflation_if_needed(self):

        if self.active_goal or self.request_in_flight:
            return

        if abs(self.current_inflation - self.initial_inflation) < 1e-6:
            return

        self.get_logger().info(
            f"Restoring global inflation radius to "
            f"{self.initial_inflation:.2f} m."
        )

        self.request_inflation(self.initial_inflation, purpose="restore")

    # -----------------------------------------------------
    # UPDATE GLOBAL COSTMAP INFLATION
    # -----------------------------------------------------

    def request_inflation(self, value, purpose):
        """Send SetParameters. purpose is "raise" or "restore"."""

        if self.request_in_flight:

            self.get_logger().warn(
                "An inflation update is already in progress."
            )

            return False

        # Non-blocking check: never wait inside a callback.
        if not self.parameter_client.service_is_ready():

            self.get_logger().error(
                f"Parameter service unavailable: "
                f"{self.parameter_service}"
            )

            return False

        parameter = Parameter(
            INFLATION_PARAMETER,
            Parameter.Type.DOUBLE,
            float(value)
        )

        request = SetParameters.Request()
        request.parameters = [parameter.to_parameter_msg()]

        self.get_logger().info(
            f"Requesting global inflation radius: {value:.2f} m"
        )

        self.request_in_flight = True

        future = self.parameter_client.call_async(request)

        future.add_done_callback(
            lambda result, v=value, p=purpose:
            self.on_inflation_set(result, v, p)
        )

        return True

    # -----------------------------------------------------
    # PARAMETER SERVICE RESPONSE
    # -----------------------------------------------------

    def on_inflation_set(self, future, value, purpose):

        try:
            response = future.result()
        except Exception as exc:
            response = None
            reason = str(exc)
        else:
            if response is None or not response.results:
                reason = "no result from parameter service"
            elif not response.results[0].successful:
                reason = response.results[0].reason
            else:
                reason = None

        if reason is not None:

            self.request_in_flight = False

            self.get_logger().error(
                f"Failed to update inflation radius: {reason}"
            )

            if purpose == "raise":
                self.end_event()

            return

        self.current_inflation = value

        self.get_logger().warn(
            f"Global inflation radius updated to {value:.2f} m."
        )

        if purpose != "raise" or not self.event_active:

            self.request_in_flight = False

            # If the goal ended while a raise was in flight,
            # put the costmap back to its initial inflation.
            self.restore_initial_inflation_if_needed()

            return

        if not self.clear_costmap:

            # The inflation layer re-inflates on its next update, and the
            # obstacles seen by the lidar are kept.
            self.request_in_flight = False
            self.arm_path_comparison()
            return

        if not self.clear_client.service_is_ready():

            self.get_logger().warn(
                "Global costmap clear service unavailable. "
                "Waiting for Nav2 to replan on the updated costmap."
            )

            self.request_in_flight = False
            self.arm_path_comparison()
            return

        clear_future = self.clear_client.call_async(
            ClearEntireCostmap.Request()
        )

        clear_future.add_done_callback(self.on_costmap_cleared)

    # -----------------------------------------------------
    # COSTMAP CLEAR RESPONSE
    # -----------------------------------------------------

    def on_costmap_cleared(self, future):

        self.request_in_flight = False

        try:

            future.result()

            self.get_logger().info(
                "Global costmap cleared. "
                "Waiting for Nav2 to generate a new path."
            )

        except Exception as exc:

            self.get_logger().error(
                f"Global costmap clear failed: {exc}"
            )

        self.arm_path_comparison()
        self.restore_initial_inflation_if_needed()

    def arm_path_comparison(self):

        if not self.active_goal or not self.event_active:
            return

        now = self.now_seconds()

        self.pending_path_comparison = True
        self.comparison_not_before = now + self.comparison_delay
        self.comparison_deadline = (
            now + self.comparison_delay + self.comparison_timeout
        )


# ---------------------------------------------------------
# MAIN
# ---------------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = AdaptivePathReplanner()

    try:

        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":

    main()
