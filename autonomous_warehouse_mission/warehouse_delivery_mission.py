#!/usr/bin/env python3
import math
import time
import csv
import yaml
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry


# ----------------------------
# Small helpers
# ----------------------------
def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """Convert yaw (rad) to quaternion (x, y, z, w)."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


@dataclass
class GoalLog:
    ts: int
    route: str
    node: str
    success: bool
    error_code: int
    duration_s: float
    recoveries: int
    note: str


class WarehouseDeliveryMission(Node):
    """
    Warehouse Mission Manager (real deployment pattern)

    What this node does:
      1) Load a warehouse YAML:
          - named stations (dock, aisle_1, station_a, ...)
          - routes as sequences of stations (aisle-following)
          - mission as sequence of routes
          - behavior knobs (timeouts, tolerances, etc.)
      2) For each station in the mission, send a Nav2 NavigateToPose action goal.
      3) Monitor:
          - recoveries via /behavior_tree_log
          - progress via /odom
      4) Abort mission safely if truly stuck.
      5) Log results to CSV.

    Nav2 does planning + obstacle avoidance + control.
    This node does mission logic + safety monitoring.
    """

    def __init__(self):
        super().__init__("warehouse_delivery_mission")

        # ----------------------------
        # Parameters (can be changed without code changes)
        # ----------------------------
        self.declare_parameter("warehouse_yaml", "")
        self.declare_parameter("csv_log_path", "/tmp/warehouse_mission_log.csv")

        warehouse_yaml = str(self.get_parameter("warehouse_yaml").value).strip()
        self.csv_log_path = str(self.get_parameter("csv_log_path").value)

        self.warehouse_yaml = self._resolve_yaml_path(warehouse_yaml)

        # ----------------------------
        # Nav2 action client
        # ----------------------------
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ----------------------------
        # Monitoring subscriptions
        # ----------------------------
        self.bt_sub = self.create_subscription(String, "/behavior_tree_log", self._on_bt_log, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, 20)

        # ----------------------------
        # Warehouse config data
        # ----------------------------
        self.frame_id = "map"
        self.stations: Dict[str, Tuple[float, float, float]] = {}
        self.routes: Dict[str, List[str]] = {}
        self.mission: List[str] = []

        # ----------------------------
        # Behavior knobs (loaded from YAML)
        # ----------------------------
        self.task_wait_s = 3.0
        self.max_recoveries = 3
        self.no_progress_timeout_s = 30.0
        self.min_progress_m = 0.25

        # THIS is the key for your problem:
        # When within this distance to goal, do not abort due to no-progress
        self.goal_near_threshold_m = 0.40

        # Extra grace time near goal so Nav2 has time to finish
        self.goal_reached_grace_s = 6.0

        # Optional reroute mechanism (kept simple)
        self.allow_reroute = False
        self.alternate_routes: Dict[str, List[str]] = {}

        # Load YAML now
        self._load_warehouse_yaml(self.warehouse_yaml)

        # ----------------------------
        # Runtime mission state
        # ----------------------------
        self.state = "IDLE"  # IDLE, NAVIGATING, WAITING, ABORTED, COMPLETED

        self.current_route_idx = 0
        self.current_route_name: Optional[str] = None
        self.current_route_nodes: List[str] = []
        self.current_node_idx = 0
        self.current_target_node: Optional[str] = None

        # Per-goal state
        self.recoveries_this_goal = 0
        self.goal_start_time = 0.0

        # Feedback distance remaining (may become stale; handled safely)
        self.latest_distance_remaining: Optional[float] = None

        # "Near goal" grace start time
        self.goal_grace_start_time: Optional[float] = None

        # Progress monitoring (odometry)
        self.latest_xy: Optional[Tuple[float, float]] = None
        self.last_progress_xy: Optional[Tuple[float, float]] = None
        self.last_progress_time = time.time()

        # Action goal handle/result future
        self._goal_handle = None
        self._result_future = None

        # ----------------------------
        # CSV logging (do NOT close on abort to avoid race conditions)
        # ----------------------------
        self._csv_file = None
        self._csv_writer = None
        self._open_csv()

        # ----------------------------
        # Start + watchdog timers
        # ----------------------------
        self._started = False
        self.start_timer = self.create_timer(1.0, self._start_once)
        self.watchdog_timer = self.create_timer(1.0, self._watchdog_tick)

    # ============================================================
    # YAML loading + validation
    # ============================================================
    def _resolve_yaml_path(self, path: str) -> str:
        import os
        if path.endswith(".yaml") and os.path.isfile(path):
            return path
        if path and os.path.isdir(path):
            return os.path.join(path, "warehouse.yaml")
        home = os.path.expanduser("~")
        return f"{home}/nav2_py_ws/src/autonomous_warehouse_mission/config/warehouse.yaml"

    def _load_warehouse_yaml(self, path: str):
        self.get_logger().info(f"Loading warehouse config: {path}")
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        self.frame_id = data.get("frame_id", "map")
        stations = data.get("stations", {})
        routes = data.get("routes", {})
        mission = data.get("mission", [])
        behavior = data.get("behavior", {})

        if not stations or not routes or not mission:
            raise RuntimeError("warehouse.yaml must contain: stations, routes, mission")

        self.stations = {k: tuple(v) for k, v in stations.items()}
        self.routes = {k: list(v) for k, v in routes.items()}
        self.mission = list(mission)

        # Behavior knobs (real deployment: configurable)
        self.task_wait_s = float(behavior.get("task_wait_seconds", self.task_wait_s))
        self.max_recoveries = int(behavior.get("max_recoveries_per_goal", self.max_recoveries))
        self.no_progress_timeout_s = float(behavior.get("no_progress_timeout_s", self.no_progress_timeout_s))
        self.min_progress_m = float(behavior.get("min_progress_m", self.min_progress_m))
        self.goal_near_threshold_m = float(behavior.get("goal_near_threshold_m", self.goal_near_threshold_m))
        self.allow_reroute = bool(behavior.get("allow_reroute", self.allow_reroute))
        self.alternate_routes = dict(behavior.get("alternate_routes", {}))

        # Validate mission references
        for rname in self.mission:
            if rname not in self.routes:
                raise RuntimeError(f"Mission references unknown route: {rname}")
            for node in self.routes[rname]:
                if node not in self.stations:
                    raise RuntimeError(f"Route {rname} references unknown station: {node}")

    # ============================================================
    # CSV logging (robust against abort races)
    # ============================================================
    def _open_csv(self):
        self._csv_file = open(self.csv_log_path, "a", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        if self._csv_file.tell() == 0:
            self._csv_writer.writerow([
                "timestamp", "route", "node", "success", "error_code",
                "duration_s", "recoveries", "note"
            ])
            self._csv_file.flush()

    def _log_goal(self, row: GoalLog):
        # IMPORTANT: never write if file closed
        if not self._csv_writer or not self._csv_file or self._csv_file.closed:
            return
        self._csv_writer.writerow([
            row.ts, row.route, row.node, int(row.success), row.error_code,
            f"{row.duration_s:.2f}", row.recoveries, row.note
        ])
        self._csv_file.flush()

    # ============================================================
    # Mission start
    # ============================================================
    def _start_once(self):
        if self._started:
            return
        self._started = True

        self.get_logger().info("Waiting for Nav2 action server...")
        self.nav_client.wait_for_server()

        self.get_logger().info("Starting warehouse mission")
        self.state = "NAVIGATING"
        self.current_route_idx = 0
        self._start_next_route()

    # ============================================================
    # Mission progression: routes -> waypoints
    # ============================================================
    def _start_next_route(self):
        if self.state in ("ABORTED", "COMPLETED"):
            return

        if self.current_route_idx >= len(self.mission):
            self._finish_mission()
            return

        self.current_route_name = self.mission[self.current_route_idx]
        self.current_route_nodes = self.routes[self.current_route_name]
        self.current_node_idx = 0

        self.get_logger().info(f"Route start: {self.current_route_name} -> {self.current_route_nodes}")
        self._send_next_waypoint()

    def _send_next_waypoint(self):
        if self.state in ("ABORTED", "COMPLETED"):
            return

        # Finished this route -> next route
        if self.current_node_idx >= len(self.current_route_nodes):
            self.current_route_idx += 1
            self._start_next_route()
            return

        node = self.current_route_nodes[self.current_node_idx]
        x, y, yaw = self.stations[node]

        self.current_target_node = node
        self.recoveries_this_goal = 0
        self.goal_start_time = time.time()

        # Reset feedback + grace + progress baseline for THIS goal
        self.latest_distance_remaining = None
        self.goal_grace_start_time = None

        self.last_progress_time = time.time()
        self.last_progress_xy = self.latest_xy

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Goal: route={self.current_route_name} waypoint={node} "
            f"x={x:.2f} y={y:.2f} yaw={yaw:.2f}"
        )

        fut = self.nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_goal_response)

    # ============================================================
    # Nav2 action callbacks
    # ============================================================
    def _on_goal_response(self, future):
        if self.state in ("ABORTED", "COMPLETED"):
            return

        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("Goal rejected by Nav2")
            self._abort("goal_rejected")
            return

        self._goal_handle = gh
        self._result_future = gh.get_result_async()
        self._result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        # If already aborted, ignore to avoid races
        if self.state == "ABORTED":
            return

        res = future.result().result
        duration = time.time() - self.goal_start_time
        ok = (res.error_code == 0)

        self.get_logger().info(
            f"Result: route={self.current_route_name} waypoint={self.current_target_node} "
            f"ok={ok} code={res.error_code} time={duration:.1f}s recov={self.recoveries_this_goal}"
        )

        self._log_goal(GoalLog(
            ts=int(time.time()),
            route=str(self.current_route_name),
            node=str(self.current_target_node),
            success=bool(ok),
            error_code=int(res.error_code),
            duration_s=float(duration),
            recoveries=int(self.recoveries_this_goal),
            note="ok" if ok else "nav2_failed"
        ))

        if not ok:
            # Optional reroute
            if self.allow_reroute and self.current_route_name in self.alternate_routes:
                alt = self.alternate_routes[self.current_route_name]
                self.get_logger().warn(f"Reroute: inserting {alt} before retry")
                self.mission[self.current_route_idx:self.current_route_idx] = list(alt)
                self._start_next_route()
                return

            self._abort("nav2_failure")
            return

        # Simulate station tasks (loading/unloading)
        if self.current_target_node in ("loading", "station_a", "station_b", "dock"):
            self.state = "WAITING"
            self.get_logger().info(f"Task at {self.current_target_node}: wait {self.task_wait_s:.1f}s")
            time.sleep(self.task_wait_s)
            self.state = "NAVIGATING"

        self.current_node_idx += 1
        self._send_next_waypoint()

    def _on_feedback(self, feedback_msg):
        dist = getattr(feedback_msg.feedback, "distance_remaining", None)
        if dist is None:
            return

        self.latest_distance_remaining = float(dist)

        # IMPORTANT FIX:
        # When we get "near goal", start a grace timer.
        # Even if odom stops changing, we give Nav2 time to return SUCCESS.
        if self.latest_distance_remaining < self.goal_near_threshold_m:
            if self.goal_grace_start_time is None:
                self.goal_grace_start_time = time.time()

        # Print distance (optional, can be noisy)
        self.get_logger().info(f"Distance remaining to {self.current_target_node}: {dist:.2f} m")

    # ============================================================
    # Recovery monitoring (BT logs)
    # ============================================================
    def _on_bt_log(self, msg: String):
        if self.state != "NAVIGATING":
            return

        text = msg.data
        keywords = ("Spin", "Backup", "ClearCostmap", "Recovery", "NavigateRecovery")
        if any(k in text for k in keywords):
            self.recoveries_this_goal += 1
            self.get_logger().warn(
                f"Recovery ({self.recoveries_this_goal}/{self.max_recoveries}) at {self.current_target_node}"
            )

            if self.recoveries_this_goal > self.max_recoveries:
                self._abort("too_many_recoveries")

    # ============================================================
    # Progress monitoring (odometry)
    # ============================================================
    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.latest_xy = (x, y)

        if self.last_progress_xy is None:
            self.last_progress_xy = (x, y)
            self.last_progress_time = time.time()
            return

        moved = math.hypot(x - self.last_progress_xy[0], y - self.last_progress_xy[1])
        if moved >= self.min_progress_m:
            self.last_progress_xy = (x, y)
            self.last_progress_time = time.time()

    # ============================================================
    # Watchdog (the part that was aborting you)
    # ============================================================
    def _watchdog_tick(self):
        """
        Abort ONLY if:
          - mission is navigating
          - Nav2 goal is still pending
          - and robot has made NO progress for too long
          - and we are NOT near the goal (goal tolerance case)

        The "near goal grace" prevents false aborts when Nav2 is finishing.
        """
        if self.state != "NAVIGATING":
            return
        if self._result_future is None or self._result_future.done():
            return

        # ----- Near-goal protection
        # If we are close to goal, do NOT abort immediately. Give Nav2 grace time.
        if self.latest_distance_remaining is not None:
            if self.latest_distance_remaining < self.goal_near_threshold_m:
                # If grace timer started and still within grace duration, do nothing
                if self.goal_grace_start_time is not None:
                    if (time.time() - self.goal_grace_start_time) < self.goal_reached_grace_s:
                        return
                # Even if grace timer didn't start (rare), being near goal should be safe
                return

        # ----- Standard no-progress abort
        elapsed = time.time() - self.last_progress_time
        if elapsed > self.no_progress_timeout_s:
            self.get_logger().error(
                f"No progress for {elapsed:.1f}s (threshold {self.no_progress_timeout_s:.1f}s). Aborting mission."
            )
            try:
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._abort("no_progress_timeout")

    # ============================================================
    # End states
    # ============================================================
    def _abort(self, reason: str):
        if self.state == "ABORTED":
            return
        self.state = "ABORTED"
        self.get_logger().error(f"MISSION ABORTED: {reason}")

        # IMPORTANT FIX:
        # Do not close CSV here, because action result may still arrive.
        if self._csv_file and not self._csv_file.closed:
            self._csv_file.flush()

    def _finish_mission(self):
        if self.state == "COMPLETED":
            return
        self.state = "COMPLETED"
        self.get_logger().info("MISSION COMPLETED SUCCESSFULLY")

        if self._csv_file and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()


def main():
    rclpy.init()
    node = WarehouseDeliveryMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
