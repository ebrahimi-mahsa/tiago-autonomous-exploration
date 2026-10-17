#!/usr/bin/env python3

import math
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav2_msgs.srv import SaveMap

from tf2_ros import Buffer, TransformListener, TransformException


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.latest_map = None

        self.min_frontier_cells = 5
        self.max_candidates_to_check = 5

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.path_client = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose'
        )

        # Map saving after autonomous exploration completes.
        self.declare_parameter(
            'map_url',
            '/workspace/maps/task1_autonomous_map'
        )
        self.map_url = self.get_parameter('map_url').value

        self.save_map_client = self.create_client(
            SaveMap,
            '/map_saver/save_map'
        )

        self.map_save_in_progress = False
        self.map_saved = False

        self.action_server_reported = False
        self.path_server_reported = False

        self.path_check_active = False
        self.nav_goal_active = False

        # Continuous exploration state.
        self.goal_attempts = 0
        self.exploration_complete = False

        # Exploration is considered complete only after
        # several consecutive rounds with no reachable target.
        self.no_reachable_rounds = 0
        self.max_no_reachable_rounds = 3

        # Remember explored / failed frontier regions.
        self.visited_frontiers = []
        self.failed_frontiers = []

        self.visited_exclusion_radius = 0.80
        self.failed_exclusion_radius = 1.00

        self.active_nav_candidate = None

        # Throttle navigation feedback logging.
        self.last_feedback_log_ns = 0

        self.pending_candidates = []
        self.reachable_candidates = []

        self.status_timer = self.create_timer(
            2.0,
            self.status_check
        )

        self.frontier_timer = self.create_timer(
            6.0,
            self.analyze_frontiers
        )

        self.get_logger().info(
            'Frontier explorer started.'
        )

    def map_callback(self, msg):
        first_map = self.latest_map is None
        self.latest_map = msg

        if first_map:
            self.get_logger().info(
                f'Map received: '
                f'{msg.info.width} x {msg.info.height}, '
                f'resolution={msg.info.resolution:.3f} m/cell'
            )

    def status_check(self):

        if not self.action_server_reported:
            if self.nav_client.wait_for_server(
                timeout_sec=0.0
            ):
                self.get_logger().info(
                    'NavigateToPose action server is available.'
                )
                self.action_server_reported = True

        if not self.path_server_reported:
            if self.path_client.wait_for_server(
                timeout_sec=0.0
            ):
                self.get_logger().info(
                    'ComputePathToPose action server is available.'
                )
                self.path_server_reported = True

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            self.get_logger().info(
                f'Robot pose in map frame: '
                f'x={x:.2f}, y={y:.2f}'
            )

        except TransformException as ex:
            self.get_logger().warn(
                f'Waiting for map -> base_footprint TF: {ex}'
            )

    def analyze_frontiers(self):

        if self.latest_map is None:
            return

        if (
            self.path_check_active
            or self.nav_goal_active
            or self.exploration_complete
        ):
            return

        if not self.path_client.wait_for_server(
            timeout_sec=0.0
        ):
            self.get_logger().warn(
                'ComputePathToPose server not available yet.'
            )
            return

        msg = self.latest_map

        width = msg.info.width
        height = msg.info.height
        data = msg.data

        frontier_cells = set()

        for row in range(1, height - 1):
            for col in range(1, width - 1):

                idx = row * width + col

                if data[idx] != 0:
                    continue

                touches_unknown = False

                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):

                        if dr == 0 and dc == 0:
                            continue

                        nidx = (
                            (row + dr) * width
                            + (col + dc)
                        )

                        if data[nidx] == -1:
                            touches_unknown = True
                            break

                    if touches_unknown:
                        break

                if touches_unknown:
                    frontier_cells.add(
                        (row, col)
                    )

        clusters = self.cluster_frontiers(
            frontier_cells
        )

        useful_clusters = [
            cluster
            for cluster in clusters
            if len(cluster) >= self.min_frontier_cells
        ]

        useful_clusters.sort(
            key=len,
            reverse=True
        )

        self.get_logger().info(
            f'Frontier analysis: '
            f'{len(frontier_cells)} frontier cells, '
            f'{len(useful_clusters)} useful clusters'
        )

        candidates = []

        for cluster in useful_clusters:

            if len(candidates) >= self.max_candidates_to_check:
                break

            candidate = self.build_standoff_candidate(
                cluster,
                msg
            )

            if candidate is None:
                self.get_logger().warn(
                    f'Skipping cluster of {len(cluster)} cells: '
                    'no safe stand-off point found.'
                )
                continue

            if self.frontier_was_attempted(candidate):

                self.get_logger().info(
                    'Skipping previously attempted frontier '
                    f'({candidate["frontier_x"]:.2f}, '
                    f'{candidate["frontier_y"]:.2f})'
                )

                continue

            candidates.append(candidate)

            self.get_logger().info(
                f'Candidate: '
                f'{candidate["size"]} cells, '
                f'frontier=({candidate["frontier_x"]:.2f}, '
                f'{candidate["frontier_y"]:.2f}), '
                f'safe target=({candidate["x"]:.2f}, '
                f'{candidate["y"]:.2f}), '
                f'stand-off={candidate["stand_off"]:.2f} m'
            )

        if not candidates:
            self.register_no_reachable_round(
                'No useful unvisited safe frontier candidates found.'
            )
            return

        self.pending_candidates = candidates
        self.reachable_candidates = []
        self.path_check_active = True

        self.check_next_candidate()

    def register_no_reachable_round(
        self,
        reason
    ):

        self.no_reachable_rounds += 1

        self.get_logger().warn(
            f'{reason} '
            f'No-reachable round '
            f'{self.no_reachable_rounds}/'
            f'{self.max_no_reachable_rounds}.'
        )

        if (
            self.no_reachable_rounds
            >= self.max_no_reachable_rounds
        ):

            self.exploration_complete = True

            self.get_logger().info(
                'EXPLORATION COMPLETE: '
                'no reachable unexplored frontier remained '
                f'for {self.max_no_reachable_rounds} '
                'consecutive analysis rounds.'
            )

            self.get_logger().info(
                f'Total navigation attempts: '
                f'{self.goal_attempts}; '
                f'visited regions: '
                f'{len(self.visited_frontiers)}; '
                f'failed regions: '
                f'{len(self.failed_frontiers)}'
            )

            self.request_map_save()


    def request_map_save(self):
        """Save the completed SLAM map through Nav2 SaveMap service."""

        if self.map_saved or self.map_save_in_progress:
            return

        if not self.save_map_client.wait_for_service(
            timeout_sec=2.0
        ):
            self.get_logger().error(
                'MAP SAVE FAILED: /map_saver/save_map '
                'service is not available.'
            )
            return

        map_path = Path(self.map_url)
        map_path.parent.mkdir(
            parents=True,
            exist_ok=True
        )

        request = SaveMap.Request()
        request.map_topic = '/map'
        request.map_url = self.map_url
        request.image_format = 'pgm'
        request.map_mode = 'trinary'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65

        self.map_save_in_progress = True

        self.get_logger().info(
            f'Saving completed map to: {self.map_url}'
        )

        future = self.save_map_client.call_async(request)
        future.add_done_callback(
            self.map_save_response_callback
        )


    def map_save_response_callback(self, future):
        self.map_save_in_progress = False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'MAP SAVE FAILED: service exception: {exc}'
            )
            return

        if response is not None and response.result:
            self.map_saved = True
            self.get_logger().info(
                f'MAP SAVE SUCCESS: {self.map_url}.pgm '
                f'and {self.map_url}.yaml'
            )
        else:
            self.get_logger().error(
                'MAP SAVE FAILED: SaveMap returned result=False.'
            )


    def representative_cell(self, cluster):

        mean_row = sum(
            row for row, _ in cluster
        ) / len(cluster)

        mean_col = sum(
            col for _, col in cluster
        ) / len(cluster)

        return min(
            cluster,
            key=lambda cell:
                (cell[0] - mean_row) ** 2
                + (cell[1] - mean_col) ** 2
        )

    def frontier_was_attempted(
        self,
        candidate
    ):

        x = candidate['frontier_x']
        y = candidate['frontier_y']

        for old_x, old_y in self.visited_frontiers:

            if math.hypot(
                x - old_x,
                y - old_y
            ) < self.visited_exclusion_radius:

                return True

        for old_x, old_y in self.failed_frontiers:

            if math.hypot(
                x - old_x,
                y - old_y
            ) < self.failed_exclusion_radius:

                return True

        return False


    def build_standoff_candidate(
        self,
        cluster,
        map_msg
    ):

        frontier_row, frontier_col = (
            self.representative_cell(cluster)
        )

        width = map_msg.info.width
        height = map_msg.info.height
        data = map_msg.data
        resolution = map_msg.info.resolution

        # Estimate the direction from the frontier cell
        # toward unknown space.
        unknown_vectors = []

        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):

                if dr == 0 and dc == 0:
                    continue

                rr = frontier_row + dr
                cc = frontier_col + dc

                if (
                    0 <= rr < height
                    and 0 <= cc < width
                ):
                    idx = rr * width + cc

                    if data[idx] == -1:
                        unknown_vectors.append(
                            (dr, dc)
                        )

        if not unknown_vectors:
            return None

        unknown_dr = sum(
            dr for dr, _ in unknown_vectors
        ) / len(unknown_vectors)

        unknown_dc = sum(
            dc for _, dc in unknown_vectors
        ) / len(unknown_vectors)

        norm = math.hypot(
            unknown_dr,
            unknown_dc
        )

        if norm < 1e-6:
            return None

        # Opposite of unknown direction = inward,
        # toward already known free space.
        inward_dr = -unknown_dr / norm
        inward_dc = -unknown_dc / norm

        desired_steps = max(
            1,
            int(round(0.50 / resolution))
        )

        min_steps = max(
            1,
            int(round(0.30 / resolution))
        )

        max_steps = max(
            desired_steps,
            int(round(0.75 / resolution))
        )

        clearance_cells = max(
            2,
            int(round(0.15 / resolution))
        )

        def is_safe_free(row, col):

            if not (
                0 <= row < height
                and 0 <= col < width
            ):
                return False

            idx = row * width + col

            if data[idx] != 0:
                return False

            # Require a small known-free neighborhood.
            for dr in range(
                -clearance_cells,
                clearance_cells + 1
            ):
                for dc in range(
                    -clearance_cells,
                    clearance_cells + 1
                ):

                    if (
                        dr * dr + dc * dc
                        > clearance_cells * clearance_cells
                    ):
                        continue

                    rr = row + dr
                    cc = col + dc

                    if not (
                        0 <= rr < height
                        and 0 <= cc < width
                    ):
                        return False

                    value = data[
                        rr * width + cc
                    ]

                    if value != 0:
                        return False

            return True

        step_candidates = list(
            range(min_steps, max_steps + 1)
        )

        step_candidates.sort(
            key=lambda s:
                abs(s - desired_steps)
        )

        target_cell = None

        for step in step_candidates:

            projected_row = int(round(
                frontier_row
                + inward_dr * step
            ))

            projected_col = int(round(
                frontier_col
                + inward_dc * step
            ))

            # Small local search around projected target.
            local_options = []

            for dr in range(-2, 3):
                for dc in range(-2, 3):

                    row = projected_row + dr
                    col = projected_col + dc

                    if is_safe_free(row, col):

                        distance = (
                            dr * dr
                            + dc * dc
                        )

                        local_options.append(
                            (
                                distance,
                                row,
                                col
                            )
                        )

            if local_options:

                local_options.sort(
                    key=lambda item: item[0]
                )

                _, target_row, target_col = (
                    local_options[0]
                )

                target_cell = (
                    target_row,
                    target_col
                )

                break

        if target_cell is None:
            return None

        target_row, target_col = target_cell

        frontier_x, frontier_y = (
            self.cell_to_world(
                frontier_row,
                frontier_col,
                map_msg
            )
        )

        target_x, target_y = (
            self.cell_to_world(
                target_row,
                target_col,
                map_msg
            )
        )

        # Final orientation points toward the frontier.
        yaw = math.atan2(
            frontier_y - target_y,
            frontier_x - target_x
        )

        stand_off = math.hypot(
            frontier_x - target_x,
            frontier_y - target_y
        )

        return {
            'size': len(cluster),

            'frontier_row': frontier_row,
            'frontier_col': frontier_col,
            'frontier_x': frontier_x,
            'frontier_y': frontier_y,

            'row': target_row,
            'col': target_col,
            'x': target_x,
            'y': target_y,

            'yaw': yaw,
            'stand_off': stand_off
        }

    def check_next_candidate(self):

        if not self.pending_candidates:

            self.finish_path_checks()
            return

        candidate = self.pending_candidates.pop(0)

        goal = ComputePathToPose.Goal()

        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.goal.pose.position.x = candidate['x']
        goal.goal.pose.position.y = candidate['y']
        goal.goal.pose.position.z = 0.0

        goal.goal.pose.orientation.x = 0.0
        goal.goal.pose.orientation.y = 0.0
        goal.goal.pose.orientation.z = math.sin(
            candidate['yaw'] / 2.0
        )
        goal.goal.pose.orientation.w = math.cos(
            candidate['yaw'] / 2.0
        )

        goal.planner_id = ''
        goal.use_start = False

        self.current_candidate = candidate

        self.get_logger().info(
            f'Checking path to '
            f'({candidate["x"]:.2f}, '
            f'{candidate["y"]:.2f}) ...'
        )

        future = self.path_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.path_goal_response_callback
        )

    def path_goal_response_callback(
        self,
        future
    ):

        try:
            goal_handle = future.result()

        except Exception as ex:
            self.get_logger().warn(
                f'Path goal request failed: {ex}'
            )

            self.check_next_candidate()
            return

        if not goal_handle.accepted:

            self.get_logger().warn(
                'Planner rejected candidate.'
            )

            self.check_next_candidate()
            return

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.path_result_callback
        )

    def path_result_callback(
        self,
        future
    ):

        candidate = self.current_candidate

        try:
            wrapped_result = future.result()
            path = wrapped_result.result.path

        except Exception as ex:

            self.get_logger().warn(
                f'Planning failed for '
                f'({candidate["x"]:.2f}, '
                f'{candidate["y"]:.2f}): {ex}'
            )

            self.check_next_candidate()
            return

        if len(path.poses) < 2:

            self.get_logger().warn(
                f'UNREACHABLE candidate '
                f'({candidate["x"]:.2f}, '
                f'{candidate["y"]:.2f})'
            )

            self.check_next_candidate()
            return

        path_length = 0.0

        for i in range(
            1,
            len(path.poses)
        ):

            p0 = path.poses[i - 1].pose.position
            p1 = path.poses[i].pose.position

            path_length += math.hypot(
                p1.x - p0.x,
                p1.y - p0.y
            )

        candidate['path_length'] = path_length

        self.reachable_candidates.append(
            candidate
        )

        self.get_logger().info(
            f'REACHABLE candidate '
            f'({candidate["x"]:.2f}, '
            f'{candidate["y"]:.2f}), '
            f'path length={path_length:.2f} m'
        )

        self.check_next_candidate()

    def finish_path_checks(self):

        if not self.reachable_candidates:

            self.path_check_active = False

            self.register_no_reachable_round(
                'No reachable frontier among tested candidates.'
            )

            return

        for candidate in self.reachable_candidates:

            candidate['score'] = (
                candidate['size']
                / (1.0 + candidate['path_length'])
            )

        best = max(
            self.reachable_candidates,
            key=lambda c: c['score']
        )

        self.get_logger().info(
            'BEST SAFE STAND-OFF TARGET: '
            f'target=({best["x"]:.2f}, '
            f'{best["y"]:.2f}), '
            f'frontier=({best["frontier_x"]:.2f}, '
            f'{best["frontier_y"]:.2f}), '
            f'stand-off={best["stand_off"]:.2f} m, '
            f'cluster={best["size"]} cells, '
            f'path={best["path_length"]:.2f} m, '
            f'score={best["score"]:.2f}'
        )

        self.path_check_active = False
        self.no_reachable_rounds = 0

        self.get_logger().info(
            'AUTONOMOUS EXPLORATION: '
            f'sending navigation goal '
            f'{self.goal_attempts + 1}.'
        )

        self.send_navigation_goal(best)

    def send_navigation_goal(self, candidate):

        if not self.nav_client.wait_for_server(
            timeout_sec=0.0
        ):
            self.get_logger().warn(
                'NavigateToPose server unavailable.'
            )
            return

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.pose.pose.position.x = candidate['x']
        goal.pose.pose.position.y = candidate['y']
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(
            candidate['yaw'] / 2.0
        )
        goal.pose.pose.orientation.w = math.cos(
            candidate['yaw'] / 2.0
        )

        self.nav_goal_active = True
        self.active_nav_candidate = candidate
        self.current_nav_candidate = candidate

        self.get_logger().info(
            'AUTONOMOUS TEST: navigating to frontier '
            f'({candidate["x"]:.2f}, '
            f'{candidate["y"]:.2f})'
        )

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.navigation_feedback_callback
        )

        future.add_done_callback(
            self.navigation_goal_response_callback
        )

    def navigation_goal_response_callback(
        self,
        future
    ):

        try:
            goal_handle = future.result()

        except Exception as ex:

            self.get_logger().error(
                f'Navigation goal request failed: {ex}'
            )

            self.finish_navigation_attempt(
                success=False,
                status_label='goal request error'
            )

            return

        if not goal_handle.accepted:

            self.get_logger().warn(
                'Navigation goal rejected.'
            )

            self.finish_navigation_attempt(
                success=False,
                status_label='rejected'
            )

            return

        self.get_logger().info(
            'Navigation goal accepted.'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.navigation_result_callback
        )


    def navigation_feedback_callback(
        self,
        feedback_msg
    ):

        # Nav2 can publish feedback very quickly.
        # Print only about once per second.
        now_ns = (
            self.get_clock().now().nanoseconds
        )

        if (
            now_ns - self.last_feedback_log_ns
            < 1_000_000_000
        ):
            return

        self.last_feedback_log_ns = now_ns

        feedback = feedback_msg.feedback

        self.get_logger().info(
            'Navigation distance remaining: '
            f'{feedback.distance_remaining:.2f} m'
        )


    def navigation_result_callback(
        self,
        future
    ):

        try:
            wrapped_result = future.result()
            status = wrapped_result.status

        except Exception as ex:

            self.get_logger().error(
                f'Navigation result failed: {ex}'
            )

            self.finish_navigation_attempt(
                success=False,
                status_label='result error'
            )

            return

        if status == 4:

            self.get_logger().info(
                'AUTONOMOUS GOAL SUCCESS: '
                'safe frontier stand-off reached.'
            )

            self.finish_navigation_attempt(
                success=True,
                status_label='SUCCEEDED'
            )

        else:

            self.get_logger().warn(
                'AUTONOMOUS GOAL FAILED '
                f'with status={status}'
            )

            self.finish_navigation_attempt(
                success=False,
                status_label=f'status={status}'
            )


    def finish_navigation_attempt(
        self,
        success,
        status_label
    ):

        candidate = self.active_nav_candidate

        if candidate is not None:

            frontier_point = (
                candidate['frontier_x'],
                candidate['frontier_y']
            )

            if success:
                self.visited_frontiers.append(
                    frontier_point
                )
            else:
                self.failed_frontiers.append(
                    frontier_point
                )

        self.goal_attempts += 1

        self.nav_goal_active = False
        self.active_nav_candidate = None

        self.get_logger().info(
            f'Navigation attempt '
            f'{self.goal_attempts} finished: '
            f'{status_label}'
        )

        self.get_logger().info(
            'Waiting for updated SLAM map before '
            'selecting the next frontier.'
        )


    def cluster_frontiers(
        self,
        frontier_cells
    ):

        remaining = set(frontier_cells)
        clusters = []

        while remaining:

            start = remaining.pop()

            queue = deque([start])
            cluster = [start]

            while queue:

                row, col = queue.popleft()

                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):

                        if dr == 0 and dc == 0:
                            continue

                        neighbor = (
                            row + dr,
                            col + dc
                        )

                        if neighbor in remaining:

                            remaining.remove(
                                neighbor
                            )

                            queue.append(
                                neighbor
                            )

                            cluster.append(
                                neighbor
                            )

            clusters.append(
                cluster
            )

        return clusters

    def cell_to_world(
        self,
        row,
        col,
        map_msg
    ):

        resolution = map_msg.info.resolution
        origin = map_msg.info.origin

        local_x = (
            col + 0.5
        ) * resolution

        local_y = (
            row + 0.5
        ) * resolution

        q = origin.orientation

        yaw = math.atan2(
            2.0 * (
                q.w * q.z
                + q.x * q.y
            ),
            1.0 - 2.0 * (
                q.y * q.y
                + q.z * q.z
            )
        )

        world_x = (
            origin.position.x
            + math.cos(yaw) * local_x
            - math.sin(yaw) * local_y
        )

        world_y = (
            origin.position.y
            + math.sin(yaw) * local_x
            + math.cos(yaw) * local_y
        )

        return world_x, world_y


def main(args=None):

    rclpy.init(args=args)

    node = FrontierExplorer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
