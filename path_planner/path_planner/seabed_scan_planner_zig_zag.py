#!/usr/bin/env python3
from typing import List, Tuple

import rclpy

from path_planner.seabed_scan_planner import SeabedScanPlanner


class SeabedScanPlannerZigZag(SeabedScanPlanner):
    def __init__(self):
        super().__init__()

    def build_raster_waypoints(
        self, origin_x: float, origin_y: float, depth_down: float
    ) -> List[Tuple[float, float, float]]:
        spacing = max(self.lane_spacing_m, 1e-3)
        width = max(self.scan_width_m, 0.0)
        height = max(self.scan_height_m, 0.0)

        y_offsets = []
        current_y = 0.0
        while current_y < height + 1e-9:
            y_offsets.append(min(current_y, height))
            current_y += spacing
        if not y_offsets:
            y_offsets.append(0.0)
        if abs(y_offsets[-1] - height) > 1e-9:
            y_offsets.append(height)

        x_start = origin_x
        x_end = origin_x + width
        waypoints: List[Tuple[float, float, float]] = []
        lateral_sign = self.get_lateral_direction_sign()

        # Build a true zig-zag path: traverse the first lane horizontally,
        # then move diagonally to the opposite side of each next lane.
        first_row_y = origin_y + lateral_sign * y_offsets[0]
        waypoints.append((x_start, first_row_y, depth_down))
        if width > 0.0:
            waypoints.append((x_end, first_row_y, depth_down))

        for row_idx, y_offset in enumerate(y_offsets[1:], start=1):
            row_y = origin_y + lateral_sign * y_offset
            x_target = x_start if row_idx % 2 == 1 else x_end
            waypoints.append((x_target, row_y, depth_down))

        return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = SeabedScanPlannerZigZag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
