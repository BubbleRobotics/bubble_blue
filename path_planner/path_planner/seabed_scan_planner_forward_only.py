#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy

from path_planner.seabed_scan_planner import SeabedScanPlanner


class SeabedScanPlannerForwardOnly(SeabedScanPlanner):
    def __init__(self):
        super().__init__()

    def build_raster_waypoints(
        self, origin_x: float, origin_y: float, depth_down: float
    ) -> List[Tuple[float, float, float]]:
        spacing = max(self.lane_spacing_m, 1e-3)
        width = max(self.scan_width_m, 0.0)
        height = max(self.scan_height_m, 0.0)

        scan_heading = self.fixed_yaw_rad
        forward_x = math.cos(scan_heading)
        forward_y = math.sin(scan_heading)
        lateral_x = math.cos(scan_heading + math.pi / 2.0)
        lateral_y = math.sin(scan_heading + math.pi / 2.0)

        y_offsets = []
        current_y = 0.0
        while current_y < height + 1e-9:
            y_offsets.append(min(current_y, height))
            current_y += spacing
        if not y_offsets:
            y_offsets.append(0.0)
        if abs(y_offsets[-1] - height) > 1e-9:
            y_offsets.append(height)

        def to_world(x_local: float, y_local: float) -> Tuple[float, float, float]:
            x_world = origin_x + x_local * forward_x + y_local * lateral_x
            y_world = origin_y + x_local * forward_y + y_local * lateral_y
            return (x_world, y_world, depth_down)

        waypoints: List[Tuple[float, float, float]] = []
        for row_idx, y_offset in enumerate(y_offsets):
            if row_idx % 2 == 0:
                waypoints.append(to_world(0.0, y_offset))
                if width > 0.0:
                    waypoints.append(to_world(width, y_offset))
            else:
                waypoints.append(to_world(width, y_offset))
                if width > 0.0:
                    waypoints.append(to_world(0.0, y_offset))

        return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = SeabedScanPlannerForwardOnly()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
