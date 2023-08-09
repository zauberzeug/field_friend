import logging
from typing import TYPE_CHECKING

import numpy as np
from shapely import affinity
from shapely.geometry import LineString, MultiLineString, Polygon
from shapely.ops import unary_union

if TYPE_CHECKING:
    from ..automations import Mowing


class CoveragePlanner:
    OBSTACLE_PADDING = 0.7
    NUM_OUTER_LANES = 3

    def __init__(self, mowing: 'Mowing') -> None:
        self.log = logging.getLogger('field_friend.coverage_planner')
        self.mowing = mowing

    def decompose_into_lanes(self) -> (list[list[LineString]], list[list[LineString]]):
        self.log.info('decomposing field into lanes')
        self.field = self.mowing.field
        self._determine_area_and_distance()
        inner_lanes = self._determine_inner_lanes()
        outer_lanes = self._determine_outer_lanes()
        return (inner_lanes, outer_lanes)

    def _determine_area_and_distance(self) -> None:
        self.min_x = self.field.outline[0].x
        self.min_y = self.field.outline[0].y
        translated_field = Polygon([(point.x - self.min_x, point.y - self.min_y) for point in self.field.outline])

        # Determine the unit vector direction of the first line of the polygon
        self.p1, self.p2 = translated_field.exterior.coords[:2]
        self.direction = np.array([self.p2[0] - self.p1[0], self.p2[1] - self.p1[1]])
        self.direction /= np.linalg.norm(self.direction)

        # Calculate the angle of rotation based on the direction
        self.theta = np.arctan2(self.direction[1], self.direction[0])

        # Rotate the translated_field
        rotated_field = affinity.rotate(translated_field, -self.theta, origin=(0, 0), use_radians=True)

        padded_polygon = rotated_field.buffer(-self.mowing.padding - self.mowing.turning_radius)

        obstacle_polygons = []  # List to hold all obstacle polygons
        if self.field.obstacles:  # Check if the obstacles list is not empty
            for obstacle in self.field.obstacles:
                translated_obstacle = Polygon([(point.x - self.min_x, point.y - self.min_y)
                                              for point in obstacle.points])
                rotated_obstacle = affinity.rotate(translated_obstacle, -self.theta, origin=(0, 0), use_radians=True)
                padded_obstacle = rotated_obstacle.buffer(
                    self.mowing.padding + self.mowing.turning_radius + self.OBSTACLE_PADDING)
                obstacle_polygons.append(padded_obstacle)
        obstacles_union = unary_union(obstacle_polygons) if obstacle_polygons else None

        if obstacles_union is not None:
            self.navigable_area = padded_polygon.difference(obstacles_union)
        else:
            self.navigable_area = padded_polygon

        self.direction = np.array([1, 0])
        self.perp_direction = np.array([0, 1])

        # Determine the minimum and maximum projections along the direction and perpendicular direction
        min_proj = max_proj = np.dot(np.array(self.p1), self.direction)
        min_perp_proj = max_perp_proj = np.dot(np.array(self.p1), self.perp_direction)
        for coord in padded_polygon.exterior.coords:
            proj = np.dot(coord, self.direction)
            perp_proj = np.dot(coord, self.perp_direction)
            min_proj = min(min_proj, proj)
            max_proj = max(max_proj, proj)
            min_perp_proj = min(min_perp_proj, perp_proj)
            max_perp_proj = max(max_perp_proj, perp_proj)

        self.min_proj = min_proj
        self.max_proj = max_proj
        self.min_perp_proj = min_perp_proj
        self.max_perp_proj = max_perp_proj

        self.log.info(f'configured lane distance: {self.mowing.lane_distance}')
        number_of_lanes = int(np.ceil((max_perp_proj - min_perp_proj) / self.mowing.lane_distance))
        self.mowing.lane_distance = (max_perp_proj - min_perp_proj) / number_of_lanes
        self.log.info(f'corrected lane distance: {self.mowing.lane_distance}')

    def _determine_inner_lanes(self) -> None:
        lanes_groups = []  # List to hold lists of lanes
        current_groups = [[]]  # Current groups of lanes
        is_multiline = False

        length = self.min_perp_proj
        while length <= self.max_perp_proj:
            start_point = np.array(self.p1) + length * self.perp_direction + self.min_proj * self.direction
            end_point = np.array(self.p1) + length * self.perp_direction + self.max_proj * self.direction
            line = LineString([start_point, end_point])

            # Intersect the line with the inverse of the obstacle polygon
            intersection = self.navigable_area.intersection(line)
            if intersection and not intersection.is_empty:
                if isinstance(intersection, MultiLineString):
                    if not current_groups or len(current_groups) != len(intersection.geoms):
                        for group in current_groups:
                            lanes_groups.append(group)
                        current_groups = [[] for _ in range(len(intersection.geoms))]
                    for i, line_segment in enumerate(intersection.geoms):
                        current_groups[i].append(line_segment)
                    is_multiline = True
                elif isinstance(intersection, LineString):
                    if is_multiline:
                        for group in current_groups:
                            lanes_groups.append(group)
                        current_groups = [[]]  # Create new group for LineString
                        is_multiline = False
                    current_groups[0].append(intersection)

            length += self.mowing.lane_distance

        for group in current_groups:
            lanes_groups.append(group)

        lanes_groups = [
            [affinity.translate(  # Translating the lanes back to their original position
                affinity.rotate(  # Rotating the lanes back to their original orientation
                    line,
                    self.theta,
                    origin=(0, 0),
                    use_radians=True
                ),
                xoff=self.min_x,
                yoff=self.min_y
            ) for line in group]
            for group in lanes_groups
        ]

        return lanes_groups

    def _determine_outer_lanes(self) -> None:
        # Create line segments along the outline
        outer_lanes_groups = []
        for i in range(self.NUM_OUTER_LANES):
            outer_lanes = []
            padded_polygon = Polygon([(point.x, point.y) for point in self.field.outline]
                                     ).buffer(-self.mowing.padding-self.mowing.lane_distance*i)
            outline_coords = list(padded_polygon.exterior.coords)
            for i in range(len(outline_coords) - 1):
                p1 = outline_coords[i]
                p2 = outline_coords[i + 1]
                line = LineString([p1, p2])
                outer_lanes.append(line)
            outer_lanes_groups.append(outer_lanes)

        return outer_lanes_groups
