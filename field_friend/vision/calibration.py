# pylint: disable=no-member
# TODO: how to get better type checking for cv2?
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Self

import cv2
import numpy as np
from rosys.geometry import Frame3d, Point, Point3d
from rosys.vision import Calibration, ImageSize

DOT_DISTANCE = 0.055


@dataclass(kw_only=True, slots=True)
class Contour:
    points: np.ndarray

    @property
    def x(self) -> float:
        return self.points[:, 0, 0].mean()

    @property
    def y(self) -> float:
        return self.points[:, 0, 1].mean()

    def is_ellipse(self, shape_threshold=0.2) -> bool:
        ellipse = cv2.fitEllipse(self.points)
        center = (int(ellipse[0][0]), int(ellipse[0][1]))
        axes = (int(ellipse[1][0]/2), int(ellipse[1][1]/2))
        angle = int(ellipse[2])
        ellipse_contour = np.array(cv2.ellipse2Poly(center, axes, angle, 0, 360, 1))
        shape_similarity = cv2.matchShapes(self.points, ellipse_contour, cv2.CONTOURS_MATCH_I3, 0.0)
        return shape_similarity < shape_threshold

    # TODO: return type was tuple[Sequence[float], Sequence[int], float] <- do we now break things with the new one
    def fit_ellipse(self) -> tuple[Sequence[float], Sequence[float], float]:
        return cv2.fitEllipse(self.points)

    def contains(self, x: float, y: float) -> bool:
        return cv2.pointPolygonTest(self.points, (float(x), float(y)), False) >= 0


@dataclass(kw_only=True, slots=True)
class Dot:
    x: float
    y: float
    is_refined: bool = False

    def snap_to(self, contour: Contour) -> None:
        (x, y), *_ = contour.fit_ellipse()
        self.x = x
        self.y = y
        self.is_refined = True


class Network:

    def __init__(self, contours: list[Contour], image_size: ImageSize) -> None:
        self.contours = contours
        self.image_size = image_size
        self.dots: dict[tuple[float, float, float], Dot] = {}

    @classmethod
    def from_img(cls, img: np.ndarray) -> Self:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.blur(gray, (101, 101))
        divided = cv2.divide(gray, blurred, scale=255)
        _, thresh = cv2.threshold(divided, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if 100 < cv2.contourArea(c) < 2000]
        contours = [c for c in contours if cv2.arcLength(c, True) / cv2.contourArea(c) < 0.5]

        network = cls([Contour(points=points) for points in contours if Contour(points=points).is_ellipse(shape_threshold=0.1)],
                      ImageSize(width=img.shape[1], height=img.shape[0]))
        cx, cy = img.shape[1] // 2, img.shape[0] // 2

        distances = [np.linalg.norm(np.array([cx, cy]) - np.array([c.x, c.y])) for c in network.contours]
        contour0_index = np.argmin(distances)
        contour0 = network.contours[contour0_index]
        remaining_contours = [c for i, c in enumerate(network.contours) if i != contour0_index]
        if not remaining_contours:
            raise ValueError('No remaining contours to create network')
        contour1 = remaining_contours[np.argmin(
            [np.linalg.norm(np.array([cx, cy]) - np.array([c.x, c.y])) for c in remaining_contours])]

        dist = np.sqrt((contour0.x - contour1.x)**2 + (contour0.y - contour1.y)**2)
        for i in [0, 1]:
            for j in [0, 1]:
                dot = Dot(x=contour0.x + j * dist, y=contour0.y + i * dist)
                network.try_refine(dot, tolerance=50)
                network.dots[(i, j, 0)] = dot
        network.auto_grow()
        return network

    def try_refine(self, dot: Dot, *, tolerance: float = 0) -> None:
        # TODO: can we get rid of the pylint disable? -> no, we live with it, but maybe document it?
        for dx in np.arange(0, tolerance + 1, 10):  # pylint: disable=too-many-nested-blocks
            for dy in np.arange(0, tolerance + 1, 10):
                for sign_x in [-1, 1] if dx else [0]:
                    for sign_y in [-1, 1] if dy else [0]:
                        for contour in self.contours:
                            if contour.contains(dot.x + sign_x * dx, dot.y + sign_y * dy):
                                dot.snap_to(contour)
                                return

    def auto_grow(self) -> None:
        while self.try_grow():
            continue

    def try_grow(self) -> bool:
        for i, j, k in list(self.dots):
            for di, dj, dk in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)]:
                if di != 0 and not -7 <= i + di <= 7:
                    continue
                if dj != 0 and not -3 <= j + dj <= 3:
                    continue
                if self._try_step((i - di, j - dj, k - dk), (i, j, k), (i + di, j + dj, k + dk), straight=True):
                    return True
            # for calibration pattern version 1.0
            if j == -3 and self._try_step((i, j + 1, k), (i, j, k), (i + 0.5, j - 0.5, k + 0.5), straight=False):
                return True
            if j == -3.5 and self._try_step((i - 0.5, j + 0.5, k - 0.5), (i, j, k), (i, j, k + 1), straight=True):
                return True
            if j == 3 and self._try_step((i, j - 1, k), (i, j, k), (i + 0.5, j + 0.5, k + 0.5), straight=False):
                return True
            if j == 3.5 and self._try_step((i - 0.5, j - 0.5, k - 0.5), (i, j, k), (i, j, k + 1), straight=True):
                return True

            # for calibration pattern version 2.0
            # if j == -3 and self._try_step((i, j + 1, k), (i, j, k), (i + 0.5, j - 0.4, k + 0.6), straight=False):
            #     return True
            # if j == -3.4 and self._try_step((i - 0.5, j + 0.5, k - 0.5), (i, j, k), (i, j, k + 1), straight=True):
            #     return True
            # if j == 3 and self._try_step((i, j - 1, k), (i, j, k), (i + 0.5, j + 0.4, k + 0.6), straight=False):
            #     return True
            # if j == 3.4 and self._try_step((i - 0.5, j - 0.5, k - 0.5), (i, j, k), (i, j, k + 1), straight=True):
            #     return True
        return False

    def _try_step(self,
                  source1: tuple[float, float, float],
                  source2: tuple[float, float, float],
                  target: tuple[float, float, float], *,
                  straight: bool) -> bool:
        if source1 not in self.dots:
            return False
        if source2 not in self.dots:
            return False
        if target in self.dots:
            return False
        if not self.dots[source1].is_refined:
            return False
        if not self.dots[source2].is_refined:
            return False
        if straight:
            new_dot = Dot(x=2 * self.dots[source2].x - self.dots[source1].x,
                          y=2 * self.dots[source2].y - self.dots[source1].y)
        else:
            dxy = np.sqrt((self.dots[source2].x - self.dots[source1].x)**2 +
                          (self.dots[source2].y - self.dots[source1].y)**2)
            dij = np.sqrt((source2[0] - source1[0])**2 +
                          (source2[1] - source1[1])**2)
            new_dot = Dot(x=self.dots[source2].x + dxy / dij * (target[1] - source2[1]),
                          y=self.dots[source2].y + dxy / dij * (0.7 * (target[0] - source2[0]) +
                                                                0.7 * (target[2] - source2[2])))
        self.try_refine(new_dot, tolerance=9)
        self.dots[target] = new_dot
        return True

    def shift(self, *, di: float = 0, dj: float = 0, dk: float = 0) -> None:
        self.dots = {(i + di, j + dj, k + dk): dot for (i, j, k), dot in self.dots.items()}

    def calibrate(self, f0: float, prediction_frame: Frame3d) -> Calibration:
        world_points = [Point3d(x=i * DOT_DISTANCE,
                                y=j * DOT_DISTANCE,
                                z=k * DOT_DISTANCE) for (i, j, k), dot in self.dots.items() if dot.is_refined]
        image_points = [Point(x=dot.x, y=dot.y) for dot in self.dots.values() if dot.is_refined]
        return Calibration.from_points(world_points, image_points, image_size=self.image_size, f0=f0, frame=prediction_frame)
