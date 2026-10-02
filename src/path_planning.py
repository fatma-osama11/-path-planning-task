from __future__ import annotations

import math
from typing import List, Tuple

from src.models import CarPose, Cone, Path2D


# ---------------------------------------------------------------------------
# Assumptions (see README for details)
# ---------------------------------------------------------------------------
TRACK_WIDTH = 3.0      # assumed track width [m] when only one side is visible
MAX_GATE_WIDTH = 6.0   # a blue and a yellow cone farther apart than this are not paired
STEP = 0.5             # distance between consecutive path points [m]
MIN_LENGTH = 7.0       # we extend the path until it is at least this long [m]
MAX_LENGTH = 10.0      # the path is cut if it becomes longer than this [m]
MAX_CURVE_RADIUS = 30.0  # curves wider than this are treated as straight [m]

BLUE = 1    # left side
YELLOW = 0  # right side


class PathPlanning:
    """Student-implemented path planner.

    Idea (centerline planning):
      1. Split the cones into blue (left) and yellow (right).
      2. Pair each blue cone with the closest yellow cone ("gate"),
         the midpoint of a gate is a point on the track center.
      3. A cone without a partner is shifted towards the track center
         by half the track width.
      4. Sort the center points by distance from the car.
      5. Connect car -> center points with smooth curves that start in
         the car heading, then continue in the track direction until
         the path is long enough.
      6. Re-sample the result so that points are every 0.5 m.

    Part 2 (3 cones on one side):
      Three cones define a circle, so we can estimate how much the track
      curves. The path then keeps following that curve (a circle with the
      same center, through the middle of the track) instead of going straight.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    # ------------------------------------------------------------------
    # Main function
    # ------------------------------------------------------------------
    def generatePath(self) -> Path2D:
        car = (self.car_pose.x, self.car_pose.y)
        heading = (math.cos(self.car_pose.yaw), math.sin(self.car_pose.yaw))

        # Step 1: split cones by color
        blue = []
        yellow = []
        for cone in self.cones:
            if cone.color == BLUE:
                blue.append((cone.x, cone.y))
            else:
                yellow.append((cone.x, cone.y))

        # Step 2: pair blue and yellow cones into gates
        gates, single_blue, single_yellow = self._match_cones(blue, yellow)

        # Half of the track width: use the real gate width if we saw a gate
        half_width = TRACK_WIDTH / 2.0
        if len(gates) > 0:
            total = 0.0
            for b, y in gates:
                total += self._dist(b, y)
            half_width = (total / len(gates)) / 2.0

        # Each center point is stored together with the track direction there
        centers = []

        for b, y in gates:
            mid = ((b[0] + y[0]) / 2.0, (b[1] + y[1]) / 2.0)
            centers.append((mid, self._gate_direction(b, y)))

        for cone in single_blue:
            direction = self._side_direction(cone, blue, gates, heading)
            right = (direction[1], -direction[0])   # blue is on the left -> center is to its right
            point = (cone[0] + right[0] * half_width, cone[1] + right[1] * half_width)
            centers.append((point, direction))

        for cone in single_yellow:
            direction = self._side_direction(cone, yellow, gates, heading)
            left = (-direction[1], direction[0])    # yellow is on the right -> center is to its left
            point = (cone[0] + left[0] * half_width, cone[1] + left[1] * half_width)
            centers.append((point, direction))

        # Step 3: sort center points by distance from the car
        centers.sort(key=lambda c: self._dist(car, c[0]))

        # Step 4: list the waypoints and the driving direction at each one
        #         (the car itself is the first waypoint, with its heading)
        waypoints = [car]
        directions = [heading]
        for point, direction in centers:
            waypoints.append(point)
            directions.append(direction)

        # Step 5: extend the path ahead so it is long enough.
        #   Part 2: if one side has 3+ cones we can see the curve of the track
        #   (circle through 3 cones) and keep following that curve.
        #   Otherwise we continue straight.
        length = self._polyline_length(waypoints)
        extra = max(MIN_LENGTH - length, 2.0)
        last = waypoints[-1]
        last_dir = directions[-1]

        circle = None
        if len(centers) > 0:
            circle = self._track_circle(blue, yellow)

        if circle is None:
            waypoints.append((last[0] + last_dir[0] * extra, last[1] + last_dir[1] * extra))
            directions.append(last_dir)
        else:
            arc_points, arc_dirs = self._follow_arc(circle, last, last_dir, extra)
            for k in range(len(arc_points)):
                waypoints.append(arc_points[k])
                directions.append(arc_dirs[k])

        # Step 6: connect the waypoints with smooth curves (cubic Bezier).
        #         The curve leaves each waypoint in its driving direction,
        #         so the path starts in the direction the car is facing.
        smooth = self._smooth_curve(waypoints, directions)

        # Step 7: re-sample every STEP meters, never longer than MAX_LENGTH
        return self._resample(smooth, STEP, MAX_LENGTH)

    # ------------------------------------------------------------------
    # Helper functions
    # ------------------------------------------------------------------
    def _match_cones(self, blue, yellow):
        """Greedy matching: the closest blue/yellow pair becomes a gate first."""
        pairs = []
        for i in range(len(blue)):
            for j in range(len(yellow)):
                d = self._dist(blue[i], yellow[j])
                if d <= MAX_GATE_WIDTH:
                    pairs.append((d, i, j))
        pairs.sort()

        used_blue = set()
        used_yellow = set()
        gates = []
        for d, i, j in pairs:
            if i in used_blue or j in used_yellow:
                continue
            used_blue.add(i)
            used_yellow.add(j)
            gates.append((blue[i], yellow[j]))

        single_blue = []
        for i in range(len(blue)):
            if i not in used_blue:
                single_blue.append(blue[i])

        single_yellow = []
        for j in range(len(yellow)):
            if j not in used_yellow:
                single_yellow.append(yellow[j])

        return gates, single_blue, single_yellow

    def _gate_direction(self, b, y):
        """Track direction through a gate.

        Blue is on the left, so the vector yellow->blue points to the left.
        Rotating it by -90 degrees gives the forward direction.
        """
        to_left = (b[0] - y[0], b[1] - y[1])
        forward = (to_left[1], -to_left[0])
        return self._normalize(forward)

    def _side_direction(self, cone, same_side, gates, heading):
        """Track direction near a cone that has no partner.

        1. If the same side has another cone, use the line between them
           (pointing away from the car).
        2. Otherwise use the direction of the nearest gate.
        3. Otherwise use the car heading.
        """
        car = (self.car_pose.x, self.car_pose.y)

        if len(same_side) >= 2:
            ordered = sorted(same_side, key=lambda p: self._dist(car, p))
            index = ordered.index(cone)
            if index == 0:
                a, b = ordered[0], ordered[1]
            else:
                a, b = ordered[index - 1], ordered[index]
            return self._normalize((b[0] - a[0], b[1] - a[1]))

        if len(gates) > 0:
            nearest = min(gates, key=lambda g: self._dist(cone, g[0]))
            return self._gate_direction(nearest[0], nearest[1])

        return heading

    def _track_circle(self, blue, yellow):
        """Part 2: estimate the track curve from 3 cones on the same side.

        Takes the side with more cones (needs at least 3), uses the 3 cones
        closest to the car and returns the circle (center, radius) that passes
        through them. Returns None if no side has 3 cones, or if the cones are
        (almost) on a straight line.
        """
        car = (self.car_pose.x, self.car_pose.y)

        side = blue
        if len(yellow) > len(blue):
            side = yellow
        if len(side) < 3:
            return None

        ordered = sorted(side, key=lambda p: self._dist(car, p))
        a, b, c = ordered[0], ordered[1], ordered[2]

        d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
        if abs(d) < 1e-6:
            return None   # the 3 cones are on a straight line

        a2 = a[0] * a[0] + a[1] * a[1]
        b2 = b[0] * b[0] + b[1] * b[1]
        c2 = c[0] * c[0] + c[1] * c[1]
        cx = (a2 * (b[1] - c[1]) + b2 * (c[1] - a[1]) + c2 * (a[1] - b[1])) / d
        cy = (a2 * (c[0] - b[0]) + b2 * (a[0] - c[0]) + c2 * (b[0] - a[0])) / d
        radius = self._dist((cx, cy), a)

        if radius > MAX_CURVE_RADIUS:
            return None   # very gentle curve: treat it as straight
        return ((cx, cy), radius)

    def _follow_arc(self, circle, start, start_dir, length):
        """Walk `length` meters along a circle parallel to the track curve.

        The circle has the same center as the cone circle but passes through
        `start` (the last center point), so it runs in the middle of the track.
        """
        center = circle[0]
        r = self._dist(center, start)
        angle = math.atan2(start[1] - center[1], start[0] - center[0])

        # turn left (counter-clockwise) or right (clockwise)?
        radial = (start[0] - center[0], start[1] - center[1])
        cross = radial[0] * start_dir[1] - radial[1] * start_dir[0]
        sign = 1.0
        if cross < 0:
            sign = -1.0

        points = []
        dirs = []
        count = int(math.ceil(length / STEP))
        for i in range(1, count + 1):
            theta = angle + sign * (i * STEP) / r
            points.append((center[0] + r * math.cos(theta), center[1] + r * math.sin(theta)))
            dirs.append((-sign * math.sin(theta), sign * math.cos(theta)))
        return points, dirs

    def _smooth_curve(self, waypoints, directions):
        """Cubic Bezier between every two waypoints.

        For the piece from A to B:
          P0 = A
          P1 = A + (direction at A) * h
          P2 = B - (direction at B) * h
          P3 = B
        h is a "handle" length: 40% of the distance between A and B.
        """
        points = [waypoints[0]]
        samples = 20
        for k in range(len(waypoints) - 1):
            a = waypoints[k]
            b = waypoints[k + 1]
            da = directions[k]
            db = directions[k + 1]
            h = 0.4 * self._dist(a, b)

            p0 = a
            p1 = (a[0] + da[0] * h, a[1] + da[1] * h)
            p2 = (b[0] - db[0] * h, b[1] - db[1] * h)
            p3 = b

            for i in range(1, samples + 1):
                t = i / samples
                u = 1.0 - t
                x = u*u*u*p0[0] + 3*u*u*t*p1[0] + 3*u*t*t*p2[0] + t*t*t*p3[0]
                y = u*u*u*p0[1] + 3*u*u*t*p1[1] + 3*u*t*t*p2[1] + t*t*t*p3[1]
                points.append((x, y))
        return points

    def _resample(self, points, step, max_length):
        """Walk along the polyline and drop a point every `step` meters."""
        result = []
        travelled = 0.0
        next_mark = step

        for k in range(len(points) - 1):
            a = points[k]
            b = points[k + 1]
            seg = self._dist(a, b)
            if seg < 1e-9:
                continue
            while next_mark <= travelled + seg and next_mark <= max_length + 1e-9:
                t = (next_mark - travelled) / seg
                result.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
                next_mark += step
            travelled += seg

        return result

    def _polyline_length(self, points):
        total = 0.0
        for k in range(len(points) - 1):
            total += self._dist(points[k], points[k + 1])
        return total

    @staticmethod
    def _dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _normalize(v):
        n = math.hypot(v[0], v[1])
        if n < 1e-9:
            return (1.0, 0.0)
        return (v[0] / n, v[1] / n)
