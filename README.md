# Path Planning Assignment – Centerline Planner

A simple **centerline path planner** for an FSAI-style cone track.
Given the car pose `(x, y, yaw)` and the detected cones (blue = left, yellow = right),
it returns path points `(x, y)` that keep the car in the middle of the track.

- Path length: between 5 and 10 m (usually ~7–8 m)
- Distance between consecutive points: exactly 0.5 m
- All the logic is in `src/path_planning.py` → `PathPlanning.generatePath()`

---

## How to run

```
python -m venv .venv
.venv\Scripts\activate          # Windows
# source .venv/bin/activate     # macOS / Linux
pip install -r requirements.txt

python -m src.run --scenario 2
```

Scenarios `1`–`20` are the original ones, `21`–`24` were added for Part 2.

---

## Part 1 – The algorithm

The idea comes from **classical centerline planning**: the safest place to drive is the
middle of the track, as far as possible from both boundaries.

### Step 1 – Split the cones by color
Blue cones (`color == 1`) are the left boundary, yellow cones (`color == 0`) are the right boundary.

### Step 2 – Match cones into "gates"
Every blue/yellow pair is scored by its distance. The closest pair is matched first,
then the next closest, and so on (greedy matching, each cone used once).
Pairs farther apart than `MAX_GATE_WIDTH` (6 m) are not matched.

The **midpoint** of a gate is a point on the track centerline.
The **track direction** at that gate is perpendicular to it: the vector yellow → blue
points to the left, so rotating it by −90° gives the forward direction.

### Step 3 – Handle cones without a partner
If a cone has no partner on the other side, we shift it towards the track center by
**half the track width**:
- blue cone → shift to its right
- yellow cone → shift to its left

"Right" and "left" are measured relative to the local track direction, which is taken from:
1. the line to the other cone on the same side (if there is one), else
2. the direction of the nearest gate (if there is one), else
3. the car heading.

The half width is the measured gate width if we saw at least one gate,
otherwise the assumed `TRACK_WIDTH / 2` (1.5 m).

### Step 4 – Order the center points
Center points are sorted by distance from the car.

### Step 5 – Extend the path
After the last center point, the path continues straight in the track direction
until it is at least `MIN_LENGTH` (7 m) long.
If there are **no cones at all**, the path goes straight ahead in the car heading.

### Step 6 – Smooth the path (cubic Bezier)
A real car cannot turn on the spot, so the waypoints are joined with cubic Bezier curves.
Each curve leaves a waypoint in its driving direction and arrives at the next one in its
driving direction. The first waypoint is the car itself with its heading, so **the path
always starts in the direction the car is facing** and then turns smoothly into the track.

For a piece from A to B (handle length `h = 0.4 · |AB|`):

```
P0 = A,  P1 = A + dirA · h,  P2 = B − dirB · h,  P3 = B
```

### Step 7 – Re-sample
We walk along the smooth curve and place a point every 0.5 m, stopping at 10 m.

---

## Part 2 – Three cones on one side

**Approach:** three points that are not on a straight line define exactly one circle.
So with 3 cones on one side we can estimate **how much the track is curving**
(the circle's radius) and **in which direction** (where its center is).

1. Take the side with the most cones (needs at least 3) and use the 3 cones closest to the car.
2. Compute the circle through them (circumcircle).
3. When extending the path after the last center point, instead of going straight we
   follow a circle with **the same center** that passes through the last center point.
   This circle is parallel to the cone line, so it stays in the middle of the track.
4. If the 3 cones are on a straight line (no circle), or the radius is larger than
   `MAX_CURVE_RADIUS` (30 m), we treat the track as straight, exactly like Part 1.

The cones themselves are still handled with steps 1–4 above, so a scenario with
3 cones on one side plus some cones on the other side also works.

**Why this solution?**
- It uses the extra information in the simplest possible way: with 2 cones we only know a
  direction, with 3 cones we also know the curvature.
- It is cheap (closed-form formula, no iterations) and deterministic.
- It does not change the behaviour of the original 20 scenarios at all.

### New test scenarios (`src/scenarios.py`)

| Scenario | Description | Expected result |
|---|---|---|
| 21 | Left turn, 3 blue cones on the inside | Path curves left, staying right of the blue cones |
| 22 | Right turn, 3 yellow cones on the inside | Path curves right, staying left of the yellow cones |
| 23 | 3 blue cones on a straight line | No circle exists → path goes straight |
| 24 | Left turn, 3 blue + 1 yellow cone | Gate midpoint is used, then the path follows the curve |

---

## Assumptions

- **Track width is 3 m** when only one side is visible (FSAI minimum track width).
  If a blue/yellow gate is visible, its real width is used instead.
- **Cone colors are always correct**: blue is always left, yellow always right.
- **The car heading comes from SLAM and is not changed by the planner.** In some scenarios
  (e.g. 14) the car is not facing the cones; the path still starts in the car heading and
  then turns towards the track, because the car cannot turn instantly.
- **Cones describe the track ahead of the car**, so the nearest center point is the next one to drive to.
- With no cones the best guess is to keep driving straight.
- Path length target: 7 m (inside the required 5–10 m range).

## Limitations

- **Ordering by distance from the car** works for these short horizons, but on a hairpin or
  when cones from another part of the track are visible, the order can be wrong.
- **Greedy matching** can pair the wrong cones if the track is very narrow and curved
  (e.g. scenario 15, where the track bends sharply between the cones).
- **Single cone cases are guesses**: with one cone we only know one boundary point, so the
  path depends on the assumed track width and on the car heading.
- **Part 2 assumes constant curvature** between and after the 3 cones. If the track changes
  from a curve to a straight (or the other way), the extension will be wrong after a few meters.
- **Sensitive to noise and ghost cones**: a wrongly detected or wrongly colored cone changes the
  circle and the center points directly. A real system would filter cones over time
  (e.g. using SLAM) or use a cost function to reject bad candidates.
- **Bezier smoothing does not check the car's minimum turning radius**, it only makes the path smooth.
- The tester window shows only −1 to 6 m, so in some scenarios (e.g. 20) part of the path is outside the view.

## Possible improvements

- Delaunay triangulation + a cost function (as discussed in the session) for better robustness.
- Use more than 3 cones with a least-squares circle or spline fit.
- Respect a maximum curvature based on the car's steering limits.
