from __future__ import annotations

from typing import Dict, List, Tuple

from src.models import CarPose, Cone


_SCENARIOS: Dict[str, Tuple[List[Cone], CarPose]] = {
    "1": (
        [],
        CarPose(x=0.0, y=0.0, yaw=0.0),
    ),

    "2": (
        [Cone(x=1.0, y=3.0, color=1), Cone(x=1.0, y=1.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=0.4),
    ),

    "3": (
        [
            Cone(x=1.0, y=3.0, color=1),
            Cone(x=3.0, y=3.0, color=1),
            Cone(x=1.0, y=1.0, color=0),
            Cone(x=3.0, y=1.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.8),
    ),

    "4": (
        [Cone(x=4.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.2),
    ),

    "5": (
        [Cone(x=4.0, y=2.0, color=0), Cone(x=3.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.6),
    ),

    "6": (
        [
            Cone(x=4.0, y=4.0, color=1),
            Cone(x=4.0, y=2.0, color=0),
            Cone(x=3.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=2.0),
    ),

    "7": (
        [
            Cone(x=2.0, y=3.0, color=1),
            Cone(x=4.0, y=3.0, color=1),
            Cone(x=2.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.4),
    ),

    "8": (
        [Cone(x=3.0, y=3.0, color=1), Cone(x=5.0, y=3.0, color=1)],
        CarPose(x=0.0, y=0.0, yaw=1.8),
    ),

    "9": (
        [Cone(x=3.0, y=3.0, color=1)],
        CarPose(x=0.0, y=0.0, yaw=0.2),
    ),

    "10": (
        [Cone(x=5.0, y=3.0, color=1), Cone(x=5.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=0.6),
    ),

    "11": (
        [
            Cone(x=1.0, y=3.0, color=1),
            Cone(x=4.0, y=5.0, color=1),
            Cone(x=1.0, y=2.0, color=0),
            Cone(x=4.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.0),
    ),

    "12": (
        [Cone(x=5.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.4),
    ),

    "13": (
        [Cone(x=5.0, y=3.0, color=0), Cone(x=5.0, y=1.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=0.1),
    ),

    "14": (
        [
            Cone(x=3.0, y=5.0, color=1),
            Cone(x=3.0, y=2.0, color=0),
            Cone(x=5.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=5.2),
    ),

    "15": (
        [
            Cone(x=2.0, y=5.0, color=1),
            Cone(x=3.0, y=4.0, color=1),
            Cone(x=3.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.6),
    ),

    "16": (
        [Cone(x=0.0, y=3.0, color=1), Cone(x=2.0, y=5.0, color=1)],
        CarPose(x=0.0, y=0.0, yaw=0.2),
    ),

    "17": (
        [Cone(x=3.0, y=5.0, color=1)],
        CarPose(x=0.0, y=0.0, yaw=0.6),
    ),

    "18": (
        [Cone(x=2.0, y=4.0, color=1), Cone(x=4.0, y=3.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.0),
    ),

    "19": (
        [
            Cone(x=2.0, y=3.0, color=1),
            Cone(x=5.0, y=3.0, color=1),
            Cone(x=2.0, y=0.0, color=0),
            Cone(x=5.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.4),
    ),

    "20": (
        [Cone(x=0.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.8),
    ),

    # ------------------------------------------------------------------
    # Part 2: three cones on one side of the track
    # ------------------------------------------------------------------

    # 21: left turn, 3 blue cones on the inside of the curve
    "21": (
        [
            Cone(x=1.25, y=1.83, color=1),
            Cone(x=2.17, y=2.75, color=1),
            Cone(x=2.5, y=4.0, color=1),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.0),
    ),

    # 22: right turn, 3 yellow cones on the inside of the curve
    "22": (
        [
            Cone(x=1.83, y=1.25, color=0),
            Cone(x=2.75, y=2.17, color=0),
            Cone(x=4.0, y=2.5, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.5708),
    ),

    # 23: straight, 3 blue cones on a line (circle fit not possible -> straight)
    "23": (
        [
            Cone(x=1.0, y=1.5, color=1),
            Cone(x=3.0, y=1.5, color=1),
            Cone(x=5.0, y=1.5, color=1),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.0),
    ),

    # 24: left turn, 3 blue cones + 1 yellow cone on the outside
    "24": (
        [
            Cone(x=1.25, y=1.83, color=1),
            Cone(x=2.17, y=2.75, color=1),
            Cone(x=2.5, y=4.0, color=1),
            Cone(x=2.75, y=-0.76, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.3),
    ),
}


def get_scenario_names() -> List[str]:
    return sorted(_SCENARIOS.keys(), key=lambda s: int(s))


def make_scenario(name: str) -> Tuple[List[Cone], CarPose]:
    if name not in _SCENARIOS:
        valid = ", ".join(get_scenario_names())
        raise ValueError(f"Unknown scenario '{name}'. Valid options: {valid}")
    cones, car = _SCENARIOS[name]
    return list(cones), CarPose(x=car.x, y=car.y, yaw=car.yaw)
