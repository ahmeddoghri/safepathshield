from __future__ import annotations

import copy
import random

from .core import SCENARIO

TUNING_SEEDS = list(range(1, 41))
HOLDOUT_SEEDS = list(range(1000, 1020))


def random_scenario(seed: int) -> dict:
    rng = random.Random(seed)
    scenario = copy.deepcopy(SCENARIO)
    scenario["dt"] = rng.uniform(0.04, 0.6)
    scenario["start"] = [rng.uniform(-2.6, -1.0), rng.uniform(-0.6, 0.6)]
    scenario["obstacle_radius"] = rng.uniform(0.4, 0.9)
    scenario["robot_radius"] = rng.uniform(0.1, 0.25)
    scenario["steps"] = 600
    return scenario
