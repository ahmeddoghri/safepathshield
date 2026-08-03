from __future__ import annotations

from .adversarial import HOLDOUT_SEEDS, TUNING_SEEDS, random_scenario
from .core import analyze
from .core_v2 import analyze_v2


def summarize(seeds: list[int]) -> dict:
    original_collision_scenarios = 0
    original_min_clearance = float("inf")
    v2_collision_scenarios = 0
    v2_min_clearance = float("inf")
    for seed in seeds:
        scenario = random_scenario(seed)
        original = analyze(scenario)["shielded"]
        fixed = analyze_v2(scenario)["shielded"]
        original_collision_scenarios += int(original["collisions"] > 0)
        v2_collision_scenarios += int(fixed["collisions"] > 0)
        original_min_clearance = min(original_min_clearance, original["minimum_clearance"])
        v2_min_clearance = min(v2_min_clearance, fixed["minimum_clearance"])
    return {
        "scenario_count": len(seeds),
        "original_collision_scenarios": original_collision_scenarios,
        "v2_collision_scenarios": v2_collision_scenarios,
        "original_worst_min_clearance": round(original_min_clearance, 6),
        "v2_worst_min_clearance": round(v2_min_clearance, 6),
    }


if __name__ == "__main__":
    import json

    print(json.dumps({"tuning": summarize(TUNING_SEEDS), "holdout": summarize(HOLDOUT_SEEDS)}, indent=2))
