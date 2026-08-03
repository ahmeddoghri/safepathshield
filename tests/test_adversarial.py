import copy
import unittest

from safepathshield.core import DEMO, SCENARIO, analyze
from safepathshield.core_v2 import analyze_v2
from safepathshield.adversarial import HOLDOUT_SEEDS, TUNING_SEEDS, random_scenario
from safepathshield.eval_v2 import summarize


class AdversarialShieldTest(unittest.TestCase):
    """core.py's safety_filter enforces the CONTINUOUS-time control-barrier
    condition, then the caller applies the resulting velocity for a
    finite-size discrete step. That linearization ignores the |v*dt|^2
    curvature term a real Euler step introduces, so for large-enough dt
    the "safe" velocity can still land the robot inside the obstacle."""

    def test_large_dt_produces_a_real_collision_in_original(self):
        scenario = copy.deepcopy(SCENARIO)
        scenario["dt"] = 0.3
        scenario["steps"] = 500
        result = analyze(scenario)["shielded"]
        self.assertGreater(result["collisions"], 0)
        self.assertLess(result["minimum_clearance"], 0)

    def test_same_scenario_is_safe_under_v2(self):
        scenario = copy.deepcopy(SCENARIO)
        scenario["dt"] = 0.3
        scenario["steps"] = 500
        result = analyze_v2(scenario)["shielded"]
        self.assertEqual(result["collisions"], 0)
        self.assertGreaterEqual(result["minimum_clearance"], -1e-6)

    def test_v2_generalizes_across_tuning_and_holdout_sweeps(self):
        tuning = summarize(TUNING_SEEDS)
        holdout = summarize(HOLDOUT_SEEDS)
        # Original genuinely fails on a large fraction of realistic
        # (randomized dt/start/obstacle-size) scenarios on both sweeps.
        self.assertGreater(tuning["original_collision_scenarios"], 0)
        self.assertGreater(holdout["original_collision_scenarios"], 0)
        # v2 has zero collisions across both, evaluated once on holdout.
        self.assertEqual(tuning["v2_collision_scenarios"], 0)
        self.assertEqual(holdout["v2_collision_scenarios"], 0)

    def test_demo_output_unaffected(self):
        # core.py is untouched; the published demo numbers must still
        # reproduce exactly.
        result = analyze(DEMO)
        self.assertEqual(result["nominal"]["collisions"], 39)
        self.assertEqual(result["shielded"]["collisions"], 0)
        self.assertEqual(result["shielded"]["interventions"], 75)
        self.assertEqual(result["shielded"]["minimum_clearance"], 0.00137)
        self.assertEqual(result["shielded"]["goal_error"], 0.05978)
        self.assertTrue(result["passed"])

    def test_v2_still_reaches_goal_on_demo_scenario(self):
        result = analyze_v2(DEMO)
        self.assertEqual(result["nominal"]["collisions"], 39)
        self.assertEqual(result["shielded"]["collisions"], 0)
        self.assertLess(result["shielded"]["goal_error"], 0.15)


if __name__ == "__main__":
    unittest.main()
