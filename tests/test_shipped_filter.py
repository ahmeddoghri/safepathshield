"""The tool must ship the filter that holds, not the one that collides."""
import unittest

from safepathshield.adversarial import HOLDOUT_SEEDS, TUNING_SEEDS, random_scenario
from safepathshield.core import DEMO
from safepathshield.core import analyze as analyze_legacy
from safepathshield.core_v2 import analyze_v2


class ShippedFilterTest(unittest.TestCase):
    def test_the_cli_and_api_default_to_the_discrete_time_filter(self):
        from safepathshield import cli, server

        self.assertIs(cli.analyze, analyze_v2)
        self.assertIs(server.analyze, analyze_v2)

    def test_the_default_filter_never_collides_on_any_scenario(self):
        for seed in list(TUNING_SEEDS) + list(HOLDOUT_SEEDS):
            scenario = random_scenario(seed)
            with self.subTest(seed=seed):
                result = analyze_v2(scenario)
                self.assertEqual(result["shielded"]["collisions"], 0, result["shielded"])
                self.assertGreaterEqual(result["shielded"]["minimum_clearance"], 0.0)

    def test_the_legacy_filter_still_reproduces_the_published_demo(self):
        shielded = analyze_legacy(DEMO)["shielded"]
        self.assertEqual(shielded["collisions"], 0)
        self.assertEqual(shielded["interventions"], 75)
        self.assertEqual(shielded["minimum_clearance"], 0.00137)
        self.assertEqual(shielded["goal_error"], 0.05978)

    def test_the_legacy_filter_does_collide_somewhere(self):
        # The reason the default changed: it is not safe at every control rate.
        collided = any(
            analyze_legacy(random_scenario(seed))["shielded"]["collisions"] > 0
            for seed in TUNING_SEEDS
        )
        self.assertTrue(collided)


if __name__ == "__main__":
    unittest.main()
