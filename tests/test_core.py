import tempfile,unittest
from pathlib import Path
from safepathshield.core import DEMO,analyze,render_svg,safety_filter
class SafePathShieldTests(unittest.TestCase):
    def test_shield_prevents_nominal_collision(self):
        result=analyze(DEMO);self.assertGreater(result["nominal"]["collisions"],0);self.assertEqual(result["shielded"]["collisions"],0);self.assertTrue(result["passed"])
    def test_shield_still_reaches_goal(self): self.assertLess(analyze(DEMO)["shielded"]["goal_error"],.15)
    def test_filter_satisfies_barrier(self):
        velocity,changed,h=safety_filter((-1.0,0),(1,0),(0,0),.9);self.assertTrue(changed);self.assertGreaterEqual(-2*velocity[0]+4*h,-1e-9)
    def test_renders_svg_artifact(self):
        with tempfile.TemporaryDirectory() as d:
            p=render_svg(analyze(DEMO),Path(d)/"path.svg");self.assertIn("<polyline",p.read_text())
if __name__=="__main__":unittest.main()
