from __future__ import annotations

import json
import math
from pathlib import Path


SCENARIO = {"start": [-2.4, 0.0], "goal": [2.4, 0.0], "obstacle": [0.0, 0.13], "obstacle_radius": 0.72, "robot_radius": 0.18, "dt": 0.04, "steps": 220}


def _norm(vector: tuple[float, float]) -> float:
    return math.hypot(*vector)


def nominal_velocity(position: tuple[float, float], goal: tuple[float, float], max_speed: float = 1.15) -> tuple[float, float]:
    velocity = ((goal[0] - position[0]) * .9, (goal[1] - position[1]) * .9)
    length = _norm(velocity)
    return tuple(value * min(1, max_speed / (length or 1)) for value in velocity)


def safety_filter(position: tuple[float, float], velocity: tuple[float, float], obstacle: tuple[float, float], clearance: float, alpha: float = 4.0) -> tuple[tuple[float, float], bool, float]:
    delta = (position[0] - obstacle[0], position[1] - obstacle[1])
    h = delta[0] ** 2 + delta[1] ** 2 - clearance ** 2
    gradient = (2 * delta[0], 2 * delta[1])
    lhs = gradient[0] * velocity[0] + gradient[1] * velocity[1] + alpha * h
    if lhs >= 0:
        return velocity, False, h
    scale = -lhs / (gradient[0] ** 2 + gradient[1] ** 2 + 1e-12)
    return (velocity[0] + scale * gradient[0], velocity[1] + scale * gradient[1]), True, h


def simulate(scenario: dict = SCENARIO, shielded: bool = True) -> dict:
    position = tuple(map(float, scenario["start"])); goal = tuple(map(float, scenario["goal"])); obstacle = tuple(map(float, scenario["obstacle"]))
    clearance = float(scenario["obstacle_radius"]) + float(scenario["robot_radius"]); dt = float(scenario.get("dt", .04)); steps = int(scenario.get("steps", 220))
    path = [list(position)]; interventions = 0; minimum = float("inf"); collisions = 0
    for _ in range(steps):
        velocity = nominal_velocity(position, goal)
        if shielded:
            velocity, intervened, _ = safety_filter(position, velocity, obstacle, clearance)
            interventions += int(intervened)
        position = (position[0] + velocity[0] * dt, position[1] + velocity[1] * dt)
        distance = _norm((position[0] - obstacle[0], position[1] - obstacle[1])) - clearance
        minimum = min(minimum, distance); collisions += int(distance < 0)
        path.append([round(position[0], 5), round(position[1], 5)])
        if _norm((goal[0] - position[0], goal[1] - position[1])) < .06: break
    return {"shielded": shielded, "collisions": collisions, "minimum_clearance": round(minimum, 5), "interventions": interventions, "goal_error": round(_norm((goal[0] - position[0], goal[1] - position[1])), 5), "steps": len(path)-1, "path": path}


def analyze(payload: dict) -> dict:
    nominal = simulate(payload, False); shielded = simulate(payload, True)
    return {"nominal": nominal, "shielded": shielded, "prevented_collision_steps": nominal["collisions"] - shielded["collisions"], "passed": nominal["collisions"] > 0 and shielded["collisions"] == 0, "scope": "Deterministic single-integrator point robot with one circular obstacle; real robots require dynamics, state-estimation uncertainty, and hardware validation."}


def render_svg(result: dict, output: str | Path) -> Path:
    def point(p): return f"{60 + (p[0]+2.6)/5.2*680:.1f},{330-(p[1]+1.3)/2.6*260:.1f}"
    nominal = " ".join(point(p) for p in result["nominal"]["path"]); safe = " ".join(point(p) for p in result["shielded"]["path"])
    svg=f'''<svg xmlns="http://www.w3.org/2000/svg" width="800" height="400" viewBox="0 0 800 400"><rect width="800" height="400" fill="#080b0c"/><circle cx="400" cy="187" r="90" fill="#191d1c" stroke="#ff7f72" stroke-width="2"/><polyline points="{nominal}" fill="none" stroke="#ff7f72" stroke-width="3"/><polyline points="{safe}" fill="none" stroke="#7de3ba" stroke-width="4"/><circle cx="60" cy="200" r="7" fill="#ddd"/><circle cx="740" cy="200" r="7" fill="#f0c86e"/></svg>'''
    output=Path(output); output.write_text(svg); return output


DEMO = SCENARIO
