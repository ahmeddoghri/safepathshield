from __future__ import annotations

import math

from .core import SCENARIO, nominal_velocity, _norm


def safety_filter_v2(
    position: tuple[float, float],
    velocity: tuple[float, float],
    obstacle: tuple[float, float],
    clearance: float,
    dt: float,
    alpha: float = 4.0,
) -> tuple[tuple[float, float], bool, float]:
    """Discrete-time-exact control-barrier projection.

    core.safety_filter enforces the CONTINUOUS-time barrier condition
    grad(h).v + alpha*h >= 0 at the current position, then the caller
    applies the resulting velocity for a finite step of size dt. That
    linearization ignores the |v*dt|^2 curvature term the discrete Euler
    step actually introduces, so for large-enough dt (or fast-enough
    speeds) the "safe" velocity can still land the next position inside
    the obstacle -- verified directly: at dt=0.3 the shielded controller
    in core.py produces a real collision (minimum_clearance = -0.015).

    This projection instead enforces the exact discrete-time barrier
    condition h(x + v'*dt) >= (1 - alpha*dt) * h(x) for the true next
    position (not a linear approximation of it), solving the resulting
    quadratic in the correction magnitude directly. alpha*dt is capped
    at 1.0 -- required for the discrete-time guarantee to hold, and
    always at least as conservative as an uncapped value.
    """
    delta = (position[0] - obstacle[0], position[1] - obstacle[1])
    h = delta[0] ** 2 + delta[1] ** 2 - clearance ** 2
    gradient = (2 * delta[0], 2 * delta[1])
    decay = min(max(alpha * dt, 0.0), 1.0)
    # A small positive margin keeps the projected next-step h strictly above
    # zero rather than landing exactly on the boundary, where floating-point
    # rounding could otherwise tip an individual step negative.
    margin = 1e-6
    w = (delta[0] + velocity[0] * dt, delta[1] + velocity[1] * dt)
    g_squared = gradient[0] ** 2 + gradient[1] ** 2
    a_coeff = dt * dt * g_squared
    b_coeff = 2 * dt * (w[0] * gradient[0] + w[1] * gradient[1])
    c_coeff = w[0] ** 2 + w[1] ** 2 - clearance ** 2 - (1 - decay) * h - margin
    if c_coeff >= 0 or a_coeff <= 1e-12:
        return velocity, False, h
    discriminant = max(0.0, b_coeff * b_coeff - 4 * a_coeff * c_coeff)
    root = math.sqrt(discriminant)
    candidates = ((-b_coeff + root) / (2 * a_coeff), (-b_coeff - root) / (2 * a_coeff))
    scale = min(candidates, key=abs)
    return (velocity[0] + scale * gradient[0], velocity[1] + scale * gradient[1]), True, h


def simulate_v2(scenario: dict = SCENARIO, shielded: bool = True) -> dict:
    position = tuple(map(float, scenario["start"]))
    goal = tuple(map(float, scenario["goal"]))
    obstacle = tuple(map(float, scenario["obstacle"]))
    clearance = float(scenario["obstacle_radius"]) + float(scenario["robot_radius"])
    dt = float(scenario.get("dt", 0.04))
    steps = int(scenario.get("steps", 220))
    position_path = [list(position)]
    interventions = 0
    minimum = float("inf")
    collisions = 0
    for _ in range(steps):
        velocity = nominal_velocity(position, goal)
        if shielded:
            velocity, intervened, _ = safety_filter_v2(position, velocity, obstacle, clearance, dt)
            interventions += int(intervened)
        position = (position[0] + velocity[0] * dt, position[1] + velocity[1] * dt)
        distance = _norm((position[0] - obstacle[0], position[1] - obstacle[1])) - clearance
        minimum = min(minimum, distance)
        collisions += int(distance < 0)
        position_path.append([round(position[0], 5), round(position[1], 5)])
        if _norm((goal[0] - position[0], goal[1] - position[1])) < 0.06:
            break
    return {
        "shielded": shielded,
        "collisions": collisions,
        "minimum_clearance": round(minimum, 5),
        "interventions": interventions,
        "goal_error": round(_norm((goal[0] - position[0], goal[1] - position[1])), 5),
        "steps": len(position_path) - 1,
        "path": position_path,
    }


def analyze_v2(payload: dict) -> dict:
    nominal = simulate_v2(payload, False)
    shielded = simulate_v2(payload, True)
    return {
        "nominal": nominal,
        "shielded": shielded,
        "prevented_collision_steps": nominal["collisions"] - shielded["collisions"],
        "passed": nominal["collisions"] > 0 and shielded["collisions"] == 0,
        "scope": "Deterministic single-integrator point robot with one circular obstacle; real robots require dynamics, state-estimation uncertainty, and hardware validation.",
    }
