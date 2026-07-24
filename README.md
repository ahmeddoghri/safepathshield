# safepathshield

**A control-barrier safety layer for learned robot policies.**

Wrap a nominal motion policy with a minimally invasive quadratic safety projection, compare both trajectories, and export a checkable path artifact.

![safepathshield cover](demo/cover.png)

![safepathshield workbench](demo/dashboard.png)

## What ships

- Nominal goal controller and analytical control-barrier projection
- Side-by-side unsafe and shielded simulations with collision, clearance, intervention, and goal metrics
- Deterministic SVG trajectory export
- CLI, JSON API, animated browser demo, Docker, tests, and CI

## Run it end to end

```bash
python -m venv .venv && source .venv/bin/activate
python -m pip install -e .
safepathshield demo
safepathshield render trajectory.svg
safepathshield serve
```

## Demo result

The nominal controller drives through the obstacle. The barrier projection changes only unsafe velocity commands, preserves non-negative clearance, and still reaches the goal. Tests verify the barrier inequality directly, not just the rendered path.

## Current basis

- [Learning control barrier functions and their application in reinforcement learning](https://proceedings.mlr.press/v270/hu25a.html)

## Scope

The shipped benchmark is a deterministic single-integrator robot with one circular obstacle. Deployment on physical systems additionally requires full dynamics, uncertainty-aware state estimation, actuator limits, and hardware validation.

## Test

```bash
python -m unittest discover -s tests -v
```

MIT licensed.
