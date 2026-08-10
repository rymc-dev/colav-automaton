# Guards / Resets / Invariants / Dynamics - Shared Context Data Table

Every guard, reset, invariant, and continuous-dynamics function in
`colav_automaton` (see `src/colav_automaton/guards`, `resets`, `invariants`,
`dynamics`) takes a single argument: `ctx`, a `hybrid_automaton.RuntimeContext`.
This replaces the older `(x, aux_x, cfg, u, dt)` multi-argument convention
used pre-v1.0.0 - this doc describes the current contract.

| Field | Type | Description | Example |
|---|---|---|---|
| `ctx.continuous_state` | `ContinuousState` | The agent's continuous state, integrated by the automaton's continuous dynamics each step. `.latest()` returns the current state vector. In this package the state is `[x_pos, y_pos, theta, velocity, yaw_rate]` (theta in radians). | `ctx.continuous_state.latest()` → `[10.5, 20.3, 1.5, 2.0, 0.0]` |
| `ctx.auxiliary_states` | `Dict[str, AuxiliaryState]` | Externally/reset-updated continuous state that isn't integrated by the automaton. Each `AuxiliaryState` has `.latest()` (most recent value) and `.aux_buffer` (a bounded history deque - each `.add()` call pushes to the front, each `.pop()` removes the front). In this package: `'waypoints'` (stack of `[wx, wy]` points, current target is `.latest()`) and `'unsafe_region'` (unsafe polygon vertices). | `ctx.auxiliary_states['waypoints'].latest()` → `[10.0, 20.0]` |
| `ctx.configuration` | `Dict` | Static per-automaton configuration set at construction time (see `ColavAutomaton()` in `automaton.py`), e.g. tolerances, radii, offset distances. | `ctx.configuration['heading_tolerance_on']` → `0.2` |
| `ctx.control_inputs` | `Dict[str, ControlInput]` | Static/piecewise-static control inputs. Not currently used by any guard/reset/invariant in this package. | - |

## Guard module contract

A guard is a function `ctx -> bool`. If it returns `True`, the transition it's
attached to fires. See `docs/guards/*.mmd` and `*_datatable.txt` for the
per-guard flowcharts and truth tables.

## Invariant module contract

An invariant is a function `ctx -> bool`. If it returns `False` while no
guard on the current state fires, the automaton has no active
transition and is considered stuck. `failing_invariant` (always `False`) is
used on `Fallback` and `Waypoint_Reached` specifically to force them to
always have an active outgoing guard - see `src/colav_automaton/invariants/invariants.py`.

## Reset module contract

A reset is a function `ctx -> ctx`, applied when a transition fires, before
entering the destination state. See `src/colav_automaton/resets/resets.py`.

## Continuous dynamics contract

A continuous dynamics function is `ctx -> np.ndarray` (the state derivative,
integrated against `ctx.continuous_state` each step). See
`src/colav_automaton/dynamics/dynamics.py`.
