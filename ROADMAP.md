# Roadmap / Known Limitations

Tracking notes for known gaps and follow-up work identified during the
v1.0.0 release pass, kept out of that release deliberately, or simply
noted so they don't get lost. See `hybrid-automaton`'s own `ROADMAP.md`
for the same pattern on the framework side.

## Removed pre-1.0.0

- **`integration/` module (`PrescribedTimeIntegrator`) removed entirely.**
  It was orphaned - never wired into `ColavAutomaton()`, its own
  `__init__.py` exported nothing, and it reached into the private
  `hybrid_automaton._runtime` API instead of the public `RuntimeContext`
  export. `tests/test_integration.py` (a non-functional stub with a
  `pytest.parameterize` typo that broke test collection for the whole
  suite) was removed alongside it. If prescribed-time control integration
  is wanted, it needs a real design pass against the current
  `hybrid-automaton>=1.0.0` public API, not a revival of the old code.

- **`is_goal_waypoint_invariant` removed** from
  `src/colav_automaton/invariants/invariants.py`. It was never referenced
  in `ColavAutomaton()`'s actual state graph (both `Fallback` and
  `Waypoint_Reached` use `failing_invariant`), and its
  `len(ctx.auxiliary_states['waypoints'].latest()) > 1` logic assumed
  `.latest()` returns the whole waypoint list rather than a single
  `[x, y]` point - inconsistent with the confirmed contract used
  everywhere else (`dynamics.py`, `guards.py`). Superseded by
  `virtual_waypoints_guard`, which correctly checks `.aux_buffer` history
  length for "are there more waypoints queued."

## Fixed in v1.0.0, worth knowing about

- **`unsafe_conditions_guard` (`e6`) now actually routes to `Fallback`.**
  Previously routed back to `Cruise` despite the code comment saying "TO
  Fallback if unsafe" - the vessel never actually held/backed off when
  unsafe conditions were detected mid-turn. `Fallback` now holds current
  heading/velocity (`constant_heading_dynamics`, no active re-planning)
  until a new `safe_conditions_guard` (`e8`) clears it back to `Cruise`.
  Worth a second look once this is running on real hardware - "hold
  course" is a conservative default, not a tuned COLREGs-aware response.

## Deliberately out of scope for v1.0.0

- **Not full COLREGs rule-category compliance.** This automaton avoids a
  generic unsafe/risk region via waypoint generation - it does not
  implement COLREGs give-way/stand-on role logic (head-on, crossing,
  overtaking encounters). The package is named `colav-automaton` rather
  than `colregs-automaton` specifically because of this; adding real
  COLREGs rule categorization would be a natural v1.1+ direction.

- **No CI lint job.** `test`/`build`/`publish` run in
  `.github/workflows/workflow.yml`; linting was deliberately left out of
  this pass. Worth adding once the API surface has settled post-ROS
  integration.

## Known rough edges (not fixed, tracked here)

- `scripts/commonocean_scenario_tester.py` and
  `scripts/commonocean_scenario_deserialization.py` depend on the
  external [commonocean-scenarios](https://commonocean.cps.cit.tum.de/)
  dataset and the `commonocean`/`commonroad` packages, neither of which
  are repo or package dependencies. Paths are now read from
  `COMMONOCEAN_SCENARIO_PATH` / `COMMONOCEAN_SCENARIO_DIR` env vars (with
  a local-machine fallback) rather than hardcoded, but the scripts still
  won't run without that external setup.
- `scripts/test_scenarios_script.py` imports `colav_unsafe_set`, a
  separate in-progress package not vendored or declared here.
- Git history still contains the ~26MB of run-log CSVs that were
  committed pre-v1.0.0 (`colav-automaton-logs/*.csv`). They're untracked
  going forward (see `.gitignore`), but history wasn't rewritten to
  remove them, to avoid a disruptive force-push on a branch already
  shared via origin.
