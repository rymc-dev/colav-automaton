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

## Done in v1.0.4

- **Cruise/Transition_to_LOS merged into a single `Transit` state.** They
  existed purely to gate when LOS heading correction was "allowed" to run
  (via `heading_not_within_tolerance_guard`/`heading_within_tolerance_guard`
  and their deadband config) - since LOS control (`flow_los_heading`)
  safely subsumes open-loop heading-hold, the split bought no behavior for
  two extra guards and transitions. The automaton is now 3 states / 5
  transitions (`e1`-`e5`), down from 4 states / 10 transitions.

- **COLREGs give-way/stand-on role logic, added.** `colav_automaton.classification`
  classifies each nearby ship's encounter geometry (Rule 13 overtaking,
  Rule 14 head-on, Rule 15 crossing give-way/stand-on) and weights it by
  Rule 18 vessel-type right-of-way (from an AIS-style `tag`: sailing,
  fishing, tanker, etc.), producing a `Maneuver` (side + urgency). For
  multiple ships, `classify_unsafe_set_obstacles` re-applies `riskenv`'s
  own I1/I2/I3 "of interest" filtering before aggregating, so the
  maneuver_bias stays scoped to the same obstacles the unsafe_region hull
  was built from - a distant, irrelevant ship can't dominate the decision
  just because its encounter geometry looks dangerous in isolation.
  `resets.generate_new_virtual_waypoint` uses the aggregated result
  (`maneuver_bias`) to override its own geometric "easiest side" default
  when classification data is available. Not full rule-category coverage -
  see below for what's still missing.

- **`unsafe_region` risk-envelope generation now uses `riskenv` for real**
  (a real `pyproject.toml` dependency, not an unfinished stub). See
  `scripts/generate_unsafe_set.py` for the working ships-in,
  unsafe-region-and-maneuver-bias-out example.

## Deliberately out of scope for v1.0.4

- **Not a certified COLREGs rule-engine.** `colav_automaton.classification`
  covers Rules 13/14/15/17/18 geometrically and by vessel type, but not
  restricted visibility (Rule 19), sound/light signal requirements, or
  simultaneous multi-vessel priority arbitration beyond
  `aggregate_maneuvers`'s highest-urgency-wins rule. Worth a second look
  once this is validated against real multi-vessel traffic.

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
