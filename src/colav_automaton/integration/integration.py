import math
from hybrid_automaton._runtime import _Runtime
import numpy as np

RuntimeContext = _Runtime.Context


def _prescribed_time_control(
    t: float, 
    px: float, py: float, theta: float, 
    psi_desired: float,
    *, 
    a: float, v: float, eta: float, tp: float
): 
    """
    u = (1/a)·ψ̇_dg + ψ_dg − η·e / (a·(tp−t))   t < tp
    u = (1/a)·ψ̇_dg + ψ_dg                         t ≥ tp

    Note: ψ_dg is passed directly from dynamics output so no
    recomputation of the LOS angle is needed here.
    """
    # ψ̇_dg approximated as 0 — it was already evaluated by the
    # dynamics function one step ago; you can extend this to a
    # finite-difference buffer if higher fidelity is needed.
    psi_dg_dot = 0.0

    e = math.atan2(math.sin(theta - psi_desired), math.cos(theta - psi_desired))

    if t < tp:
        u = (1.0 / a) * psi_dg_dot + psi_desired - eta * e / (a * (tp - t + 1e-9))
    else:
        u = (1.0 / a) * psi_dg_dot + psi_desired

    return u

class PrescribedTimeIntegrator:
    """
    Drop-in replacement for Euler integration.

    Instead of x += xdot·dt it:
        1. Unpacks the desired heading from the dynamics output
        2. Runs the prescribed-time control law
        3. Evaluates the plant model to get true xdot
        4. Steps Euler with that real xdot

    This mirrors the real system where the controller runs in a fast
    parallel loop consuming the automaton's desired-heading commands.

    Usage
    -----
    integrator = PrescribedTimeIntegrator()
    # wire into runtime before activation:
    runtime._ctx.continuous_state._integration_function = integrator.make(ctx)
    # or pass via the factory below
    """

    def make(self, ctx: RuntimeContext):
        """
        Return a closure bound to `ctx` so the integrator can read the
        clock and configuration without an extra argument.
        """
        def _integrate(x: np.ndarray, desired_heading: np.ndarray, dt: float) -> np.ndarray:
            px, py, theta, v, yaw_rate = x

            # ── controller parameters from configuration ──────────────────
            a     = ctx.configuration.get("a",                 1.0)
            eta   = ctx.configuration.get("eta",               1.5)
            tp    = ctx.configuration.get("tp",                5.0)
            v_des = ctx.configuration.get("constant_velocity", 2.0)
            k_v   = ctx.configuration.get("k_v",               1.0)

            # ── time since last discrete transition ───────────────────────
            t = ctx.clock.get_time_elapsed_since_transition()

            # ── prescribed-time control law ───────────────────────────────
            psi_des = float(desired_heading[0])
            u = _prescribed_time_control(
                t, px, py, theta, psi_des,
                a=a, v=v, eta=eta, tp=tp,
            )

            # ── plant / vessel kinematics ─────────────────────────────────
            px_dot       = v * math.cos(theta)
            py_dot       = v * math.sin(theta)
            theta_dot    = a * (u - theta)      # first-order heading plant
            v_dot        = k_v * (v_des - v)
            yaw_rate_dot = 0.0

            xdot_true = np.array(
                [px_dot, py_dot, theta_dot, v_dot, yaw_rate_dot], dtype=float
            )

            # ── Euler step ────────────────────────────────────────────────
            return x + xdot_true * dt

        return _integrate