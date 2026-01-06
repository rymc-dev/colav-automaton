
from typing import Dict, List
import math
import numpy as np
from hybrid_automaton import Automaton
from shapely.geometry import Polygon, Point

def constant_heading_dynamics(
    x: Automaton.Runtime.ContinousState,
    aux_x: Dict[str, Automaton.Runtime.AuxiliaryState],
    u: Dict[str, Automaton.Runtime.ControlInput],
    cfg: Dict,
    clk: Automaton.Runtime.Clock
):
    """
    State x: [px, py, theta, v, yaw_rate]
    """

    px, py, theta, v, yaw_rate = x.get_continous_state()

    v_des = cfg.get("constant_velocity", 2.0)  # target speed
    k_v   = cfg.get("k_v", 1.0)                # velocity gain

    # kinematics
    px_dot = v * math.cos(theta)
    py_dot = v * math.sin(theta)
    theta_dot = 0.0

    # drive v -> v_des
    v_dot = k_v * (v_des - v)

    yaw_rate_dot = 0.0

    return np.array([px_dot, py_dot, theta_dot, v_dot, yaw_rate_dot], dtype=float)

# def constant_heading_dynamics(
#     x: Automaton.Runtime.ContinousState,
#     aux_x: Dict[str, Automaton.Runtime.AuxiliaryState],
#     u: Dict[str, Automaton.Runtime.ControlInput],
#     cfg: Dict,
#     clk: Automaton.Runtime.Clock
# ):
#     """
#     State x: [px, py, theta, v, yaw_rate]
#     Slowly converges heading toward the waypoint during cruise
#     """
#     px, py, theta, v, yaw_rate = x.get_continous_state()
    
#     v_des = cfg.get("constant_velocity", 2.0)  # target speed
#     k_v   = cfg.get("k_v", 1.0)                # velocity gain
    
#     # Get waypoint for heading reference
#     x_wp, y_wp = aux_x["waypoints"].state[0]
    
#     # Compute desired heading toward waypoint
#     desired_heading = math.atan2(y_wp - py, x_wp - px)
    
#     # Heading error in [-pi, pi]
#     e_theta = (desired_heading - theta + math.pi) % (2 * math.pi) - math.pi
    
#     # Very slow heading convergence gain (much smaller than LOS)
#     k_theta_cruise = cfg.get("k_theta_cruise", 0.1)  # slow convergence
    
#     # kinematics
#     px_dot = v * math.cos(theta)
#     py_dot = v * math.sin(theta)
    
#     # Slow heading convergence
#     theta_dot = k_theta_cruise * e_theta
    
#     # drive v -> v_des
#     v_dot = k_v * (v_des - v)
    
#     yaw_rate_dot = 0.0
    
#     return np.array([px_dot, py_dot, theta_dot, v_dot, yaw_rate_dot], dtype=float)

def flow_los_heading(
    x: Automaton.Runtime.ContinousState,
    aux_x: Dict[str, Automaton.Runtime.AuxiliaryState],
    u: Dict[str, Automaton.Runtime.ControlInput],
    cfg: Dict,
    clk: Automaton.Runtime.Clock
):
    """
    LOS heading control with velocity driven toward constant_velocity.
    State x: [px, py, theta, v, yaw_rate]
    """

    px, py, theta, v, yaw_rate = x.get_continous_state()

    # Waypoint
    x_wp, y_wp = aux_x["waypoints"].state[0]

    # LOS desired heading
    desired_heading = math.atan2(y_wp - py, x_wp - px)

    # Heading error in [-pi, pi]
    e_theta = (desired_heading - theta + math.pi) % (2 * math.pi) - math.pi

    # Heading gain
    k_theta = cfg.get("k_theta", 1.0)

    # Heading dynamics
    theta_dot = k_theta * e_theta

    # Translational dynamics using current speed
    px_dot = v * math.cos(theta)
    py_dot = v * math.sin(theta)

    # Velocity control toward target
    v_des = cfg.get("constant_velocity", 2.0)
    k_v   = cfg.get("k_v", 1.0)
    v_dot = k_v * (v_des - v)

    yaw_rate_dot = 0.0

    return np.array([px_dot, py_dot, theta_dot, v_dot, yaw_rate_dot], dtype=float)

# Controllers from paper
class PrescribedTimeController:
    """
    Prescribed-time heading controller from ship navigation.
    Guarantees heading convergence to LOS in prescribed time tp.
    """
    def __init__(self, a: float, v: float, eta: float, tp: float):
        """
        Args:
            a: System parameter
            v: Ship velocity (m/s)
            eta: Controller gain (n > 1)
            tp: Prescribed time (seconds)
        """
        self.a = a
        self.v = v
        self.eta = eta
        self.tp = tp
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-π, π]"""
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def compute_control(self, t: float, x: float, y: float, psi: float, xw: float, yw: float) -> float:
        """
        Prescribed-time control law
        
        u = (1/a)dot{psi}_dg + psi - η(psi - psi_dg)/(a(tp - t))  for t < tp
        u = (1/a)dot{psi}_dg + psi                                for t >= tp
        """
        # Desired heading to waypoint
        psi_dg = np.arctan2(yw - y, xw - x)
        
        # Heading derivative
        dx = xw - x
        dy = yw - y
        d_squared = dx**2 + dy**2
        
        if d_squared < 1e-6:
            psi_dg_dot = 0.0
        else:
            psi_dg_dot = (-self.v * dx * np.sin(psi) + self.v * dy * np.cos(psi)) / d_squared
        
        e = self.normalize_angle(psi - psi_dg)
        
        if t < self.tp:
            time_varying_term = self.eta * e / (self.a * (self.tp - t + 1e-6))
            u = (1/self.a) * psi_dg_dot + psi - time_varying_term
        else:
            u = (1/self.a) * psi_dg_dot + psi
        
        return u
    
    def compute_dynamics(self, t: float, x: float, y: float, psi: float, xw: float, yw: float) -> np.ndarray:
        """
        Ship dynamics with prescribed-time control
        
        dot{x} = v cos(psi)
        dot{y} = v sin(psi)
        dot{psi} = -a*psi + a*u
        
        Returns: np.array([dx/dt, dy/dt, dpsi/dt])
        """
        u = self.compute_control(t, x, y, psi, xw, yw)
        
        dx_dt = self.v * np.cos(psi) # dot(x)
        dy_dt = self.v * np.sin(psi) # dot{y}
        dpsi_dt = -self.a * psi + self.a * u # dot{psi}
        
        return np.array([dx_dt, dy_dt, dpsi_dt])


class CollisionAvoidanceController:
    """
    COLREGs-compliant collision avoidance controller.

    - Creates unsafe set
    - Computes virtual waypoint, V1 of unsafe set
    - Used prescribed_time controller to navigate to V1

    """
    def __init__(self, a: float, v: float, eta: float, tp: float, Cs: float):
        """
        Args:
            a: System parameter
            v: Ship velocity
            eta: Controller gain
            tp: Prescribed time (seconds)
            Cs: Safe distance from obstacle (m)
        """
        self.a = a
        self.v = v
        self.eta = eta
        self.tp = tp
        self.Cs = Cs
        self.virtual_waypoint = None
        self.last_control = 0.0
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle [-π, π]"""
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def create_unsafe_set_polygon(self, ox: float, oy: float) -> Polygon:
        """
        Create unsafe set B∞(po, Cs) as a Shapely Polygon.

        Returns:
            Polygon: Square unsafe set centered at obstacle
        """
        return Polygon([
            (ox - self.Cs, oy - self.Cs),  # V1 - bottom left
            (ox + self.Cs, oy - self.Cs),  # V2 - bottom right
            (ox + self.Cs, oy + self.Cs),  # V3 - top right
            (ox - self.Cs, oy + self.Cs),  # V4 - top left
        ])

    def compute_V1(self, pos_x: float, pos_y: float, psi: float, ox: float, oy: float) -> tuple:
        """
        Compute virtual waypoint V1 (starboard vertex of unsafe set) using Shapely.

        Selects from V1, V2 (starboard vertices) based on:
        - Must be ahead (within ±π/2 of heading)
        - Prefer smallest relative angle

        Returns: (v1_x, v1_y)
        """
        # Create unsafe set polygon
        unsafe_set = self.create_unsafe_set_polygon(ox, oy)

        # Get all vertices (coordinates)
        vertices = list(unsafe_set.exterior.coords)[:-1]  # Exclude duplicate last point

        # For COLREGs, prefer starboard maneuver (V1, V2 are bottom vertices)
        starboard_vertices = vertices[:2]  # V1, V2

        best_vertex = starboard_vertices[0]
        best_score = -np.inf

        for vx, vy in starboard_vertices:
            angle_to_vertex = np.arctan2(vy - pos_y, vx - pos_x)
            relative_angle = self.normalize_angle(angle_to_vertex - psi)

            # Check if vertex is ahead (L2: within ±π/2)
            if -np.pi/2 < relative_angle < np.pi/2:
                dist = np.sqrt((vx - pos_x)**2 + (vy - pos_y)**2)
                score = -relative_angle - 0.01 * dist  # Prefer negative angles (starboard)

                if score > best_score:
                    best_score = score
                    best_vertex = (vx, vy)

        return best_vertex
    
    def set_virtual_waypoint(self, pos_x: float, pos_y: float, psi: float, ox: float, oy: float):
        """Compute and store virtual waypoint V1"""
        self.virtual_waypoint = self.compute_V1(pos_x, pos_y, psi, ox, oy)
    
    def compute_dynamics(self, t: float, x: float, y: float, psi: float) -> np.ndarray:
        """
        Compute state derivatives for S2 mode (collision avoidance).
        
        Returns: np.array([dx/dt, dy/dt, dpsi/dt])
        """
        if self.virtual_waypoint is None:
            raise ValueError("Virtual waypoint not set")
        
        vx, vy = self.virtual_waypoint
        psi_dg = np.arctan2(vy - y, vx - x)
        
        dx = vx - x
        dy = vy - y
        d_squared = dx**2 + dy**2
        
        if d_squared < 1e-6:
            psi_dg_dot = 0.0
        else:
            psi_dg_dot = (-self.v * dx * np.sin(psi) + self.v * dy * np.cos(psi)) / d_squared
        
        e = self.normalize_angle(psi - psi_dg)
        
        if t < self.tp:
            time_varying_term = self.eta * e / (self.a * (self.tp - t + 1e-6))
            u = (1/self.a) * psi_dg_dot + psi - time_varying_term
        else:
            u = (1/self.a) * psi_dg_dot + psi
        
        self.last_control = u
        
        dx_dt = self.v * np.cos(psi)
        dy_dt = self.v * np.sin(psi)
        dpsi_dt = -self.a * psi + self.a * u
        
        return np.array([dx_dt, dy_dt, dpsi_dt])
    
    def compute_constant_dynamics(self, x: float, y: float, psi: float) -> np.ndarray:
        """
        Compute state derivatives for S3 mode (constant control).
        Uses last_control as uc.
        
        Returns: np.array([dx/dt, dy/dt, dpsi/dt])
        """
        uc = self.last_control
        
        dx_dt = self.v * np.cos(psi)
        dy_dt = self.v * np.sin(psi)
        dpsi_dt = -self.a * psi + self.a * uc
        
        return np.array([dx_dt, dy_dt, dpsi_dt])
    
    def reset(self):
        """Reset for new avoidance maneuver"""
        self.virtual_waypoint = None
        self.last_control = 0.0

# Dynamics from paper

def S1_waypoint_reaching_dynamics(
    x: Automaton.Runtime.ContinousState,
    aux_x: Dict[str, Automaton.Runtime.AuxiliaryState],
    u: Dict[str, Automaton.Runtime.ControlInput],
    cfg: Dict,
    clk: Automaton.Runtime.Clock
):
    """
    S1: Waypoint reaching - navigate to waypoint using prescribed-time control.
    
    State x: [x, y, psi]
        - x, y: position (m)
        - psi: heading (rad)
    
    Uses prescribed-time controller to guarantee heading convergence to LOS.
    """
    state = x.get_continous_state()
    t = clk.get_time_elapsed_since_last_transition()

    # Get or create controller (stored in cfg to persist across calls)
    if 'pt_controller' not in cfg:
        cfg['pt_controller'] = PrescribedTimeController(
            cfg['a'], cfg['v'], cfg['eta'], cfg['tp']
        )

    return cfg['pt_controller'].compute_dynamics(
        t, state[0], state[1], state[2],
        cfg['waypoint_x'], cfg['waypoint_y']
    )


def S2_collision_avoidance_dynamics(
    x: Automaton.Runtime.ContinousState,
    aux_x: Dict[str, Automaton.Runtime.AuxiliaryState],
    u: Dict[str, Automaton.Runtime.ControlInput],
    cfg: Dict,
    clk: Automaton.Runtime.Clock
):
    """
    S2: Collision avoidance - navigate to V1 using prescribed-time control.
    
    State x: [x, y, psi]
        - x, y: position (m)
        - psi: heading (rad)
    
    Computes virtual waypoint V1 (starboard vertex of unsafe set) and
    applies prescribed-time control to reach it.
    """
    state = x.get_continous_state()
    t = clk.get_time_elapsed_since_last_transition()

    # Get or create controller
    if 'ca_controller' not in cfg:
        cfg['ca_controller'] = CollisionAvoidanceController(
            cfg['a'], cfg['v'], cfg['eta'], cfg['tp'], cfg['Cs']
        )

    # Set virtual waypoint V1
    cfg['ca_controller'].set_virtual_waypoint(
        state[0], state[1], state[2],
        cfg['obstacle_x'], cfg['obstacle_y']
    )

    return cfg['ca_controller'].compute_dynamics(t, state[0], state[1], state[2])


def S3_constant_control_dynamics(
    x: Automaton.Runtime.ContinousState,
    aux_x: Dict[str, Automaton.Runtime.AuxiliaryState],
    u: Dict[str, Automaton.Runtime.ControlInput],
    cfg: Dict,
    clk: Automaton.Runtime.Clock
):
    """
    S3: Constant control - hold last control value from S2.
    
    State x: [x, y, psi]
        - x, y: position (m)
        - psi: heading (rad)
    
    Maintains the control input that was active when transitioning from S2.
    """
    state = x.get_continous_state()

    return cfg['ca_controller'].compute_constant_dynamics(
        state[0], state[1], state[2]
    )
