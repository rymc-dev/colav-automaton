import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__)))

from colav_automaton import ColavAutomaton
from hybrid_automaton import Automaton
from hybrid_automaton_runner import AutomatonRunner
import asyncio
import numpy as np
from hybrid_automaton_evaluation.figure_generator import continuous_states_over_time_fig, auxiliary_states_over_time_fig, automaton_states_over_time, transitions_times_over_time_fig

async def main():
    ha: Automaton = ColavAutomaton(heading_tolerance=0.2, k_theta=1.0, constant_velocity=2.0, acceptance_radius=20, los_distance_threshold=100, longitudinal_offset_distance=50.0, lateral_offset_distance=50.0)
    x0 = np.array([-200.0, -200.0, 0.0, 0.0, 0.0], dtype=float)
    aux_t0 = {
        "waypoints": [np.array([200, 200]), np.array([-200, 200]), np.array([-100, 80]), np.array([200, -200])],
        "unsafe_region": [
            np.array([20, 20]), 
            np.array([-50, -50]),
            np.array([50, -50]),
            np.array([50, 100]),
            np.array([-50, 100])
        ]
    }

    print (str(ha))
    print (repr(ha))
    
    ha_runner: AutomatonRunner = AutomatonRunner(ha, sampling_rate=0.001)
    await ha_runner.run(
        x0=x0,
        aux_x0=aux_t0,
        duration=15.0,
        real_time_mode=False,
        integrate=True,
        dt=0.1,
        collect_control=False
    )
    

    results = ha_runner.get_results()
    print (results)

    

    # async def print_state():
    #     await asyncio.sleep(2.0)  # initial delay
    #     while True:
    #         print(f"{ha.get_active_elapsed_time()}: {ha._runtime._continous_state.get_continous_state()}")
    #         await asyncio.sleep(0.01)

    # async def runner():
    #     await ha.activate(x0=x0, aux_x0=aux_t0, dt=0.1)

    # # Create tasks inside the running loop
    # t1 = asyncio.create_task(print_state())
    # t2 = asyncio.create_task(runner())
    

    # await asyncio.gather(t1, t2)

# Run the event loop
asyncio.run(main())
