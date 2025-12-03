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
    ha: Automaton = ColavAutomaton(heading_tolerance=0.2, k_theta=1.0, constant_velocity=2.0, acceptance_radius=50)
    x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
    aux_t0 = {
        "waypoints": [np.array([50.0, 50.0]), np.array([100.0, 100.0])],
    }

    print (str(ha))
    print (repr(ha))
    
    ha_runner: AutomatonRunner = AutomatonRunner(ha, sampling_rate=0.001)
    await ha_runner.run(
        x0=x0,
        aux_x0=aux_t0,
        duration=5.0,
        real_time_mode=False,
        integrate=True,
        dt=0.01,
        collect_control=False
    )
    
    import matplotlib.pyplot as plt
    results = ha_runner.get_results()
    print (results)
    fig1 = continuous_states_over_time_fig(results['continuous_states'])
    fig2 = auxiliary_states_over_time_fig(results['auxiliary_states'])
    fig3 = automaton_states_over_time(results['automaton_states'])
    fig4 = transitions_times_over_time_fig(results['transition_times'])
    plt.show()
    
    import time 
    
    time.sleep(10.0)
    
    

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
