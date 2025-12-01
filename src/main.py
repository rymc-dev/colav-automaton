import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__)))

from colav_automaton import ColavAutomaton
from hybrid_automaton import Automaton
import asyncio
import numpy as np


async def main():
    ha: Automaton = ColavAutomaton()
    x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    aux_t0 = {
        "waypoints": [(10.0, 10.0)]
    }

    async def print_state():
        await asyncio.sleep(2.0)  # initial delay
        while True:
            print(f"{ha._runtime._time_elapsed_active}: {ha._runtime._continous_state.get_continous_state()}")
            await asyncio.sleep(0.01)

    async def runner():
        await ha.activate(x0=x0, aux_x0=aux_t0, dt=0.1)

    # Create tasks inside the running loop
    t1 = asyncio.create_task(print_state())
    t2 = asyncio.create_task(runner())

    await asyncio.gather(t1, t2)

# Run the event loop
asyncio.run(main())
