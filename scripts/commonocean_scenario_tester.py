import matplotlib.pyplot as plt
import imageio


from commonocean.common.file_reader import CommonOceanFileReader
from commonocean.visualization.draw_dispatch_cr import draw_object
from commonroad.visualization.mp_renderer import MPRenderer

file_path = "/home/ryan/commonocean-sim/commonocean-scenarios-main/scenarios/HandcraftedTwoVesselEncounters_01_24/ZAM_AAA-1_20240121_T-1596.xml"

scenario, planning_problem_set = CommonOceanFileReader(file_path).open()

# plot scenario and planning_problem_set for time frame 1
frame = 1
i = 1
plt.figure(figsize=(9, 4))
draw_object(scenario, draw_params={'time_begin': frame,'trajectory_steps': 0})
draw_object(planning_problem_set, draw_params={'time_begin': i})
plt.gca().set_aspect('equal')
plt.show()


print ('test')

