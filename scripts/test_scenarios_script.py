import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), "..", 'src'))

import asyncio

from colav_automaton import ColavAutomaton
from hybrid_automaton import RunResult
from hybrid_automaton import Automaton
from hybrid_automaton import ContinuousState
from hybrid_automaton import AuxiliaryState
from hybrid_automaton import RunResult

from hybrid_automaton import RuntimeContext

from hybrid_automaton.definition import continuous_state_provider, auxiliary_state_provider, control_input_states_provider


from colav_unsafe_set.unsafe_set.unsafe_set import create_unsafe_set
from colav_unsafe_set.objects import Agent, DynamicObstacle


DSF = 100.0

""" 
commonocean scenario extraction and deserialization for the purpose of running scenarios. 
in this colav automaton it is specifically used for deserialization of the commonocean marinecadestre mid east coast
off the us, based on real AIS data scenarios. 

... 
"""
import os
import xml.etree.ElementTree as ET

scenario_dir = "/home/ryan/commonocean-scenarios-main-scenarios-MarineCadastre_01_19-UpperWestCoast/commonocean-scenarios-main-scenarios-MarineCadastre_01_19-UpperWestCoast/scenarios/MarineCadastre_01_19/UpperWestCoast"
scenario_name = "USA_UWC-1_20190112_T-16.xml"

tree = ET.parse(os.path.join(scenario_dir, scenario_name))
root = tree.getroot()

location_data = root.find('location')
geoNameID = location_data.find('geoNameId').text
gpsLatitude = location_data.find('gpsLatitude').text
gpsLongitude = location_data.find('gpsLongitude').text

scenario_tags = root.find('scenarioTags')
tags = [child.tag for child in scenario_tags]

dynamic_obstacles = []
for dynamic_obstacle in root.findall('dynamicObstacle'):
    obstacle_type = dynamic_obstacle.find('type').text
    obstacle_length = float(dynamic_obstacle.find('shape').find('rectangle').find('length').text)
    obstacle_width = float(dynamic_obstacle.find('shape').find('rectangle').find('width').text)
    point = dynamic_obstacle.find('initialState').find('position').find('point')
    obstacle_initial_position = (float(point.find('x').text), (float(point.find('y').text)))
    obstacle_initial_state_orientation = float(dynamic_obstacle.find('initialState').find('orientation').find('exact').text)
    dynamic_obstacle = {
        "type": obstacle_type,
        "length": obstacle_length,
        "width": obstacle_width,
        "point": point,
        "obstacle_initial_position": obstacle_initial_position,
        "obstacle_initial_state_orientation": obstacle_initial_state_orientation
    }
    dynamic_obstacles.append(
        dynamic_obstacle
    )
    
static_obstacles = []
for static_obstacle in root.findall('StaticObstacles'):
    ... 

initial_point =  root.find('planningProblem').find('initialState').find('position').find('point')
initial_point = float(initial_point.find('x').text), float(initial_point.find('y').text)

goal_point = root.find('planningProblem').find('goalState').find('position').find('rectangle').find('center')
goal_point = float(goal_point.find('x').text), float(goal_point.find('y').text)

scenarios = {
    "t1": {
        "hyperparameters": {
            "heading_tolerance": 0.2,
            "k_theta": 1.0,
            "k_v": 1.0,
            "constant_velocity": 2.0,
            "safety_radius": 30.0,
            "acceptance_radius": 5, 
            "los_distance_threshold": 60.0, 
            "longitudinal_offset_distance": 50.0, 
            "lateral_offset_distance": 50.0
        },
        "environment": {
            "initial_state": [initial_point[0], initial_point[1], 0.0, 0.0, 0.0],
            "obstacles": {
              "static_obstacles": static_obstacles,
              "dynamic_obstacles": dynamic_obstacles  
            },
            "unsafe_region": [ # unsafe set should be created initially based on status of static obstacles and dynamic obstacles
                [40.0, 20.0],
                [80.0, 20.0],
                [80.0, 60.0],
                [40.0, 60.0]
            ],
            "waypoint": goal_point
        }
    }
}


automaton: Automaton = ColavAutomaton(**scenarios["t1"]["hyperparameters"]) 

from typing import Dict

@auxiliary_state_provider
def obstacle_state_and_unsafe_set_provider(ctx: RuntimeContext) -> Dict: 
    # 1. get continuous state 
    x = ctx.continuous_state.latest()
    # 2. dynamic obstacles are an external data source while 
    #    unsafe set is passed in so need to pass auxiliary state in manually 
    obstacles_state = ctx.auxiliary_states.get('obstacles_state', {})
    
    # conver to unsafe set objects
    x = Agent(position=(x[0], x[1], x[2]), orientation=x[3], velocity=x[4], yaw_rate=x[5], safety_radius=20.0)
    # need to iterate dynamic obstacles here and conver then 
    dynamic_obstacles = []
    ctx.auxiliary_states['unsafe_region'] = create_unsafe_set(
        agent=x,
        dynamic_obstacles=dynamic_obstacles,
        dsf=DSF 
    )
    
    return ctx.auxiliary_states 

async def run_automaton():
    results: RunResult = await automaton.activate(
        initial_continuous_state=ContinuousState(
            name="agent_state", 
            x0=scenarios['t1']['environment']['initial_state'], 
            x_labels=["x", "y", "theta", "velocity", "yaw_rate"]
        ),
        initial_auxiliary_states=[
            AuxiliaryState("waypoints", aux0=scenarios["t1"]['environment']['waypoint']),
            AuxiliaryState("obstacles", aux0={}),
            AuxiliaryState("unsafe_region", aux0=scenarios['t1']['environment']['unsafe_region'])
        ],
        delta_time=0.1,
        enable_real_time_mode=False,
        continuous_state_sampler_enabled=True,
        continuous_state_sampler_rate=10,
        auxiliary_states_sampler_enabled=True,
        auxiliary_states_sampler_rate=10,
        auxiliary_states_provider=obstacle_state_and_unsafe_set_provider,
        auxiliary_states_provision_rate=1,
        should_write_logs=True,
        output_dir="./colav-automaton-logs" 
    )
    print (results)

asyncio.run(run_automaton())