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
            "unsafe_region": [
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

async def run_automaton():
    results: RunResult = await automaton.activate(
        initial_continuous_state=ContinuousState(
            name="agent_state", 
            x0=scenarios['t1']['environment']['initial_state'], 
            x_labels=["x", "y", "theta", "velocity", "yaw_rate"]
        ),
        initial_auxiliary_states=[
            AuxiliaryState("waypoints", aux0=scenarios["t1"]['environment']['waypoint']),
            AuxiliaryState("unsafe_region", aux0=scenarios['t1']['environment']['unsafe_region'])
        ],
        delta_time=0.1,
        enable_real_time_mode=False,
        continuous_state_sampler_enabled=True,
        continuous_state_sampler_rate=10,
        auxiliary_states_sampler_enabled=True,
        auxiliary_states_sampler_rate=10,
        should_write_logs=True,
        output_dir="./colav-automaton_logs" 
    )
    print (results)

asyncio.run(run_automaton())