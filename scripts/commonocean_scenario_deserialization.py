""" 
commonocean scenario extraction and deserialization for the purpose of running scenarios. 
in this colav automaton it is specifically used for deserialization of the commonocean marinecadestre mid east coast
off the us, based on real AIS data scenarios. 

... 
"""
import os
import xml.etree.ElementTree as ET

# Path to a scenario directory from the external commonocean-scenarios
# dataset (https://commonocean.cps.cit.tum.de/) - not part of this repo.
# Set COMMONOCEAN_SCENARIO_DIR to point at your local checkout, or edit
# the fallback below.
scenario_dir = os.environ.get(
    "COMMONOCEAN_SCENARIO_DIR",
    "/home/ryan/commonocean-scenarios-main-scenarios-MarineCadastre_01_19-UpperWestCoast/commonocean-scenarios-main-scenarios-MarineCadastre_01_19-UpperWestCoast/scenarios/MarineCadastre_01_19/UpperWestCoast"
)
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



print (initial_point)
print (goal_point)