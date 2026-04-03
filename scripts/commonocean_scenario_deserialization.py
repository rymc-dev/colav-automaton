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
for dynamic_obstacle in root.findall('DynamicObstacle'):
    obstacle_type = dynamic_obstacle.find('type').text
    obstacle_length = float(dynamic_obstacle.find('shape').find('rectangle').find('length').text)
    obstacle_width = float(dynamic_obstacle.find('shape').find('rectangle').find('width').text)
    obstacle_initial_state = [(float(point.find('x').text), float(point.find('y').text)) for point in dynamic_obstacle.find('initialState').find('position').find('point')]
    obstacle_initial_state_orientation = float(dynamic_obstacle.find('initialState').find('orientation').find('exact'))
    
for static_obstacle in root.findall('StaticObstacles'):
    ... 

initial_state = [(position.find('x'), position.find('y')) for position in root.find('planningProblem').find('initialState').find('position')]
goal_state = [(position.find('x'), position.find('y')) for position in root.find('planningProblem').find('goalState').find('position').find('rectange').find('center')]



