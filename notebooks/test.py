import xml
import matplotlib.pyplot as plt
import xml.etree.cElementTree as ET

scneario_file = '/home/ryan/colav-automaton/notebooks/scenarios/scenario_1.xml'

scenario_tree = ET.parse(scneario_file)
scenario_root = scenario_tree.getroot()

matrix = scenario_root.find('matrix') 
matrix_width = matrix.find('width')
matrix_height = matrix.find('height')

obstacles = scenario_root.find('obstacles')

for obstacle in obstacles: 
    print (obstacle.find('tag').text)
    print (obstacle.find('position').text)
    orientation_values = str(obstacle.find('orientation').text).split(',')
    for value in orientation_values:
        print(value)
    print (obstacle.find('velocity').text)
    print (obstacles.find('yaw_rate').text)


print (scneario_file)