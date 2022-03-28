
import os
import sys
import random
import csv
from numpy.random import RandomState

ADDITIONAL = """<additional>{loops}
</additional>"""

LOOP = """
<inductionLoop id="{id}" lane="{lane}" pos="{pos}" freq="60" file="induction_loops.out.xml" friendlyPos="true"/>
"""

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
    import sumolib
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

this_script_file = os.path.realpath(__file__)
this_script_directory = os.path.dirname(this_script_file)
induction_loop_coordinates_filename = '../induction_loop_coordinates.csv'
induction_loop_coordinates_file_path = os.path.join(this_script_directory, induction_loop_coordinates_filename)

sumo_net_filename = '../osm.net.xml'
sumo_net_file_path = os.path.join(this_script_directory, sumo_net_filename)
sumo_network = sumolib.net.readNet(sumo_net_file_path)

with open(induction_loop_coordinates_file_path) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    next(csv_reader)
    
    loops = ""
    for row in csv_reader:
        id = 1
        induction_loop_station_id = row[0]
        x,y = sumo_network.convertLonLat2XY(row[1],row[2])
        edges = sumo_network.getNeighboringEdges(x,y,r=20)
        edges = sorted(edges,key=lambda t:t[1])
        isPassengerLane = False
        edgeCounter = 0
                
        for edge, _ in edges:
            lanes = edge.getLanes()
            lanes = sorted(lanes,key=lambda t:t.getIndex())
            for lane in lanes:
                if lane.allows("passenger"):
                    isPassengerLane = True
                    loops += LOOP.format(id=str(induction_loop_station_id) + "_" + str(id),pos=str(lane.getLength()/2),lane=lane.getID())
                    id += 1
            if isPassengerLane:
                edgeCounter += 1
                isPassengerLane = False
            
            if edgeCounter == 2:
                break
        

this_script_file = os.path.realpath(__file__)
this_script_directory = os.path.dirname(this_script_file)
filename = '../induction_loops.add.xml'
output_file = os.path.join(this_script_directory, filename)
with open(output_file, 'w', encoding="utf8") as tripfile:
    tripfile.write(ADDITIONAL.format(loops=loops))