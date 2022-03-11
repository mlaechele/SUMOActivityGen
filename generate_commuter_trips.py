# %% 
import os
import sys
import random
import collections
from agsrc import environment
import json
import logging

random.seed(42)


# %%

ROUTES_TPL = """<?xml version="1.0" encoding="UTF-8"?>

<!-- Generated with SUMO Activity-Based Mobility Generator [https://github.com/lcodeca/SUMOActivityGen] -->

<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd"> {trips}
</routes>"""

VEHICLE = """
<vehicle id="{id}" type="passenger" depart="{depart}">{route}
</vehicle>"""

ROUTE = """
    <route edges="{edges}"/>"""

STOP = """
        <stop parkingArea="{parking_area}" duration="{duration}"/>"""

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
    import traci
    import traci.constants as tc
    from traci._simulation import Stage
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

logger = None
this_script_file = os.path.realpath(__file__)
this_script_directory = os.path.dirname(this_script_file)
filename = '../osm_activitygen.json'
config_file = os.path.join(this_script_directory, filename)
conf = json.loads(open(config_file).read())

def configure_loggers():
    """ Setup the console and file logger. """
    logger = logging.getLogger('ActivityGen')
    logger.setLevel(logging.DEBUG)
    _console_handler = logging.StreamHandler()
    _console_handler.setLevel(logging.INFO)
    _console_handler.setFormatter(
        logging.Formatter('[%(asctime)s] %(levelname)s: %(message)s'))
    logger.addHandler(_console_handler)
    _file_handler = logging.FileHandler('{}.debug.log'.format(conf['outputPrefix']))
    _file_handler.setLevel(logging.DEBUG)
    _file_handler.setFormatter(
        logging.Formatter(
            '[%(asctime)s]:[%(name)s]:[%(module)s:%(funcName)s:%(lineno)d]:[%(levelname)s] '
            '%(message)s'))
    logger.addHandler(_file_handler)
    return logger

logger = configure_loggers()

traci.start(['sumo', '-c', conf['sumocfg']], traceFile='traci.log')
env = environment.Environment(
            conf, sumo=traci, profiling=False, logger=logger) 

# %%

start_edges = "202155857"
end_edge = "31947791"
trips = []
final_trips = ""
for commuter_id in range(1,30):

    start, duration = env.get_timing_from_activity("P-Day")

    building_type_index = random.randint(1,len(env._buildings) - 1)
    key = list(env._buildings.keys())[building_type_index]

    buildings = env._buildings[key]

    building_index = random.randint(0,len(buildings) - 1)

    building = buildings[building_index]
    building_g_edge = building[2]

    parking_area = env._parkings_by_edge_id[building_g_edge]
    parking_area_id = parking_area[0]
    parkingEdge = parking_area[1]

    edges = start_edges + " " + parkingEdge + " " + end_edge
    trips.append((VEHICLE.format(id = "commuter_" + str(commuter_id), depart=str(start),route=ROUTE.format(edges=edges) + " "+ STOP.format(parking_area=parking_area_id,duration=duration)),start))

trips = sorted(trips,key=lambda t:t[1])

for trip, _ in trips:
    final_trips += trip

this_script_file = os.path.realpath(__file__)
this_script_directory = os.path.dirname(this_script_file)
filename = 'osm_commuter.rou.xml'
output_file = os.path.join(this_script_directory, filename)
with open(output_file, 'w', encoding="utf8") as tripfile:
    tripfile.write(ROUTES_TPL.format(trips=final_trips))

# %%
