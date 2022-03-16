# %% 
import os
import sys
import random
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
<vehicle id="{id}" type="passenger" depart="{depart}" departLane="best">{route}
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
filename = '../osm_activitygen_rikardo.json'
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

commuter_config = {
    "a65_nord": {
        "start": "204365351",
        "end": "294893885",
        "amount": 2501,
    },
    "a65_süd": {
        "start": "204211401",
        "end": "204205696",
        "amount": 667,
    },
    "b272_ost": {
        "start": "365249200",
        "end": "-365249200",
        "amount": 1787,
    },
    "l506": {
        "start": "-80007569#1",
        "end": "80007569#1",
        "amount": 1525,
    },
    "b10_west": {
        "start": "254340256",
        "end": "-387838396",
        "amount": 5430,
    },
    "b38_süd_west": {
        "start": "73595126#0",
        "end": "-73595126#0",
        "amount": 4429,
    },

}

trips = []
final_trips = ""

for commuter_key in commuter_config:
    start_edges = commuter_config[commuter_key]["start"]
    end_edge = commuter_config[commuter_key]["end"]

    for commuter_id in range(1,commuter_config[commuter_key]["amount"]):

        start, duration = env.get_timing_from_activity("P-Day")

        rand_type = random.randint(0, 100)

        if rand_type in range(0,10): # additional buildings
            key = 'additional'
        elif rand_type in range(10,40): # commercial buildings
            key = 'commercial'
        elif rand_type in range(40,70): # industrial buildings
            key = 'industrial'
        else: # downtown buildings
            key = 'downtown'

        buildings = env._buildings[key]

        building_index = random.randint(0,len(buildings) - 1)

        building = buildings[building_index]
        building_g_edge = building[2]

        parking_area = env._parkings_by_edge_id[building_g_edge]
        parking_area_id = parking_area[0]
        parkingEdge = parking_area[1]

        edges = start_edges + " " + parkingEdge + " " + end_edge
        trips.append((VEHICLE.format(id = "commuter_" + commuter_key + "_" + str(commuter_id), depart=str(start),route=ROUTE.format(edges=edges) + " "+ STOP.format(parking_area=parking_area_id,duration=duration)),start))

trips = sorted(trips,key=lambda t:t[1])

for trip, _ in trips:
    final_trips += trip

this_script_file = os.path.realpath(__file__)
this_script_directory = os.path.dirname(this_script_file)
filename = '../osm_commuter_incoming.rou.xml'
output_file = os.path.join(this_script_directory, filename)
with open(output_file, 'w', encoding="utf8") as tripfile:
    tripfile.write(ROUTES_TPL.format(trips=final_trips))
