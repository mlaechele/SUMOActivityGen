
import os
import random
import json
from numpy.random import RandomState

random.seed(42)

ROUTES_TPL = """<?xml version="1.0" encoding="UTF-8"?>

<!-- Generated with SUMO Activity-Based Mobility Generator [https://github.com/lcodeca/SUMOActivityGen] -->

<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd"> {trips}
</routes>"""

VEHICLE = """
<vehicle id="{id}" type="passenger" depart="{depart}" departLane="best">{route}
</vehicle>"""

ROUTE = """
    <route edges="{edges}"/>"""
    

logger = None
this_script_file = os.path.realpath(__file__)
this_script_directory = os.path.dirname(this_script_file)
filename = '../osm_activitygen_rikardo.json'
config_file = os.path.join(this_script_directory, filename)
conf = json.loads(open(config_file).read())


transit_config = {
    "a65_nord_süd": {
        "start": "204365351",
        "end": "204205696",
        "amount": 25000,
    },
    "a65_süd_nord": {
        "start": "204211401",
        "end": "294893885",
        "amount": 25000,
    },
    "b10_a65_15_west_ost": {
        "start": "254340256",
        "end": "387838399",
        "amount": 12000,
    },
    "a65_15_b10_ost_west": {
        "start": "982149684",
        "end": "-387838396",
        "amount": 12000,
    },
    "b10_a65_15_ost_west": {
        "start": "365249200",
        "end": "878698552",
        "amount": 6500,
    },
    "a65_15_b10_west_ost": {
        "start": "-878698552",
        "end": "-365249200",
        "amount": 6500,
    },
    "wollmesheim_a65_16": {
        "start": "-238401706",
        "end": "297916280",
        "amount": 5405,
    },
    "a65_16_wollmesheim_a65_16": {
        "start": "-297916280",
        "end": "169268451",
        "amount": 5405,
    },
    "a65_16_bellheim": {
        "start": "297916280",
        "end": "80007569#1",
        "amount": 3700,
    },
    "bellheim_a65_16": {
        "start": "-80007569#1",
        "end": "-297916280",
        "amount": 3700,
    },

}

trips = []
final_trips = ""

random_generator = RandomState(seed=42)

for transit_key in transit_config:
    start_edges = transit_config[transit_key]["start"]
    end_edge = transit_config[transit_key]["end"]

    for commuter_id in range(1,transit_config[transit_key]["amount"]):

        rand_type = random.randint(0, 100)

        if rand_type in range(0,20):
            depart = 21600 # 6:00
        elif rand_type in range(20,40):
            depart = 28800 # 8:00
        elif rand_type in range(40,45):
            depart = 36000 # 10:00
        elif rand_type in range(45,50):
            depart = 43200 # 12:00
        elif rand_type in range(50,55):
            depart = 50400 # 14:00
        elif rand_type in range(55,75):
            depart = 57600 # 16:00
        elif rand_type in range(75,95):
            depart = 64800 # 18:00
        else: 
            depart = 72000

        depart = random_generator.normal(
                loc=depart,
                scale=3600)

        #rand_depart = random.randint(-3600, 3600)
        #depart += rand_depart

        edges = start_edges + " " + end_edge
        trips.append((VEHICLE.format(id = "transit_" + transit_key + "_" + str(commuter_id), depart=depart,route=ROUTE.format(edges=edges)),depart))

trips = sorted(trips,key=lambda t:t[1])

for trip, _ in trips:
    final_trips += trip

this_script_file = os.path.realpath(__file__)
this_script_directory = os.path.dirname(this_script_file)
filename = '../osm_transit_traffic.rou.xml'
output_file = os.path.join(this_script_directory, filename)
with open(output_file, 'w', encoding="utf8") as tripfile:
    tripfile.write(ROUTES_TPL.format(trips=final_trips))
