# %% 
import os
import pickle
import sys
import random
import json
import logging
import collections
from agsrc import environment

random.seed(42)

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
    import traci
    import traci.constants as tc
    from traci._simulation import Stage
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

def _get_random_sample_from_list(data, amount):
  indices = random.sample(range(len(data)), amount)
  random_sample = [data[i] for i in indices]
  remaining = []
  for i in range(len(data)):
    if i in indices:
      continue
    remaining.append(data[i])
  return random_sample, remaining

this_script_file = os.path.realpath(__file__)
this_script_directory = os.path.dirname(this_script_file)
input_file = os.path.join(this_script_directory, '../osm_activitygen.trips.dump.pkl')
with open(input_file, 'rb') as f:
  all_trips = pickle.load(f)

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

ROUTES_TPL = """<?xml version="1.0" encoding="UTF-8"?>

<!-- Generated with SUMO Activity-Based Mobility Generator [https://github.com/lcodeca/SUMOActivityGen] -->

<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd"> {trips}
</routes>"""

VEHICLE = """
<vehicle id="{id}" type="passenger" depart="{depart}" departLane="best">{route}
</vehicle>"""

ROUTE = """
    <route edges="{edges}"/>"""


commuter_config = {
    "a65_nord": {
        "start": "204365351",
        "end": "294893885",
        "amount": 1979,
    },
    "a65_süd": {
        "start": "204211401",
        "end": "204205696",
        "amount": 1480,
    },
    "b272_ost": {
        "start": "365249200",
        "end": "-365249200",
        "amount": 1527,
    },
    "l506": {
        "start": "-80007569#1",
        "end": "80007569#1",
        "amount": 974,
    },
    "b10_west": {
        "start": "254340256",
        "end": "-387838396",
        "amount": 2114,
    },
    "b38_süd_west": {
        "start": "73595126#0",
        "end": "-73595126#0",
        "amount": 2114,
    },

}

saga_person_trips = list()
primary_person_trips = list()
secondary_person_trips = list()
person_trips = all_trips["all_all"]

for dict_trips in all_trips.values():
  for time in sorted(dict_trips.keys()):
    for person in dict_trips[time]:
      if not "stages" in person:
        continue
      for stage in person["stages"]:
        if stage.description.startswith('P'):
            primary_person_trips.append({
                "type": "P",
                "time": time,
                "person": person
            })
            break
        elif stage.description.startswith('S'):
            secondary_person_trips.append({
                "type": "S",
                "time": time,
                "person": person
            })
            break


commuter_outgoing_amount = 0
for key in commuter_config:
    commuter_outgoing_amount += commuter_config[key]["amount"]

random_trip_sample, remaining_trips = _get_random_sample_from_list(primary_person_trips,commuter_outgoing_amount)

saga_person_trips = secondary_person_trips + remaining_trips

trips = []
final_trips = ""
random.shuffle(random_trip_sample)

counter = 0
for key in commuter_config:
    for i in range(commuter_config[key]['amount']):
        person_trip = random_trip_sample[counter]

        new_id = person_trip["person"]["id"] + "_outgoing"
        depart_work = float(person_trip["person"]["depart"])
        home_edge = person_trip["person"]["stages"][0].edges[0]
        _, duration = env.get_timing_from_activity("P-Day")
        depart_home = depart_work + duration

        edges = home_edge + " " + commuter_config[key]['end']
        trips.append((VEHICLE.format(id = "commuter_" + key + "_" + new_id + "_work", depart=str(depart_work),route=ROUTE.format(edges=edges)), depart_work))

        edges = commuter_config[key]['start'] + " " + home_edge
        trips.append((VEHICLE.format(id = "commuter_" + key + "_" + new_id + "_home", depart=str(depart_home),route=ROUTE.format(edges=edges)), depart_home))

        counter += 1

    
trips = sorted(trips,key=lambda t:t[1])

for trip, _ in trips:
    final_trips += trip

this_script_file = os.path.realpath(__file__)
this_script_directory = os.path.dirname(this_script_file)
filename = '../osm_commuter_outgoing.rou.xml'
output_file = os.path.join(this_script_directory, filename)
with open(output_file, 'w', encoding="utf8") as tripfile:
    tripfile.write(ROUTES_TPL.format(trips=final_trips))


def _saving_trips_to_single_file(trips):
        """ Saving all the trips into a single file. """
        ## Sort (by time) all the slice into one
        
        sorted_trips = sorted(trips,key=lambda t:t["time"])

        merged_trips = collections.defaultdict(list)
        for dict_trips in sorted_trips:
          merged_trips[str(dict_trips["time"])].append(dict_trips["person"]['string'])


        this_script_file = os.path.realpath(__file__)
        this_script_directory = os.path.dirname(this_script_file)
        filename = '../osm_activitygen_without_commuter.rou.xml'
        output_file = os.path.join(this_script_directory, filename)
        with open(output_file, 'w', encoding="utf8") as tripfile:
            all_trips = ''
            for time in sorted(merged_trips.keys()):
                for person in merged_trips[time]:
                    all_trips += person

            tripfile.write(ROUTES_TPL.format(trips=all_trips))

_saving_trips_to_single_file(saga_person_trips)