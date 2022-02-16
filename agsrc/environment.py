#!/usr/bin/env python3

""" SUMO Activity-Based Mobility Generator - Environment

    Author: Lara CODECA

    This program and the accompanying materials are made available under the
    terms of the Eclipse Public License 2.0 which is available at
    http://www.eclipse.org/legal/epl-2.0.
"""

import collections
import csv
import logging
import os
from pprint import pformat
from shutil import ExecError
import sys
import xml.etree.ElementTree

from numpy.random import RandomState

from agsrc import sagaexceptions, sumoutils

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
    import sumolib
    from traci.exceptions import TraCIException
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

class Environment():
    """ Loads, stores, interact the SAGA evironment required for the mobility generation. """

    def __init__(self, conf, sumo, logger, profiling=False):
        """
        Initialize the synthetic population.
            :param conf: distionary with the configurations
            :param sumo: already initialized SUMO simulation (TraCI or LibSUMO)
            :param profiling=False: enable cProfile
        """
        self._conf = conf
        self._sumo = sumo
        self.logger = logger

        self._max_retry_number = 1000
        if 'maxNumTry' in conf:
            self._max_retry_number = conf['maxNumTry']

        self._profiling = profiling

        self._random_generator = RandomState(seed=self._conf['seed'])

        self.logger.info('Loading SUMO net file %s', conf['SUMOnetFile'])
        self.sumo_network = sumolib.net.readNet(conf['SUMOnetFile'])

        self.logger.info('Loading SUMO parking lots from file %s', conf['SUMOadditionals']['parkings'])
        self._blacklisted_edges = set()
        self._sumo_parkings = collections.defaultdict(list)
        self._parking_cache = dict()
        self._parking_position = dict()
        self._load_parkings(conf['SUMOadditionals']['parkings'])

        self.logger.info('Loading SUMO taxi stands from file %s', conf['intermodalOptions']['taxiStands'])
        self._sumo_taxi_stands = collections.defaultdict(list)
        self._taxi_stand_cache = dict()
        self._taxi_stand_position = dict()
        self._load_taxi_stands(conf['intermodalOptions']['taxiStands'])

        self.logger.info('Loading TAZ weights from %s', conf['population']['tazWeights'])
        self._taz_weights = dict()
        self._load_weights_from_csv(conf['population']['tazWeights'])

        self.logger.info('Loading buildings weights from %s', conf['population']['buildingsWeight'])
        self._buildings_by_taz = dict()
        self._building_additionals_by_id = dict()
        self._load_buildings_from_csv_dir(conf['population']['buildingsWeight'])

        self.logger.info('Loading edges in each TAZ from %s', conf['population']['tazDefinition'])
        self._edges_by_taz = dict()
        self._load_edges_from_taz(conf['population']['tazDefinition'])

        self.logger.info('Storing building ids categorized')
        self._residential_buildings = []
        self._commercial_buildings = []
        self._industrial_buildings = []
        self._store_buildings_categorized()

    def _get_all_files_from_dir(self, directory):
        return [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]

    def _store_buildings_categorized(self):
        for taz in self._buildings_by_taz:
            for building in self._buildings_by_taz[taz]:
                id = building[0]
                if id not in self._building_additionals_by_id:
                    continue
                building_type, _ = self._building_additionals_by_id[id]
                if building_type == 'residential':
                    self._residential_buildings.append(building)
                if building_type == 'commercial':
                    self._commercial_buildings.append(building)
                if building_type == 'industrial':
                    self._industrial_buildings.append(building)
        self.logger.info('Stored %d residential, %d commercial and %d industrial buildings', 
            len(self._residential_buildings), len(self._commercial_buildings), len(self._industrial_buildings))

    # LOADERS

    def _load_parkings(self, filename):
        """ Load parkings ids from XML file. """
        if not os.path.isfile(filename):
            return
        xml_tree = xml.etree.ElementTree.parse(filename).getroot()
        for child in xml_tree:
            if child.tag != 'parkingArea':
                continue
            if child.attrib['id'] not in self._conf['intermodalOptions']['parkingAreaBlacklist']:
                edge = child.attrib['lane'].split('_')[0]
                position = float(child.attrib['startPos']) + 2.5
                self._sumo_parkings[edge].append(child.attrib['id'])
                self._parking_position[child.attrib['id']] = position

    def _load_taxi_stands(self, filename):
        """ Taxi stands ids from XML file. """
        if not os.path.isfile(filename):
            return
        xml_tree = xml.etree.ElementTree.parse(filename).getroot()
        for child in xml_tree:
            if child.tag != 'parkingArea':
                continue
            if child.attrib['id'] not in self._conf['intermodalOptions']['taxiStandsBlacklist']:
                edge = child.attrib['lane'].split('_')[0]
                position = float(child.attrib['startPos']) + 2.5
                self._sumo_taxi_stands[edge].append(child.attrib['id'])
                self._taxi_stand_position[child.attrib['id']] = position

    def _load_weights_from_csv(self, filename):
        """ Load the TAZ weight from a CSV file. """
        with open(filename, 'r') as csvfile:
            weightreader = csv.reader(csvfile)
            header = []
            for row in weightreader:
                if not row:
                    continue # empty line
                if not header:
                    header = row
                elif row: # ignoring empty lines
                    self._taz_weights[row[0]] = {
                        header[0]: row[0],
                        header[1]: row[1],
                        header[2]: int(row[2]),
                        header[3]: float(row[3]),
                        'weight': (int(row[2])/float(row[3])),
                    }

    def _load_buildings_from_csv_dir(self, directory):
        """ Load the buildings data from multiple CSV files. """
        allfiles = self._get_all_files_from_dir(directory)
        for filename in sorted(allfiles):
            if filename.endswith('.csv'):
                self.logger.debug('Loading %s', filename)
                if filename.endswith('.add.csv'):
                    self._load_building_additionals_from_csv(filename)
                else:
                    self._load_building_weights_from_csv(filename)

    def _load_building_additionals_from_csv(self, filename):
        """ Load the building additionals from CSV file. """
        with open(filename, 'r') as csvfile:
            reader = csv.reader(csvfile)
            header = None
            for row in reader:
                if not row:
                    continue # empty line
                if header is None:
                    header = row
                else:
                    poly = row[0]
                    building_type = row[1]
                    population = float(row[2])
                    self._building_additionals_by_id[poly] = (building_type, population)

    def _load_building_weights_from_csv(self, filename):
        """ Load the building weights from CSV file. """
        with open(filename, 'r') as csvfile:
            weightreader = csv.reader(csvfile)
            header = None
            taz = None
            buildings = []
            for row in weightreader:
                if not row:
                    continue # empty line
                if header is None:
                    header = row
                else:
                    taz = row[0]
                    buildings.append((row[1],float(row[3]),    # weight
                                        row[4],           # generic edge
                                        row[5]))          # pedestrian edge

            if len(buildings) < 10:
                self.logger.debug('Dropping %s, only %d buildings found.', filename, len(buildings))
                return

            weighted_buildings = []
            cum_sum = 0.0
            for id, weight, g_edge, p_edge in sorted(buildings):
                cum_sum += weight
                weighted_buildings.append((id, cum_sum, g_edge, p_edge, weight))
            self._buildings_by_taz[taz] = weighted_buildings

    def _load_edges_from_taz(self, filename):
        """ Load edges from the TAZ file. """
        xml_tree = xml.etree.ElementTree.parse(filename).getroot()
        for child in xml_tree:
            if child.tag == 'taz':
                self._edges_by_taz[child.attrib['id']] = child.attrib['edges'].split(' ')

    # LANES & EDGES

    def get_random_lane_from_tazs(self):
        """
        Retrieve a random edge usable by a taxi based on the option
            "intermodalOptions":"taxiFleetInitialTAZs": ['taz', ...]
        """
        _locations = self._conf['intermodalOptions']['taxiFleetInitialTAZs']
        _lane = None
        _retry_counter = 0
        while not _lane and _retry_counter < self._max_retry_number * 100:
            try:
                if _locations:
                    _taz = self._random_generator.choice(_locations)
                    _edges = self._edges_by_taz[_taz]
                    _edge = self._random_generator.choice(_edges)
                else:
                    _edge = self._random_generator.choice(self.sumo_network.getEdges()).getID()
                _lane = self.get_stopping_lane(_edge, ['taxi', 'passenger'])
            except sagaexceptions.TripGenerationGenericError:
                _retry_counter += 1
                _lane = None
        if _lane is None:
            self.logger.critical(
                '_get_random_lane_from_TAZs with "%s" generated %d errors, '
                'taxi generation aborted..', pformat(_locations), _retry_counter)
        return _lane

    def get_all_neigh_edges(self, origin, distance):
        """ Returns all the edges reachable from the origin within the given radius. """
        _edge_shape = self.sumo_network.getEdge(origin).getShape()
        x_coord = _edge_shape[-1][0]
        y_coord = _edge_shape[-1][1]
        edges = self.sumo_network.getNeighboringEdges(x_coord, y_coord, r=distance)
        edges = [edge.getID() for edge, _ in edges]
        return edges

    def get_arrival_pos_from_edge(self, edge, position):
        """
        If the position is too close to the end, it may genrate error with
        findIntermodalRoute.
        """
        length = self.sumo_network.getEdge(edge).getLength()
        if length < self._conf['minEdgeAllowed']:
            return None
        if position > length - 1.0:
            return length - 1.0
        if position < 1.0:
            return 1.0
        return position

    def get_random_pos_from_edge(self, edge):
        """ Return a random position in the given edge. """
        length = self.sumo_network.getEdge(edge).getLength()
        position = None
        if length < self._conf['stopBufferDistance']:
            position = length/2.0

        # avoid the proximity of the intersection
        begin = self._conf['stopBufferDistance'] / 2.0
        end = length - begin
        position = (end - begin) * self._random_generator.random_sample() + begin
        self.logger.debug('get_random_pos_from_edge: [%s] %f (%f)', edge, position, length)
        
        # If the position is too close to the end, it may genrate error with findIntermodalRoute.
        if position > length - 1.0:
            position = length - 1.0
        if position < 1.0:
            position = 1.0
        return position

    def get_stopping_lane(self, edge, vtypes=['passenger']):
        """
        Returns the vehicle-friendly stopping lane closer to the sidewalk that respects the
        configuration parameter 'minEdgeAllowed'.
        """
        for lane in self.sumo_network.getEdge(edge).getLanes():
            if lane.getLength() >= self._conf['minEdgeAllowed']:
                for vtype in vtypes:
                    if lane.allows(vtype):
                        return lane.getID()
        raise sagaexceptions.TripGenerationGenericError(
            '"{}" cannot stop on edge {}'.format(vtypes, edge))

    ## PARKING AREAS: location and selection

    def get_parking_position(self, parking_id):
        """ Returns the position for a given parking. """
        return self._parking_position[parking_id]

    def find_closest_parking(self, edge):
        """ Given and edge, find the closest parking area. """
        distance = sys.float_info.max

        ret = self._check_parkings_cache(edge)
        if ret:
            return ret

        p_id = None

        for p_edge, parkings in self._sumo_parkings.items():
            for parking in parkings:
                if parking not in self._conf['intermodalOptions']['parkingAreaBlacklist']:
                    p_id = parking
                    break
            if p_id:
                try:
                    route = self._sumo.simulation.findIntermodalRoute(
                        p_edge, edge, pType="pedestrian")
                except TraCIException:
                    route = None
                if route and not isinstance(route, list):
                    # list in until SUMO 1.4.0 included, tuple onward
                    route = list(route)
                if route:
                    cost = sumoutils.cost_from_route(route)
                    if distance > cost:
                        distance = cost
                        ret = p_id, p_edge, route

        if ret:
            self._parking_cache[edge] = ret
            return ret

        self.logger.fatal('Edge %s is not reachable from any parking lot.', edge)
        self._blacklisted_edges.add(edge)
        return None, None, None

    def _check_parkings_cache(self, edge):
        """ Check among the previously computed results of _find_closest_parking """
        if edge in self._parking_cache.keys():
            return self._parking_cache[edge]
        return None

## ---- PAIR SELECTION: origin - destination - mode ---- ##

    def select_pair(self, from_building, to_area, pedestrian=False):
        """ Randomly select one pair, chosing between buildings and TAZ. """

        # TODO: select tazes when not using single taz mode
        # from_taz = str(self._select_taz_of_building_id(from_building[0]))
        # to_taz = str(self._select_taz_from_weighted_area(to_area))

        from_taz = 'all'
        to_taz = 'all'
        
        if from_taz in self._buildings_by_taz.keys() and to_taz in self._buildings_by_taz.keys():
            return self._select_pair_from_taz_wbuildings(
                from_building, self._buildings_by_taz[to_taz][:], pedestrian)
        return self._select_pair_from_taz(
            self._edges_by_taz[from_taz][:], self._edges_by_taz[to_taz][:])

    def valid_pair(self, from_edge, to_edge):
        """ This is just to avoid a HUGE while condition.
            sumolib.net.edge.is_fringe()
        """

        if not self.sumo_network.hasEdge(to_edge):
            return False

        from_edge_sumo = self.sumo_network.getEdge(from_edge)
        to_edge_sumo = self.sumo_network.getEdge(to_edge)

        if from_edge_sumo.is_fringe(from_edge_sumo.getOutgoing()):
            return False
        if to_edge_sumo.is_fringe(to_edge_sumo.getIncoming()):
            return False
        if from_edge == to_edge:
            return False
        if to_edge in self._blacklisted_edges:
            return False
        if not to_edge_sumo.allows('pedestrian'):
            return False
        return True

    def _select_taz_of_building_id(self, id):
        taz_of_building = None
        for taz in self._buildings_by_taz:
            for building_id, _, _, _, _ in self._buildings_by_taz[taz]:
                if id == building_id:
                    taz_of_building = taz
                    break
            if taz_of_building is not None:
                break
        return taz_of_building

    def _select_taz_from_weighted_area(self, area):
        """ Select a TAZ from an area using its weight. """
        selection = self._random_generator.uniform(0, 1)
        total_weight = sum([self._taz_weights[taz]['weight'] for taz in area])
        if total_weight <= 0:
            error_msg = 'Error with area {}, total sum of weights is {}. '.format(
                area, total_weight)
            error_msg += 'It must be strictly positive.'
            raise Exception(error_msg, [(taz, self._taz_weights[taz]['weight']) for taz in area])
        cumulative = 0.0
        for taz in area:
            cumulative += self._taz_weights[taz]['weight'] / total_weight
            if selection <= cumulative:
                return taz
        return None # this is matematically impossible,
                    # if this happens, there is a mistake in the weights.

    def _select_pair_from_taz(self, from_taz, to_taz):
        """ Randomly select one pair from a TAZ.
            Important: from_taz and to_taz MUST be passed by copy.
            Note: sumonet.getEdge(from_edge).allows(v_type) does not support distributions.
        """

        from_edge = from_taz.pop(
            self._random_generator.randint(0, len(from_taz)))
        to_edge = to_taz.pop(
            self._random_generator.randint(0, len(to_taz)))

        _to = False
        while not self.valid_pair(from_edge, to_edge) and from_taz and to_taz:
            if not self.sumo_network.getEdge(to_edge).allows('pedestrian') or _to:
                to_edge = to_taz.pop(
                    self._random_generator.randint(0, len(to_taz)))
                _to = False
            else:
                from_edge = from_taz.pop(
                    self._random_generator.randint(0, len(from_taz)))
                _to = True

        return from_edge, to_edge

    def _select_pair_from_taz_wbuildings(self, from_building, to_buildings, pedestrian):
        """ Randomly select one pair from a TAZ.
            Important: to_buildings MUST be passed by copy.
            Note: sumonet.getEdge(from_edge).allows(v_type) does not support distributions.
        """
        from_edge = from_building[2] # g_edge
        to_edge = self._get_random_commercial_or_industrial_building_edge(pedestrian)

        if not self.valid_pair(from_edge, to_edge):
            if to_edge.startswith('-'):
                to_edge = to_edge[1:]
            else:
                to_edge = '-' + to_edge

        while not self.valid_pair(from_edge, to_edge) and to_buildings:
            to_edge = self._get_random_commercial_or_industrial_building_edge(pedestrian)

        return from_edge, to_edge

    def _get_random_commercial_or_industrial_building_edge(self, pedestrian):
        rand_type = self._random_generator.randint(0, 2)
        g_edge = None
        p_edge = None
        if rand_type == 0: # commercial buildings
            rand_building_idx = self._random_generator.randint(0, len(self._commercial_buildings))
            _, _, g_edge, p_edge, _ = self._commercial_buildings[rand_building_idx]
        else: # industrial buildings
            rand_building_idx = self._random_generator.randint(0, len(self._industrial_buildings))
            _, _, g_edge, p_edge, _ = self._industrial_buildings[rand_building_idx]
        if pedestrian:
            return p_edge
        return g_edge

    @staticmethod
    def _get_weighted_edge(edges, double, pedestrian):
        """ Return an edge and its position using the cumulative sum of the weigths in the area. """
        pos = -1
        ret = None
        for _, cum_sum, g_edge, p_edge, _ in edges:
            if ret and cum_sum > double:
                return ret, pos
            if pedestrian and p_edge:
                ret = p_edge
            elif not pedestrian and g_edge:
                ret = g_edge
            elif g_edge:
                ret = g_edge
            else:
                ret = p_edge
            pos += 1
        return edges[-1][1], len(edges) - 1

## ---- Helper functions ---- ##

    def get_total_population(self):
        population = 0
        for buildings in self._buildings_by_taz.values():
            for id, _, _, _, _ in buildings:
                if id in self._building_additionals_by_id:
                    population += self._building_additionals_by_id[id][1]
        return population

    def get_buildings_by_taz(self):
        return self._buildings_by_taz

    def get_population_by_building_id(self, id):
        return self._building_additionals_by_id[id][1]

    def has_additionals_for_building_id(self, id):
        if id in self._building_additionals_by_id:
            return True
        return False 
