#!/usr/bin/env python3

import argparse
import collections
import json
import logging
import os
import pickle
import sys
import xml.etree.ElementTree
from tqdm import tqdm

from numpy.random import RandomState

from agsrc import sagaexceptions, sumoutils

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
    import sumolib
    import traci
    import traci.constants as tc
    from traci._simulation import Stage
    from traci.exceptions import TraCIException
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


def get_options(cmd_args):
    """ Argument Parser. """
    parser = argparse.ArgumentParser(
        prog='generateEdgeToParkingAreaMapping.py', usage='%(prog)s -c configuration.json',
        description='generateEdgeToParkingAreaMapping.py')
    parser.add_argument(
        '-c', type=str, dest='config', default="../osm_activitygen.json", required=False,
        help='JSON configuration file.')
    parser.set_defaults(profiling=False)
    return parser.parse_args(cmd_args)


class ParkingAreaMapper():


    def _configure_loggers(self):
        """ Setup the console and file logger. """
        self.logger = logging.getLogger('ActivityGen')
        self.logger.setLevel(logging.DEBUG)
        _console_handler = logging.StreamHandler()
        _console_handler.setLevel(logging.INFO)
        _console_handler.setFormatter(
            logging.Formatter('[%(asctime)s] %(levelname)s: %(message)s'))
        self.logger.addHandler(_console_handler)
        _file_handler = logging.FileHandler('{}.debug.log'.format(self._conf['outputPrefix']))
        _file_handler.setLevel(logging.DEBUG)
        _file_handler.setFormatter(
            logging.Formatter(
                '[%(asctime)s]:[%(name)s]:[%(module)s:%(funcName)s:%(lineno)d]:[%(levelname)s] '
                '%(message)s'))
        self.logger.addHandler(_file_handler)


    def __init__(self, conf):
        """
        Initialize the synthetic population.
            :param conf: distionary with the configurations
        """
        self._conf = conf
        self._random_generator = RandomState(seed=self._conf['seed'])
        
        self._configure_loggers()

        self.logger.info('Starting TraCI with file %s.', conf['sumocfg'])
        traci.start(['sumo', '-c', conf['sumocfg']], traceFile='traci.log')

        self.logger.info('Loading SUMO net file %s', conf['SUMOnetFile'])
        self._sumo_network = sumolib.net.readNet(conf['SUMOnetFile'])

        self.logger.info('Loading SUMO parking lots from file %s', conf['SUMOadditionals']['parkings'])
        self._blacklisted_edges = set()
        self._sumo_parkings = collections.defaultdict(list)
        self._parking_edges = list()
        self._rtreeEdges = None
        self._parking_cache = dict()
        self._parking_position = dict()
        self._load_parkings(conf['SUMOadditionals']['parkings'])


    def _load_parkings(self, filename):
        """ Load parkings ids from XML file. """
        if not os.path.isfile(filename):
            return
        xml_tree = xml.etree.ElementTree.parse(filename).getroot()
        total_roadside_capacity = 0
        for child in xml_tree:
            if child.tag != 'parkingArea':
                continue
            if child.attrib['id'] not in self._conf['intermodalOptions']['parkingAreaBlacklist']:
                edge = child.attrib['lane'].split('_')[0]

                if 'startPos' in child.attrib:
                    position = float(child.attrib['startPos']) + 2.5
                else:
                    laneID = child.attrib['lane']
                    lane = self._sumo_network.getLane(laneID)
                    position = lane.getLength()

                if 'roadsideCapacity' in child.attrib:
                    total_roadside_capacity += float(child.attrib['roadsideCapacity'])

                if self._sumo_network.hasEdge(edge):
                    sumo_edge = self._sumo_network.getEdge(edge)
                    self._parking_edges.append(sumo_edge)
                    self._sumo_parkings[edge].append(child.attrib['id'])
                    self._parking_position[child.attrib['id']] = position
        self.logger.info('Loading parking areas have total capacity of %s', str(total_roadside_capacity))

    def _close_traci(self):
        """ Artefact to close TraCI properly. """
        self.logger.debug('Closing TraCI.')
        traci.close()


    def _dump_parking_data(self, parkings_by_edge_id):
        with open('osm_edge_parking_area_mapping.pkl', 'wb') as f:
            pickle.dump(parkings_by_edge_id, f)


    # From sumolib _initRTree
    def _init_rtree(self, shape_list, include_junctions=True):
        import rtree  # noqa
        result = rtree.index.Index()
        result.interleaved = True
        for ri, shape in enumerate(shape_list):
            result.add(ri, shape.getBoundingBox(include_junctions))
        return result


    # From sumolib getNeighboringParkingEdges
    def get_neighboring_parking_edges_from_xy(self, x, y, r=0.1, include_junctions=True):
        edges = []
        try:
            if self._rtreeEdges is None:
                self._rtreeEdges = self._init_rtree(self._parking_edges, include_junctions)
            for i in self._rtreeEdges.intersection((x - r, y - r, x + r, y + r)):
                e = self._parking_edges[i]
                d = sumolib.geomhelper.distancePointToPolygon((x, y), e.getShape(include_junctions))
                if d < r:
                    edges.append((e, d))
        except ImportError:
            sys.stderr.write("Error: Module 'rtree' not available.\n")
            sys.exit(1)
        return edges


    def get_neighboring_parking_edges_from_edge(self, sumo_edge, r=0.1, include_junctions=True):
        edge_bbox = sumo_edge.getBoundingBox(include_junctions)
        x = (edge_bbox[0] + edge_bbox[2]) / 2.0
        y = (edge_bbox[1] + edge_bbox[3]) / 2.0
        return self.get_neighboring_parking_edges_from_xy(x, y, r=r, include_junctions=include_junctions)


    def start_mapping(self, max_search_radius=1000):
        """ Calculate mapping of all net edges to closests parking areas. """
        
        parkings_by_edge_id = dict()
        skipped_edges = list()

        net_edges = self._sumo_network.getEdges()
        self.logger.info('Mapping %s edges to %s parking areas', str(len(net_edges)), str(len(self._sumo_parkings)))

        i = 0
        for net_edge in net_edges:
            if i % 500 == 0:
                self.logger.info('Mapped %s of %s edges', str(i), str(len(net_edges)))
            
            net_edge_id = net_edge.getID()
            radius = 50
            near_edge_dist_pairs = []
            parking_id = None
            
            while len(near_edge_dist_pairs) == 0 or not parking_id:
                near_edge_dist_pairs = self.get_neighboring_parking_edges_from_edge(net_edge, r=radius)

                for parking_edge, _ in near_edge_dist_pairs:
                    parking_edge_id = parking_edge.getID()
                    if parking_edge_id not in self._sumo_parkings:
                        continue

                    parking_ids_for_edge = self._sumo_parkings[parking_edge_id]
                    parking_id = parking_ids_for_edge[0] # TODO: handle multiple parking areas for same edge
                    if not parking_id:
                        continue

                    try:
                        route = traci.simulation.findIntermodalRoute(parking_edge_id, net_edge_id, pType="pedestrian")
                    except TraCIException:
                        parking_id = None
                        continue

                    parking = (parking_id, parking_edge_id, route)
                    self.logger.debug('Mapped parking area %s to edge %s', parking_id, net_edge_id)
                    parkings_by_edge_id[net_edge_id] = parking
                    break

                if radius >= max_search_radius:
                    # Stop searching if no parking is in max search radius
                    self.logger.error('Stopped searching for edge %s at radius %s', net_edge_id, str(radius))
                    parkings_by_edge_id[net_edge_id] = (None, None, None)
                    skipped_edges.append(net_edge_id)
                    break
                else:
                    radius += 50
            i += 1

        self.logger.info('Done. Mapping contains %s entries.', str(len(parkings_by_edge_id)))
        self.logger.info('Skipped %s edges in mapping cause max radius search failed.', str(len(skipped_edges)))
        self._dump_parking_data(parkings_by_edge_id)
        self._close_traci()


def main(cmd_args):
    args = get_options(cmd_args)

    print('Loading configuration file {}.'.format(args.config))
    mapper = ParkingAreaMapper(json.loads(open(args.config).read()))
    mapper.start_mapping(max_search_radius=2200)

    print('Done.')


if __name__ == "__main__":
    main(sys.argv[1:])
