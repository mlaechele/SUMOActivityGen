from xml.etree import ElementTree


DEFAULT_SIDE_PARKING_XML = 'osm_parking.xml'

## generateParkingAreasFromOSM
DEFAULT_PARKING_AREAS = 'osm_parking_areas.add.xml'

## merged parking files
DEFAULT_COMPLETE_PARKING_XML = 'osm_complete_parking_areas.add.xml'

def _merge_parking_files(side_parking, parking_areas, complete_parking):
    """ Merge the two additional files containing parkings into one. """

    side_parking_struct = ElementTree.parse(side_parking).getroot()
    parking_areas_struct = ElementTree.parse(parking_areas).getroot()

    for element in parking_areas_struct:
        side_parking_struct.append(element)

    merged_parking = ElementTree.ElementTree(side_parking_struct)
    merged_parking.write(open(complete_parking, 'wb'))


if __name__ == '__main__':
    _merge_parking_files(DEFAULT_SIDE_PARKING_XML,DEFAULT_PARKING_AREAS,DEFAULT_COMPLETE_PARKING_XML)