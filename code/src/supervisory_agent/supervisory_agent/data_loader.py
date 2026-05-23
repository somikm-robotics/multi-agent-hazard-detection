import os
import json
from ament_index_python.packages import get_package_share_directory

_SUPERVISORY_PKG = get_package_share_directory('supervisory_agent')
_JSON_PATH = os.path.join(_SUPERVISORY_PKG, 'config', 'mine_locations.json')

def _get_locations_dict():
    with open(_JSON_PATH, 'r') as f:
        loc_data = json.load(f)
    return loc_data["Location"]

def get_location_names():
    """Return a list of location names from the locations JSON."""
    return list(_get_locations_dict().keys())

def get_location_coordinates(location_name):
    """Return the coordinates dict (with x, y) for the given location name."""
    return _get_locations_dict().get(location_name)
