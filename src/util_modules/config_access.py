import json
import os

config_file_path = os.path.join(os.path.dirname(__file__),
                                '../../resources/configurations/leslie.json')

with open(config_file_path, 'r') as f:
    config = json.load(f)

"""
    Static strings in order to access the keys
"""
KEY_VERSION = "version"


def get_config(key):
    """
    Retrieve a specific configuration
    :param key: An identifier for the specific requested configuration
    :type key: String
    :return: The configuration according to the key from the JSON file.
    :rtype: String
    """
    return config[key]
