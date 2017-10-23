import json
with open(rospy.get_param('../../resources/configurations/leslie.json'), 'r') as f:
    config = json.load(f)

"""
    Static strings in order to access the keys
"""
KEY_VERSION = "version"

def get_config(key):
    """
    Retrieve a specific configuration

    :Args:
        | key (String): An identifier for the specific requested configuration
    :Return:
        | (String): The configuration acording to the key from the JSON file.
     """
    return config[key]
