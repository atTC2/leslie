import json
import os

config_file_path = os.path.join(os.path.dirname(__file__), '../../resources/people.json')

with open(config_file_path, 'r') as f:
    config = json.load(f)


def get_user_info(key):
    """
    Retrieves information for a user
    :param key: An identifier for a person
    :type key: String
    :return: The person's information
    :rtype: dict[String, String]
    """
    return config[key]
