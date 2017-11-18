import json
import os

config_file_path = os.path.join(os.path.dirname(__file__), '../../resources/people.json')

with open(config_file_path, 'r') as f:
    config = json.load(f)


def get_user_info(key):
    """
    Retrieves information for a user
    :param key: An identifier for a person
    :type key: str
    :return: The person's information
    :rtype: dict[String, String]
    """
    return config[key]


def is_user_in_system(key):
    """
    Checks to see if a user is in people.json
    :param key: The user name to check
    :type: str
    :return: If the user is in people.json
    :rtype: bool
    """
    return key in config


def get_user_from_twitter_handle(twitter_handle):
    """
    Retrieves a user's name from their twitter handle, given the twitter handle is in the people's file
    :param twitter_handle: The twitter handle
    :type twitter_handle: str
    :return: The name of the user, or None if their twitter handle is not in the people's file
    :rtype: str or None
    """
    for key, values in config.iteritems():
        if 'twitter' in values and values['twitter'].lower() == twitter_handle.lower():
            # Found the owner of the twitter handle, return their name
            return key

    # No one found
    return None
