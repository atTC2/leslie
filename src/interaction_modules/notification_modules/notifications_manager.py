import beep_module
import email_module
import twitter_module
from interaction_modules.user_info import get_user_info


def manage_notification(name):
    """
    Sends alarm notifications to the owner and plays out a message
    :param name: The identifier of the owner
    :type name: String
    """
    # Get someones info from json
    data = get_user_info(name)

    # Beep
    beep_module.notify()

    # Twitter
    if 'twitter' in data:
        twitter_module.notify(data['twitter'])
    # Email
    if 'email_address' in data:
        email_module.notify(name, data['email_address'])
