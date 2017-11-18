from datetime import datetime
from tweepy import TweepError
from util_modules import twitter_api


def notify(handle):
    """
    Tweets the user to inform them their stuff has been stolen
    :param handle: Twitter handle (not with an @ prefix)
    :type handle: str
    """
    try:
        twitter_api.api.update_status('@' + handle + ' An alarm has been trigger at your table at ' +
                                      str(datetime.now()) + '!')
    except TweepError:
        print 'Unable to send tweet'
