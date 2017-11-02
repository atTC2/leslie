import tweepy
import requests.packages.urllib3
from datetime import datetime

requests.packages.urllib3.disable_warnings()

CONSUMER_KEY = 'VDbLbeG9KxlJ3M7uufFKe5gH5'
CONSUMER_SECRET = 'lTpOzWtV3y8L7R1iwDC0ylD1iIB59eQxlr9lYiXW4MlghjVqEr'
ACCESS_KEY = '922476500703678464-pd6ESkPDdX5oSlVuTU5pmoBjzPX1VZC'
ACCESS_SECRET = 'COrPlm4DOqRQmg2Hogj0nLCUtowvWMkbl4OYMvbuffGQ2'
auth = tweepy.OAuthHandler(CONSUMER_KEY, CONSUMER_SECRET)
auth.set_access_token(ACCESS_KEY, ACCESS_SECRET)
api = tweepy.API(auth)


def notify(handle):
    """
    Tweets @LeslieTheRobot
    :param handle: Twitter handle (not with an @ prefix)
    :type handle: String
    """
    api.update_status('@' + handle + ' An alarm has been trigger at your table at ' + str(datetime.now()) + '!')
