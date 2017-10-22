from textmagic.rest import TextmagicRestClient #sudo pip install textmagic

ROBOT_TEXTMAGIC_USERNAME = ''
ROBOT_TEXTMAGIC_TOKEN = ''

def notify(number, message):
        client = TextmagicRestClient(username, token)
        message = client.messages.create(phones=numner, text=message)
