import pyttsx   # sudo pip install pyttsx

engine = pyttsx.init()


def notify(data):
    """
    Say the message out loud and print it
    """
    beep_full_message = 'Hi {0}! {1}'.format(data['recipient'],
                                             data['beep_message'])
    engine.say('beep beep bloop blob')
    engine.say(beep_full_message)
    engine.runAndWait()
    print (beep_full_message)
