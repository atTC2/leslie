import pyttsx   # sudo pip install pyttsx

engine = pyttsx.init()


def notify():
    """
    Say the message out loud and print it
    """
    beep_full_message = 'ALARM!'
    engine.say('beep beep bloop blob')
    engine.say(beep_full_message)
    engine.runAndWait()
    print (beep_full_message)
