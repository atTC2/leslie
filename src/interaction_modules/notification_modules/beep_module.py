import pyttsx # sudo pip install pyttsx

engine = pyttsx.init()

def notify(recipient, message):
    beep_message = 'Hi {0}! {1}'.format(recipient, message)
    engine.say('beep beep bloop blob')
    engine.say(beep_message)
    engine.runAndWait()
    print (beep_message)
