import pyttsx

engine = pyttsx.init()


def say(text):
    """
    Say some words
    :param text: The words to say
    :type text: str
    """
    engine.say(text)
    engine.say("   ")
    engine.runAndWait()
