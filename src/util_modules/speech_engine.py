import os
import subprocess

script_path = os.path.join(os.path.dirname(__file__), '../../resources/scripts/pico_say.sh')


def say(text, tempo=1.17):
    """
    Say some words
    :param text: The words to say
    :param tempo: The speed of playback
    :type text: str
    :type tempo: float
    """
    subprocess.call(['bash', script_path, str(tempo), text])
