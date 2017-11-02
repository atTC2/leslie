from datetime import datetime
from interaction_modules.email_util import create_email, send_email, attach_body, attach_file
from util_modules.people_info import get_user_info
from os.path import getctime
from sys import stderr


def send_report_email(name, file_path):
    """
    Sends a report email for an incident, including the video recording
    :param name: The name of whom the report is being sent to
    :param file_path: The path to the video file
    :type name: String
    :type file_path: String
    """
    # Check if they have an email address
    user_info = get_user_info(name)
    if 'email_address' not in user_info:
        # No email address
        return

    recipient_address = user_info['email_address']

    # TODO ask the user if they want the email report

    # Get the incident time
    try:
        incident_time = str(datetime.fromtimestamp(getctime(file_path)))
    except OSError:
        print >> stderr, 'File not available:\n', file_path
        return

    # Create the message
    msg = create_email(recipient_address, 'PIPS Incident @ ' + incident_time)

    # Attach the text
    text_content = 'Hello ' + name + '.\r\n\r\n'
    text_content += 'Attached is a video recording of a thief detected at ' + incident_time + '.\r\n\r\n'
    text_content += 'Kind regards,\r\nLeslie (Personal Item Protection System)'
    attach_body(msg, text_content)

    # Attached the video
    if not attach_file(msg, file_path):
        # Attaching the video failed (and was logged)
        return

    # Send the email
    send_email(recipient_address, msg)
