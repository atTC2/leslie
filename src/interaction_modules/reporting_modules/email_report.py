import smtplib
import socket
import requests.packages.urllib3
from datetime import datetime

from interaction_modules.user_info import get_user_info
from util_modules import config_access
from email.mime.application import MIMEApplication
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from os.path import basename, getctime
from sys import stderr

requests.packages.urllib3.disable_warnings()

EMAIL_ADDRESS = config_access.get_config(config_access.KEY_EMAIL_ADDRESS)
EMAIL_PASSWORD = config_access.get_config(config_access.KEY_EMAIL_PASSWORD)
EMAIL_SERVER = config_access.get_config(config_access.KEY_EMAIL_SERVER)
EMAIL_PORT = config_access.get_config(config_access.KEY_EMAIL_PORT)


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

    recipient_address = ['email_address']

    # TODO ask the user if they want the email report

    # Get the incident time
    try:
        incident_time = str(datetime.fromtimestamp(getctime(file_path)))
    except OSError:
        print >> stderr, 'File not available:\n', file_path
        return

    # Create the message
    msg = MIMEMultipart()
    msg['From'] = EMAIL_ADDRESS
    msg['To'] = recipient_address
    msg['Subject'] = 'PIPS Incident @ ' + incident_time

    # Attach the text
    text_content = 'Hello ' + name + '.\r\n\r\n'
    text_content += 'Attached is a video recording of a thief detected at ' + incident_time + '.\r\n\r\n'
    text_content += 'Kind regards,\r\nLeslie (Personal Item Protection System)'
    msg.attach(MIMEText(text_content))

    # Attached the video
    try:
        with open(file_path, "rb") as video:
            part = MIMEApplication(
                video.read(),
                Name=basename(file_path)
            )
    except IOError:
        print >> stderr, 'Could not read file (please check permissions):\n', file_path
        return

    # After the file is closed
    part['Content-Disposition'] = 'attachment; filename="%s"' % basename(file_path)
    msg.attach(part)

    # Send the email
    try:
        server = smtplib.SMTP(EMAIL_SERVER, EMAIL_PORT)
        server.starttls()
        server.login(EMAIL_ADDRESS, EMAIL_PASSWORD)
        server.sendmail(EMAIL_ADDRESS, recipient_address, msg.as_string())

        server.quit()
    except socket.gaierror:
        print >> stderr, 'Failed to send email (please check internet connection and SMTP configuration)'
    except smtplib.SMTPAuthenticationError:
        print >> stderr, 'Could not authenticate with SMTP server (please check credentials)'
    except smtplib.SMTPSenderRefused, e:
        print >> stderr, 'Failed to send email (refused):\n', file_path, '\n', e.smtp_error
    except socket.error:
        print >> stderr, 'Connection timeout (check SMTP configuration)'
