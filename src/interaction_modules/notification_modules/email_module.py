import smtplib
import socket
import requests.packages.urllib3
from datetime import datetime
from util_modules import config_access
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from sys import stderr

requests.packages.urllib3.disable_warnings()

EMAIL_ADDRESS = config_access.get_config(config_access.KEY_EMAIL_ADDRESS)
EMAIL_PASSWORD = config_access.get_config(config_access.KEY_EMAIL_PASSWORD)
EMAIL_SERVER = config_access.get_config(config_access.KEY_EMAIL_SERVER)
EMAIL_PORT = config_access.get_config(config_access.KEY_EMAIL_PORT)


def notify(name, recipient_address):
    """
    Sends a notification email of a triggered alarm
    :param name: The name of whom the report is being sent to
    :param recipient_address: The email address of the recipient
    :type name: String
    :type recipient_address: String
    """

    # Create the message
    msg = MIMEMultipart()
    msg['From'] = EMAIL_ADDRESS
    msg['To'] = recipient_address
    msg['Subject'] = 'PIPS Alarm @ ' + str(datetime.now())

    # Attach the text
    text_content = 'Hello ' + name + '.\r\n\r\n'
    text_content += 'An alarm has been triggered at your table at ' + str(datetime.now()) + '.\r\n\r\n'
    text_content += 'Kind regards,\r\nLeslie (Personal Item Protection System)'
    msg.attach(MIMEText(text_content))

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
        print >> stderr, 'Failed to send email (refused):\n', e.smtp_error
    except socket.error:
        print >> stderr, 'Connection timeout (check SMTP configuration)'
