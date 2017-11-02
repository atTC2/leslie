import smtplib
import socket
import requests.packages.urllib3
from email.mime.application import MIMEApplication
from email.mime.text import MIMEText
from sys import stderr
from email.mime.multipart import MIMEMultipart
from util_modules import config_access
from os.path import basename

requests.packages.urllib3.disable_warnings()

EMAIL_ADDRESS = config_access.get_config(config_access.KEY_EMAIL_ADDRESS)
EMAIL_PASSWORD = config_access.get_config(config_access.KEY_EMAIL_PASSWORD)
EMAIL_SERVER = config_access.get_config(config_access.KEY_EMAIL_SERVER)
EMAIL_PORT = config_access.get_config(config_access.KEY_EMAIL_PORT)


def create_email(recipient_address, subject):
    """
    Creates an email object
    :param recipient_address: The address the email is to
    :param subject: The subject line of the email
    :type recipient_address: String
    :type subject: String
    :return: The email message object
    :rtype: MIMEMultipart
    """
    # Create the message
    msg = MIMEMultipart()
    msg['From'] = EMAIL_ADDRESS
    msg['To'] = recipient_address
    msg['Subject'] = subject
    return msg


def attach_body(msg, text_content):
    """
    Attaches supplied text as the body of the email
    :param msg: The email under construction
    :param text_content: The text to add as the body
    :type msg: MIMEMultipart
    :type text_content: String
    """
    msg.attach(MIMEText(text_content))


def attach_file(msg, file_path):
    """
    Attaches a file to the message
    :param msg: The message to attach the file to
    :param file_path: The path to the file
    :type msg: MIMEMultipart
    :type file_path: String
    :return: True if this was successful, false if there was a problem attaching the file
    :rtype: bool
    """
    try:
        with open(file_path, "rb") as video:
            part = MIMEApplication(
                video.read(),
                Name=basename(file_path)
            )
    except IOError:
        print >> stderr, 'Could not read file (please check permissions):\n', file_path
        return False

    # After the file is closed
    part['Content-Disposition'] = 'attachment; filename="%s"' % basename(file_path)
    msg.attach(part)

    return True


def send_email(recipient_address, msg):
    """
    Sends an email
    :param recipient_address: The address to send the message to
    :param msg: The message to send
    :type recipient_address: String
    :type msg: MIMEMultipart
    """
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
        print >> stderr, 'Failed to send email (refused)\n', e.smtp_error
    except socket.error, e:
        print >> stderr, 'Connection timeout (check SMTP configuration)', e.message
