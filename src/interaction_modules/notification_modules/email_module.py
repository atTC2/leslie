from datetime import datetime
from interaction_modules.email_util import create_email, send_email, attach_body


def notify(notify_owner, recipient_address):
    """
    Sends a notification email of a triggered alarm
    :param notify_owner: The name of whom the report is being sent to
    :param recipient_address: The email address of the recipient
    :type notify_owner: String
    :type recipient_address: String
    """

    # Create the message
    msg = create_email(recipient_address, 'PIPS Alarm @ ' + str(datetime.now()))

    # Attach the text
    text_content = 'Hello ' + notify_owner + '.\r\n\r\n'
    text_content += 'An alarm has been triggered at your table at ' + str(datetime.now()) + '.\r\n\r\n'
    text_content += 'Kind regards,\r\nLeslie (Personal Item Protection System)'
    attach_body(msg, text_content)

    # Send the email
    send_email(recipient_address, msg)

