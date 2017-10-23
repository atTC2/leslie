import smtplib

# TODO: Move to configuration file
ROBOT_EMAIL_ADDRESS = 'leslietherobot.aka.pips@gmail.com'
ROBOT_PASSWORD = 'totallynottheactualthief'


def notify(data):
    server = smtplib.SMTP('smtp.gmail.com', 587)
    server.starttls()
    server.login(ROBOT_EMAIL_ADDRESS, ROBOT_PASSWORD)

    server.sendmail(ROBOT_EMAIL_ADDRESS,
                    data['email_address'],
                    data['email_text_content'])

    print ("Email sent to " + data['email_address'])

    server.quit()
