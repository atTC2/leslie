import smtplib

ROBOT_EMAIL_ADDRESS = 'leslietherobot.aka.pips@gmail.com'
ROBOT_PASSWORD = 'totallynottheactualthief'

def notify(recipient_email, recipient_message):
    server = smtplib.SMTP('smtp.gmail.com', 587)
    server.starttls()
    server.login(ROBOT_EMAIL_ADDRESS, ROBOT_PASSWORD)

    server.sendmail(ROBOT_EMAIL_ADDRESS, recipient_email, recipient_message)

    print ("Email sent to " + recipient_email)

    server.quit()
