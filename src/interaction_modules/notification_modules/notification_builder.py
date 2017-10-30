class NotificationBuilder(object):

    def __init__(self, recipient):
        self.data = {}
        self.data['send_beep'] = False
        self.data['send_email'] = False
        self.data['send_tweet'] = False
        self.data['recipient'] = recipient

    def send_beep(self, beep_message):
        self.data['send_beep'] = True
        self.data['beep_message'] = beep_message

    def send_email(self, email_address, email_text_content):
        self.data['send_email'] = True
        self.data['email_address'] = email_address
        self.data['email_text_content'] = email_text_content

    def send_tweet(self, tweet_message):
        self.data['send_tweet'] = True
        self.data['tweet_message'] = tweet_message
