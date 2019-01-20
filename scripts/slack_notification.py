# import requests
# import getpass

# URL_PATH = '/home/aifs2/group/bin/slack_notification_url.txt'


def read_url():
    pass
    # with open(URL_PATH) as url:
    #     return url.readline()


def start_experiment_notification(experiment_count=None, machine=None):
    pass
    # user = getpass.getuser().capitalize()
    # if experiment_count is None:
    #     message = user + ' just started running experiments.'
    # else:
    #     if machine is None:
    #         message = '{user} just started running {experiment_count} experiments.'.format(user=user, experiment_count=experiment_count)
    #     else:
    #         message = '{user} just started running {experiment_count} experiments on {machine}'.format(user=user, experiment_count=experiment_count, machine=machine)
    # 
    # send_notification(message)


def end_experiment_notification():
    pass
    # user = getpass.getuser().capitalize()
    # message = user + "'s experiments just finished."
    # 
    # send_notification(message)


def update_experiment_status(total_experiment_count, completed_experiment_count, remaining_minutes):
    pass


def send_notification(text, user='experiment_bot', channel='#experiments'):
    pass
    # data = {'channel': channel,
    #         'username': user,
    #         'text': text,
    #         'icon_emoji': ':mailbox_with_mail:'}
    # 
    # url = read_url()
    # 
    # requests.post(url=url, json=data, headers={'Content-Type': 'application/json'})


# # Test
# 
# if __name__ == '__main__':
#     start_experiment_notification(1)
#     end_experiment_notification()
