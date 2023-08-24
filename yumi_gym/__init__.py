from gym.envs.registration import register

# Yumi
register(
    id='yumi-inspire',
    entry_point='yumi_gym.envs:YumiInspireEnv')

register(
    id='yumi-schunk',
    entry_point='yumi_gym.envs:YumiSchunkEnv')