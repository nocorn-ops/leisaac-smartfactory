import gymnasium as gym

gym.register(
    id="LeIsaac-LeRobot-Kitchen-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.lerobot_kitchen_env_cfg:LeRobotKitchenEnvCfg",
    },
)
