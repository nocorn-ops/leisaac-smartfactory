import gymnasium as gym

gym.register(
    id="LeIsaac-JXB-LiftCube-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.jxb_test_env_cfg:JxbLiftCubeEnvCfg",
    },
)
