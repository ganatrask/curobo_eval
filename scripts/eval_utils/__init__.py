# cuRobo Evaluation Utilities

import os

from curobo.util_file import get_robot_configs_path, join_path, load_yaml

# Project root: two levels up from this file (scripts/eval_utils/ -> project)
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def load_robot_config(robot_file: str) -> dict:
    """Load robot config dict from file path or cuRobo's built-in configs.

    Resolution order for relative paths:
      1. Relative to PROJECT_DIR (e.g. "robots/curobo_configs_1/foo.yml")
      2. Relative to cuRobo's built-in robot_configs directory

    After loading, resolves urdf_path, collision_spheres, and asset_root_path
    relative to the YAML file's directory so configs stay portable.
    """
    if os.path.isabs(robot_file):
        cfg_dict = load_yaml(robot_file)
        yaml_dir = os.path.dirname(robot_file)
    elif os.path.exists(os.path.join(PROJECT_DIR, robot_file)):
        abs_path = os.path.join(PROJECT_DIR, robot_file)
        cfg_dict = load_yaml(abs_path)
        yaml_dir = os.path.dirname(abs_path)
    else:
        cfg_dict = load_yaml(join_path(get_robot_configs_path(), robot_file))
        yaml_dir = get_robot_configs_path()

    kin = cfg_dict.get("robot_cfg", {}).get("kinematics", {})
    for key in ("urdf_path", "collision_spheres", "asset_root_path"):
        val = kin.get(key)
        if val and not os.path.isabs(val):
            kin[key] = os.path.normpath(os.path.join(yaml_dir, val))

    return cfg_dict
