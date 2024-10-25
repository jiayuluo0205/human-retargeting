from pathlib import Path
from typing import Optional

def get_default_config_path(robot_name) -> Optional[Path]:
    config_path = Path(__file__).parent / "configs"
    config_name = f"{robot_name}.yml"
    return config_path / config_name