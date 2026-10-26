from pathlib import Path

import yaml

from robot_patrol.inspection_prompts import DIGITAL_METER_PROMPT


def test_deployed_numeric_meter_prompt_matches_node_default():
    config=Path(__file__).resolve().parents[1]/'config/patrol.yaml'
    parameters=yaml.safe_load(config.read_text())['robot_patrol']['ros__parameters']
    assert parameters['inspection_default_prompt'].strip()==DIGITAL_METER_PROMPT
