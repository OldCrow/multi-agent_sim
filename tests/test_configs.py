# runs a few tests on the config system, to make sure it doesn't break when we change it

# to run:
#   pytest tests/test_configs.py -v

# import stuff
import json
import sys
import os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from config.config import Config, load_config

# paths
ROOT           = os.path.join(os.path.dirname(__file__), '..')
CONFIG_PATH    = os.path.join(ROOT, 'config', 'config.json')
ALLOWABLE_PATH = os.path.join(ROOT, 'tests', 'allowable_configs.json')
TYPE_MAP = {"str": str, "int": int, "float": (int, float), "bool": bool}

# does it even load?
def test_load():
    config = Config(CONFIG_PATH)
    assert config is not None

# are the configs from the allowed set?
def test_allowable_configs():
    
    config      = Config(CONFIG_PATH)
    allowable   = json.loads(open(ALLOWABLE_PATH).read())

    # pull the allowable configs (attribute, and the rules for it)
    for attr, rules in allowable.items():

        # get that attribute from the config
        value = getattr(config, attr)

        # check the type
        expected_type = TYPE_MAP[rules["type"]]
        assert isinstance(value, expected_type), f"'{attr}' expected {rules['type']}, got {type(value).__name__}"
        
        # if it has other options, check those too
        if "options" in rules:
            assert value in rules["options"], f"'{attr}' value '{value}' is not in allowed options: {rules['options']}"

