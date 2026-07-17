"""Loader for the shared GR6-v2 config file (see ../config.yaml)."""

from pathlib import Path

import yaml

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"


def load_config(path: Path = CONFIG_PATH) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def service_config(name: str, path: Path = CONFIG_PATH) -> dict:
    return load_config(path)["services"][name]
