#!/usr/bin/env python3
"""Validate CI YAML files beyond generic YAML parsing."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import yaml


RESERVED_GITLAB_TOP_LEVEL_KEYS = {
    "default",
    "include",
    "stages",
    "workflow",
    "variables",
    "cache",
    "image",
    "services",
    "before_script",
    "after_script",
}


def validate_script_value(value: Any, path: str, depth: int = 0) -> list[str]:
    if depth > 10:
        return [f"{path}: nested array is deeper than 10 levels"]

    if isinstance(value, str):
        return []

    if isinstance(value, list):
        failures: list[str] = []
        for index, item in enumerate(value):
            failures.extend(validate_script_value(item, f"{path}[{index}]", depth + 1))
        return failures

    return [f"{path}: expected string or nested array of strings, got {type(value).__name__}"]


def validate_gitlab_ci(path: Path, data: Any) -> list[str]:
    if not isinstance(data, dict):
        return [f"{path}: expected mapping at document root"]

    failures: list[str] = []
    for job_name, job_config in data.items():
        if job_name in RESERVED_GITLAB_TOP_LEVEL_KEYS or not isinstance(job_config, dict):
            continue

        for key in ("before_script", "script", "after_script"):
            if key in job_config:
                failures.extend(validate_script_value(job_config[key], f"{path}:{job_name}:{key}"))

    return failures


def main() -> int:
    workflow_paths = list(Path(".github/workflows").glob("*.yml")) + [Path(".gitlab-ci.yml")]
    failures: list[str] = []

    for workflow in workflow_paths:
        with workflow.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)

        if workflow.name == ".gitlab-ci.yml":
            failures.extend(validate_gitlab_ci(workflow, data))

        print(f"validated {workflow}")

    if failures:
        print("CI YAML validation failed:", file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
