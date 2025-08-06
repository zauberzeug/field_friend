import re


def define_env(env):
    # Extract Python version from the Dockerfile's first line
    with open('Dockerfile') as f:
        first_line = f.readline().strip()
    match = re.search(r'FROM python:(\d+\.\d+)', first_line)
    python_version = match.group(1)
    env.variables['python_version'] = python_version
