import glob
import importlib
import os


def construct_config_path_simulation(module: str, robot_id: str):
    """Construct the path to the configuration file for the given module and robot_id. Used to simulate the given robot_id."""
    try:
        hostname = robot_id
        folder_path = find_matching_config_folders(hostname)
        return f'{folder_path}.{module}'
    except FileNotFoundError:
        print(f'Error: The file {folder_path} was not found. The config filename cant be constructed.')
        return None


def construct_config_path(module: str):
    """Construct the path to the configuration file for the given module. Used to run the code on the robot."""
    file_path = '/mnt/host_hostname'
    try:
        with open(file_path) as file:
            hostname = file.read().strip()
            folder_path = find_matching_config_folders(hostname)
            return f'{folder_path}.{module}'
    except FileNotFoundError:
        print(f'Error: The file {file_path} was not found. The config filename cant be constructed.')
        return None


def import_config_simulation(module: str, robot_id: str):
    """Import the configuration file for the given module and robot_id. Used to simulate the given robot_id."""
    attribute_name = 'configuration'
    path = construct_config_path_simulation(module, robot_id)
    imported_module = importlib.import_module(path)
    attribute = getattr(imported_module, attribute_name, None)
    if attribute is None:
        raise ImportError(f'Could not find {attribute_name} in {path}')
    return attribute


def import_config(module: str) -> dict:
    """Import the configuration file for the given module. Used to run the code on the robot."""
    attribute_name = 'configuration'
    path = construct_config_path(module)
    imported_module = importlib.import_module(path)
    attribute = getattr(imported_module, attribute_name, None)
    if attribute is None:
        raise ImportError(f'Could not find {attribute_name} in {path}')
    return attribute


def find_matching_config_folders(hostname):
    """Find the folder that contains the configuration files for the given hostname."""
    # Construct the pattern to match against
    pattern = f'config/*_config_{hostname}'
    # List all paths in the top folder that match the pattern
    matching_folders = glob.glob(pattern)
    # Filter out the ones that are not directories
    matching_dirs = [folder for folder in matching_folders if os.path.isdir(folder)]
    assert len(matching_dirs) == 1, \
        f'Expected exactly one matching folder, but got {len(matching_dirs)} for {pattern}: {matching_dirs}'
    return matching_dirs[0].replace('/', '.')
