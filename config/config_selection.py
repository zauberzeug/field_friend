import glob
import importlib
import os


def construct_config_path(module: str):
    file_path = '/mnt/host_hostname'
    try:
        with open(file_path, 'r') as file:
            hostname = file.read().strip()
            folder_path = find_matching_config_folders(hostname)
            return f"{folder_path}.{module}"
    except FileNotFoundError:
        print(f"Error: The file {file_path} was not found. The config filename cant be constructed.")
        return None


def import_config(module: str) -> dict:
    attribute_name = "configuration"
    path = construct_config_path(module)
    module = importlib.import_module(path)
    attribute = getattr(module, attribute_name, None)
    if attribute is None:
        raise ImportError(f"Could not find {attribute_name} in {path}")
    return attribute


def find_matching_config_folders(hostname):
    # Construct the pattern to match against
    pattern = f"config/*_config_{hostname}"
    # List all paths in the top folder that match the pattern
    matching_folders = glob.glob(pattern)
    # Filter out the ones that are not directories
    matching_dirs = [folder for folder in matching_folders if os.path.isdir(folder)]
    return matching_dirs[0].replace("/", ".")
