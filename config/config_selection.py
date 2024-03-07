import importlib


def construct_config_path(file_path='/mnt/host_hostname'):
    try:
        with open(file_path, 'r') as file:
            hostname = file.read().strip()
            return f"config.config_{hostname}"
    except FileNotFoundError:
        print(f"Error: The file {file_path} was not found. The config filename cant be constructed.")
        return None


def import_config() -> dict[str, dict]:
    attribute_name = "fieldfriend_configuration"
    path = construct_config_path()
    module = importlib.import_module(path)
    attribute = getattr(module, attribute_name, None)
    if attribute is None:
        raise ImportError(f"Could not find {attribute_name} in {path}")
    return attribute
