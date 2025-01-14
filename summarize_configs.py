import importlib
import os
from typing import Annotated, Any

import typer


def main(part: str,
         key: Annotated[str | None, typer.Option(help='The key in the configuration to summarize.')] = None,
         detail: Annotated[str | None, typer.Option(help='The detail of this key in the configuration to summarize.')] = None):
    keys: set[str] = set()
    for root, _, files in os.walk('./config'):
        if '__pycache__' not in root:
            for file in files:
                if file == f'{part}.py':
                    module_name = 'config.' + root.split('/')[-1]
                    module = importlib.import_module(f'{module_name}.{part}', package='config')
                    config: dict[str, dict[str, Any]] = getattr(module, 'configuration', {})
                    if key is not None:
                        if isinstance(config.get(key, None), dict):
                            keys.update(config.get(key, {}).keys())
                        if detail is not None:
                            detail_config = config.get(key, {}).get(detail, None)
                            print(detail_config, ':', type(detail_config).__name__, ':', module_name)
                        else:
                            print(config.get(key, None), ':', module_name)
                    else:
                        keys.update(config.keys())
                        print(config, ':', module_name)
    print(sorted(keys))


if __name__ == '__main__':
    typer.run(main)
