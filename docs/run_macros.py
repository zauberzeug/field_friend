import toml


def define_env(env):
    """Define variables for mkdocs-macros"""
    # Read pyproject.toml
    with open('pyproject.toml') as f:
        data = toml.load(f)

    # Make variables available to Markdown
    env.variables['python_versions'] = data['project']['requires-python']
