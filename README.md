<img src="https://github.com/zauberzeug/field_friend/raw/main/assets/field_friend.webp"  alt="Field Friend rendering" width="40%" align="right" />

# Zauberzeug Field Friend

This is the full source code of the [Field Friend](http://feldfreund.de) (aka Feldfreund) agricultural robot.
The software is based on [RoSys](https://rosys.io) and fully Open Source.
The hardware is build by [Zauberzeug](http://zauberzeug.com) and intended as a platform to advance organic and regenerative agriculture.

Please see the [documentation](https://docs.feldfreund.de) for details on installation, setup and usage.

## Setup

To use the Field Friend software, you'll need to have Python 3.12 and pip installed.
If this is not your current Python version, check out [pyenv](https://github.com/pyenv/pyenv).

If you want to separate the Field Friend environment from other projects, you can use a virtual environment.
Check out the [FastAPI tutorial](https://fastapi.tiangolo.com/virtual-environments/) on how to set one up.

Install the dependencies:

- `pip install -r requirements.txt` or
- `pip install -r requirements-dev.txt` when developing

### Formatting

We use [autopep8](https://github.com/hhatto/autopep8) with a 120 character line length to format our code.
Before submitting a pull request or to manually check your code, please run

```bash
autopep8 --max-line-length=120 --in-place --recursive .
```

Alternatively you can use VSCode, open the `field_friend.code-workspace` file and install the recommended extensions.
Then the formatting rules are applied whenever you save a file.

### Linting

We use [pre-commit](https://github.com/pre-commit/pre-commit) to make sure the coding style is enforced.
When installing dependencies with `requirements-dev.txt`, pre-commit is already installed.
To install the corresponding git commit hooks run: `pre-commiit install`.

After that you can make sure your code satisfies the coding style by running the following command:

```bash
pre-commit run --all-files
```

These checks will also run automatically before every commit:

- Run `ruff check . --fix` to check the code and sort imports.
- Remove trailing whitespace.
- Fix end of files.
- Enforce single quotes.

### Running tests

Our tests are built with pytest and pytest plugins that are installed with `requirements-dev.txt`.

Before submitting a pull request, please make sure that all tests are passing.
To run them all, use the following command in the root directory of Field Friend:

```bash
pytest
```
