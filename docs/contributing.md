# Contributing

We're thrilled that you're interested in contributing to the Field Friend source code!
Here are some guidelines that will help you get started.

## Reporting issues

If you encounter a bug or other issue, the best way to report it is by opening a new issue on our [GitHub repository](https://github.com/zauberzeug/field_friend).
When creating the issue, please provide a clear and concise description of the problem, including any relevant error messages and code snippets.
If possible, include steps to reproduce the issue.

## Code of Conduct

We follow a [Code of Conduct](https://github.com/zauberzeug/field_friend/blob/main/CODE_OF_CONDUCT.md) to ensure that everyone who participates in the NiceGUI community feels welcome and safe.
By participating, you agree to abide by its terms.

## Contributing code

We are excited that you want to contribute code to the Field Friend source code.
We're always looking for bug fixes, performance improvements, and new features.

## Coding Style Guide

### Formatting

We use [autopep8](https://github.com/hhatto/autopep8) with a 120 character line length to format our code.
Before submitting a pull request, please run

```bash
autopep8 --max-line-length=120 --in-place --recursive .
```

on your code to ensure that it meets our formatting guidelines.
Alternatively you can use VSCode, open the field_friend.code-workspace file and install the recommended extensions.
Then the formatting rules are applied whenever you save a file.

In our point of view, the Black formatter is sometimes a bit too strict.
There are cases where one or the other arrangement of, e.g., function arguments is more readable than the other.
Then we like the flexibility to either put all arguments on separate lines or only put the lengthy event handler
on a second line and leave the other arguments as they are.

### Imports

We use [ruff](https://docs.astral.sh/ruff/) to automatically sort imports:

```bash
ruff check . --fix
```

### Single vs Double Quotes

Regarding single or double quotes: [PEP 8](https://peps.python.org/pep-0008/) doesn't give any recommendation, so we simply chose single quotes and sticked with it.
On qwerty keyboards it's a bit easier to type, is visually less cluttered, and it works well for strings containing double quotes from the English language.

### F-Strings

We use f-strings where ever possible because they are generally more readable - once you get used to them.
There are only a few places in the code base where performance really matters and f-strings might not be the best choice.
These places should be marked with a `# NOTE: ...` comment when diverging from f-string usage.

## Documentation

### Formatting

Because it has [numerous benefits](https://nick.groenen.me/notes/one-sentence-per-line/) we write each sentence in a new line.

### Examples

Each example should be about one concept.
Please try to make them as minimal as possible to show what is needed to get some kind of functionality.
We are happy to merge pull requests with new examples which show new concepts, ideas or interesting use cases.

## Pull requests

To get started, fork the repository on GitHub, clone it somewhere on your filesystem, commit and push your changes,
and then open a pull request (PR) with a detailed description of the changes you've made
(the PR button is shown on the GitHub website of your forked repository).

When submitting a PR, please make sure that the code follows the existing coding style and that all tests are passing.
If you're adding a new feature, please include tests that cover the new functionality.

## Thank you!

Thank you for your interest in contributing.
We're looking forward to work with you!
