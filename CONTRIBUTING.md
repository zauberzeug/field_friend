# Contributing to Field Friend

We're thrilled that you're interested in contributing to Field Friend!
Here are some guidelines that will help you get started.

## Reporting issues

If you encounter a bug or other issue, the best way to report it is by opening a new issue on our [GitHub repository](https://github.com/zauberzeug/field_friend).
When creating the issue, please provide a clear and concise description of the problem, including any relevant error messages and code snippets.
If possible, include steps to reproduce the issue.

## Code of Conduct

We follow a [Code of Conduct](https://github.com/zauberzeug/field_friend/blob/main/CODE_OF_CONDUCT.md) to ensure that everyone who participates in the Field Friend community feels welcome and safe.
By participating, you agree to abide by its terms.

## Contributing code

We are excited that you want to contribute code to NiceGUI.
We're always looking for bug fixes, performance improvements, and new features.

### AI Assistant Integration

This project is designed to work well with AI assistants like Cursor, GitHub Copilot, and others.
The `.cursor/rules/` directory contains guidelines specifically for AI assistants that complement this contributing guide.

## Coding Style Guide

### Style Principles

- Always prefer simple solutions
- Avoid having files over 200-300 lines of code. Refactor at that point
- Use single quotes for strings in Python, double quotes in JavaScript
- Use f-strings wherever possible for better readability (except in performance-critical sections which should be marked with "NOTE:" comments)
- Follow autopep8 formatting with 120 character line length
- Use ruff for linting and code checks

### Workflow Guidelines

- Always simplify the implementation as much as possible:
  - Avoid duplication of code whenever possible, which means checking for other areas of the codebase that might already have similar code and functionality
  - Remove obsolete code
  - Ensure the code is not too complicated
  - Strive to have minimal maintenance burden and self explanatory code without the need of additional comments
- Be careful to only make changes that are requested or are well understood and related to the change being requested
- When fixing an issue or bug, do not introduce a new pattern or technology without first exhausting all options for the existing implementation. And if you finally do this, make sure to remove the old implementation afterwards so we don't have duplicate logic
- Keep the codebase very clean and organized
- Write tests for new features
- Run tests before submitting any changes
- Format code using autopep8 before submitting changes
- Use pre-commit hooks to ensure coding style compliance
- When adding new features, include corresponding tests

### Formatting

Because it has [numerous benefits](https://nick.groenen.me/notes/one-sentence-per-line/) in docs,
we write at least each sentence in a new line.

## Pull requests

To get started, fork the repository on GitHub, clone it somewhere on your filesystem, commit and push your changes,
and then open a pull request (PR) with a detailed description of the changes you've made
(the PR button is shown on the GitHub website of your forked repository).

When submitting a PR, please make sure that the code follows the existing coding style and that all tests are passing.
If you're adding a new feature, please include tests that cover the new functionality.

## Thank you!

Thank you for your interest in contributing to Field Friend!
We're looking forward to working with you!
