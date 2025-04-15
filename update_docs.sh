#!/bin/bash
# Script to update documentation with information from pyproject.toml

# Install required packages if not already installed
pip install -r docs_requirements.txt

# Run the update script
python update_docs.py

echo "Documentation update complete!"
