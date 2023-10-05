#!/bin/bash

# setup virtual environment
python3 -m venv venv
source venv/bin/activate

# install pre-commit
pip install pre-commit==3.4.0
pre-commit install
