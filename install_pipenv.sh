#!/bin/bash

pyenv install -v 3.7.8
pyenv global 3.7.8
pip install --upgrade pip
pip install pipenv
pipenv install
