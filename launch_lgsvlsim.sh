#!/bin/bash

export PASSIVE="0"

function launch {
    export PYTHONPATH="$PWD"
    cd selfdrive
    python3 ./controls/lgsvlsim.py
}

launch
