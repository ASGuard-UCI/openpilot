#!/bin/bash

function launch {
  export PYTHONPATH="$PWD"
  ./lgsvl-sim/lgsvl-camerad.py
}

launch
