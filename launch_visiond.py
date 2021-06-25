#!/bin/bash

export PASSIVE="0"

function launch {
    cd selfdrive/visiond
    ./visiond
}

launch
