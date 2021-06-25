# How to run the lane detection experiment

### NOTE: make sure to run the correct simulator binary for each scenario!!!

1. Copy config file for the scenario
    - Highway: `cp simconfig.ini.sl_highway simconfig.ini`
    - Local: `cp simconfig.ini.sl_local simconfig.ini`
2. Edit `selfdrive/locationd/calibrationd.py` to use the correct vanishing point coordinate
    - Checkout line 39 & 43
3. Edit `selfdrive/controls/simcontrol.py` to disable or enable NPC on the other direction
    - Comment out lines 81 - 97
4. Edit `selfdrive/controls/simcontrol.py` to disable or enable NPC in the front
    - Be careful about the NPC distance & speed for the highway and local scenarios
