# Software-in-the-Loop Simulation for DRP Attack
- System requirements:
  - OS: Ubuntu 16.04, 18.04, 20.04
  - Python: 3.7+
  - GPU: a decent Nvidia GPU (e.g., GTX 1080) with Nvidia driver properly installed

## 0. Setup Python dependencies
- Download LGSVL PythonAPI version 2020.06
  - ``mkdir -p drp_attack_utils``
  - ``cd drp_attack_utils``
  - ``git clone https://github.com/lgsvl/PythonAPI.git``
  - ``cd PythonAPI``
  - ``git checkout tags/2020.06``
  - ``cd ../``
- Install ``pyenv``: ``./install_pyenv.sh``
- Log out and then log back in to enable ``pyenv``
- Install ``pipenv``: ``./install_pipenv.sh``
- Enable the Python environemnt: ``pipenv shell``

## 1. Setup LGSVL simulation environment

### Simulators
- Download our customized LGSVL simulators with dynamic patching capability
  - Local road simulator: ``https://drive.google.com/file/d/1kDRcetwCN78hbh80_KTRNSzB92U3zw4E/view?usp=sharing``, unzip and save the folder as ``drp_attack_utils/simulator-sl_local``
  - Highway simulator: ``https://drive.google.com/file/d/1fMukfe1bw4nwfaoc_WdGsr_hwJcD5mEx/view?usp=sharing``, unzip and save the folder as ``drp_attack_utils/simulator-sl_highway``
- Install vulkan: ``sudo apt install libvulkan1``
  - For more details, please refer: ``https://www.svlsimulator.com/docs/archive/2020.06/getting-started/#downloading-and-starting-simulator``
- Double click to execute one of the simulators: ``drp_attack_utils/simulator-sl_local``
- Click "Open Browser" and move to the next step
  - The browser may prompt for logging in. Since LGSVL currently has closed the account registration for the older simulator versions, please use our testing account instead: username: ``xlab4@bctf.com``, passwd: ``xlab@us``

### Setup map & vehicle resource bundles
- Local Road Map: set map name as **sl_local**
  - Map bundle path: ``{openpilot path}/drp_attack_utils/simulator-sl_local/AssetBundles/environment_SingleLaneRoad``
  - Note: make sure to use **full path**
- Highway Map: set map name as **sl_highway**
  - Map bundle path: ``{openpilot path}/drp_attack_utils/simulator-sl_highway/AssetBundles/environment_SingleLaneRoad``
  - Note: make sure to use **full path**
- Vehicle: set vehicle name as **Jaguar XE 2015**
  - Vehicle URL: ``https://assets.lgsvlsimulator.com/5797dab8650131e7b9518c5207d4515fe75b2cbf/vehicle_Jaguar2015XE``
  - Sensor conf: ``vehicle_conf.json``
- Close the simulator

## 2. Setup OpenPilot
- Assuming you are currently under ``openpilot`` folder
- Install necessary dependencies according to ``./Dockerfile.openpilot`` line 4--37
- Install pcapnp: ``sudo ./phonelibs/install_capnp.sh``
- Create a folder (required by OpenPilot): ``sudo mkdir /data; sudo chmod -R 777 /data``
- Place this line in ``~/.bashrc``: ``export PYTHONPATH={parent dir}/openpilot``
  - Note: make sure to replace the path with your local openpilot path
- ``. ~/.bashrc``
- Compile OpenPilot:
  - ``cd ./openpilot/cereal; make clean; make``
  - ``cd ../selfdrive/messaging/; make clean; make``
  - ``cd ../visiond/; make clean; make``

## 3. Simulation scenario configuration
- Copy config file for the scenario (i.e., local or highway)
  - For highway simulation: ``cp simconfig.ini.sl_highway simconfig.ini``
  - For local road simulation: ``cp simconfig.ini.sl_local simconfig.ini``
  - Note: make sure to replace the ``log_dir`` in the configuration files with your local paths
- Set to use the correct vanishing point coordinate based on the scenario
  - Checkout line 39--45 in ``./selfdrive/locationd/calibrationd.py``
- Enable the NPC truck if simulating the local road scenario
  - Checkout line 77--89 in ``selfdrive/controls/simcontrol.py``

## 4. Place your patch
- Place the patch image to the corresponding simulator folder based on your need
  - For local road simulation: ``cp {your_patch_name}.png ./drp_attack_utils/simulator-sl_local/simulator_Data/Resources/patch.png``
  - For highway simulation: ``cp {your_patch_name}.png ./drp_attack_utils/simulator-sl_highway/simulator_Data/Resources/patch.png``
  - Note: the above paths already contain the patch files used in our paper

## 5. Run simulation!
- Double click to execute the simulator: ``./drp_attack_utils/simulator-sl_{local or highway}/simulator``
- Click "Open Browser"
- Start the "Python API" simulation in the browser
- Enable the pipenv environemnt if you haven't: ``pipenv shell``
- Run the simulation: ``python ./start_sim.py``
  - It is possible that the openpilot will need to compile lateral/longitudinal controllers in your first run. Just be patient and wait for the compilation ends.
  - If necessary, kill the simulation after compilation finishes and re-run ``python ./start_sim.py``.
- After the simulation ends, you can generate simulation video using: ``ffmpeg -r 20 -i {your log dir}/frames/free_frame_%d.png -vcodec libx264 -pix_fmt yuv420p {your log dir}/sim_view.mp4 -y``
  - Note: make sure to use the correct log directory

## Troubleshooting
- Cannot load visiond module with error: "beignet-opencl-icd: no supported GPU found, this is probably the wrong opencl-icd package for this hardware (If you have multiple ICDs installed and OpenCL works, you can ignore this message) visiond: visiond.cc:251: void (anonymous namespace)::cl_init((anonymous namespace)::VisionState \*): Assertion `err == 0' failed."
  - Solution: Use the following command to install OpenCL if your GPU is from Nvidia: ``sudo apt install nvidia-opencl-icd ocl-icd-libopencl1 clinfo``
  - Note: May need to uninstall ``beignet-opencl-icd`` before executing the above: ``sudo apt remove beignet-opencl-icd``
