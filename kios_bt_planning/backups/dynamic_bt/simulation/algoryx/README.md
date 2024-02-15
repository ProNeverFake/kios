# Robotics Digital Lab

This package contains a `python` implementation of a simulation environment based on Algoryx.

At the current state only the lowes fidelity level of the simulation is implemented. It features a gripper that teleports above the items to pick without simulating friction or contacts. There is no slippage.


## Installation

1. [Install AGX Dynamics 2.34.0.2](https://www.algoryx.se/documentation/complete/agx/tags/latest/UserManual/source/installation.html#) for your platform. See your license email for download instructions.
2. Install the `agxBrick` and `agxClick` python packages using `pip install -r requirements.txt`
  * Some bug with `--extra-index-url` in `requirements.txt` might mean that you need to install them separate
  * `pip install --extra-index-url https://agx-access:UymNbuiLHJ13CKGnmsoD@git.algoryx.se/api/v4/projects/270/packages/pypi/simple agxBrick==0.5.16`
  * `pip install --extra-index-url https://click-access:F2q7LauW_d-HJ7bH37sV@git.algoryx.se/api/v4/projects/262/packages/pypi/simple agxClick==0.1.34`
3. Install OpenGL dependency with `pip install PyOpenGL-accelerate`
4. Install dotnet dependencies:
```
wget -q https://packages.microsoft.com/config/ubuntu/20.04/packages-microsoft-prod.deb -O packages-microsoft-prod.deb
dpkg -i packages-microsoft-prod.deb
add-apt-repository universe
apt-get install apt-transport-https
apt-get update
apt-get install -qy dotnet-sdk-3.1 dotnet-sdk-5.0 dotnet-sdk-6.0
```
5. Add these lines to the `.bashrc` file, where version is e.g 2.34.0.1:
  * `source /opt/Algoryx/AGX-version/setup_env.bash`
  * `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/opt/Algoryx/AGX-version/lib"`


## Run the BT Demo
To test the installation you need to source the AGX environment. See the user manual for instruction for your platform.

You can run the BT Demo by running the demo script.
```
python run_demo.py --model models/BTDemo.yml:CoarseGripperInLabDR --decorate --timeStep 0.05
```

Note that this command works only from the folder where the `run_demo.py` script is stored.

