# MEEN 612 Project

`project.pdf` is the project report.  

The manipulator PyBullet simulation and control is located in `project.py`.
Running this script runs all unit tests such as simulation flopping, position regulation control, impulse response, and
trajectory following control.  

The manipulator URDF and meshes are located in `ManipulatorUrdf`, which was created using a SolidWorks URDF exporter
add-in.  

## Running PyBullet Simulation  
1. Create Python virtual environment [optional]: `virtualenv venv`  
2. Activate virtual environment [optional]: `source venv/bin/activate`  
3. Install dependencies: `pip install -r requirements.txt`  
4. Run unit tests: `./project.py`  

## Running MATLAB Simulation Flop  
1. Start MATLAB and add directory to path.  
2. Run `sim_flopping.m` script.  
