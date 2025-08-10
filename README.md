# AILAB_ADAS_Sensor_SW
Practice of sensors and ADAS software created by Automotive Intelligence Lab at Hanyang University.  
This repository contains tutorial code and supplementary note for Autonomous driving technology, sensors, and ADAS software.

## Repository structure
```bash
┌─ data                # files for examples
└─ practice            # python codes of tutorial
    ├─ tutlibs         # packages for tutorial codes
    |    └─ {package}  # package for tutorial codes
    └─ *.ipynb         # tutorial codes
```

## How to use
### 1. Environment Setup

#### Basic Environment:

```bash
conda create -n adas_practice python=3.11
conda activate adas_practice
pip install ipykernel
pip install numpy matplotlib
pip install rospkg rosbag
pip install pycryptodomex
pip install gnupg
pip install pandas
pip install folium
pip install opencv-python
pip install scipy
pip install open3d
```

## Authors

- **Jaehwan Lee** - idljh5529@gmail.com