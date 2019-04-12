## Installation
There are some packages that are required. To install with `pip` do the following.
``` shell
pip install matplotlib numpy scipy pyinterval networkx
```

## Installation (alternative)
Alternatively, if you use macOS and have yet had any experiences with `pip`, `conda` is a better choice. 
First install conda on your machine from [this](https://www.anaconda.com/download/). 
Then, create an environment, activate it and run the following commands.
``` bash
# Create conda environment
conda create --name atm
# Activate
source activate atm 
# Install packages
conda install matplotlib numpy scipy networkx
pip install pyinterval
# Deactivate the environment
source deactive atm
```

## Simulation on Changi
To run a simple simulation on Changi Airport map:
```
cd ~/git/ATM/code/tests
python testrrt2.py
```
