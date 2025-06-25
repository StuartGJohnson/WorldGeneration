# WorldGeneration

WorldGeneration exists to provide world models for robotics development with multiple simulators. Currently supported simulators are:

- Gazebo Garden
- NVIDIA IsaacSim 4.5

While generating SDF files for Gazebo can be done entirely with XML, I use the Nvidia USD library to compose all USD(A) world files.

## Requirements

This code requires a USD python library. I used the prebuilt NVIDIA USD library, installed via:

[nvidia usd] (https://developer.nvidia.com/usd?sortBy=developer_learning_library%2Fsort%2Ffeatured_in.usd_resources%3Adesc%2Ctitle%3Aasc&hitsPerPage=6)

You can also install usd via pypi (see the link). Searching the nvidia developer pages for up-to-date options is advisable.

After installing USD in the binary manner, I used PYTHONPATH to extend my python interpreter so it could use the installed version of USD. See the file `env.sh`. I also use a conda env for this repo - based on python 3.11.

## Usage

Currently, all code is run from the unit test: `test_gpy.py`. This will evolve.

### World generation

TODO

