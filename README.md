# SaRA-shield
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This package provides safety for human-robot interaction using reachability analysis.
We use [SaRA](https://github.com/Sven-Schepp/SaRA) to calculate the reachable sets of humans and robots.
The SaRA shield additionally provides the necessary trajectory control to stop the robot before any collision with the human could occur.

# Installation
### Clone the repo with submodules
```
git clone --recurse-submodules git@github.com:JakobThumm/sara-shield.git
```
### Install the shield [C++ only]
```
cd safety_shield
mkdir build && cd build
cmake ..
make -j 4
```
### Install the shield [With Python bindings]
```
pip install -r requirements.txt
python setup.py install
```
### Run the python binding tests
```
pytest safety_shield/tests
```