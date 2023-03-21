# Repository of Esmacat Master High Performance

This repository is to store/manage the software of Esmacat Master software that runs on Esmacat Master Hardware

## How to clone the repository

Since this repository includes submodules, the repository can be cloned by --recurse-submodules command as below:

_git clone --recurse-submodules  [https://bitbucket.org/harmonicbionics/esmacat_master_high_performance](https://bitbucket.org/harmonicbionics/esmacat_master_high_performance)_


## Folder Structure
 - esmacat_core: core source and header files of Esmacat master (git submodule)
 - esmacat_applications: example and template applications using esmacat_core (git submodule)
 - esmacat_slave_driver: driver files to use esmacat slaves (git submodule)
- resource: third party software
- configuration: configuration files for different environments
- SOEM: EtherCAT master stack from Simple Opensource EtherCAT Master (git submodule)
- doc: Doxygen documentation

## Build

mkdir build

cd build

cmake ..

make

## Run
cd esmacat_applications

cd ${application_name_you_want_to_execute}

./run.sh
