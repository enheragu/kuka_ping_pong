# Kuka Ping Pong

This project implements a Kuka Ping Pong (or an approximation to it) making use of Pthon and CoppeliaSim. 
Note that is a prototype and code is not at it's best...it's research code :)

## Dependencies

- Python3.8
- pyARTE - https://github.com/4rtur1t0/pyARTE

Note that pyARTE is added in this repository as a submodule. It is recommended that you clone this repository with `--recurse-submodules` option.

## Repo Structure

- **./sceneconfig/** - Python package handling the interface between the project and CoppeliaSim
- **./scenes/** -  CoppeliaSim scene with the Robots, table and ball placed on it.
- **./src/** - Main code folder
- **config.sh** - Script to load different directories into PYTHONPATH

## Usage

Setup python path to have all blocks available making use of `config.sh` script:
```sh
    $ source config.sh
```

CoppeliaSim has to be running. Please follow it's instructions to execute it and load scneario located in `scenes/pingpong_scene.ttt`.

From the root folder of the repository the main script can be launched. If you added execution permission to it you can run it just with `./src/main.py`, if not execute it making use of `python3` interpreter.

## Feedback
