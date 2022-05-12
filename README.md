# Kuka Ping Pong

This project implements a Kuka Ping Pong (or an approximation to it) making use of Pthon and CoppeliaSim. 

https://www.youtube.com/watch?v=JNsb-VydYu4

## Dependencies

- Python3.8
- pyARTE - https://github.com/4rtur1t0/pyARTE

Note that pyARTE is added in this repository as a submodule. It is recommended that you clone this repository with `--recurse-submodules` option.

## Repo Structure

- ./sceneconfig/ - Python package handling the interface between the project and CoppeliaSim
- ./scenes/ -  CoppeliaSim scene with the Robots, table and ball placed on it.
- ./src/ - Main code folder

## Usage

Setup python path to have all blocks available making use of `config.sh` script:
```sh
    $ source config.sh
```

## Feedback
