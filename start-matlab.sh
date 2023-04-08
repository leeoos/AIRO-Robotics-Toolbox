#! /bin/bash

export ROB2LIB_PATH=$(pwd)"/lib"

# starting matlab inside src dir
setsid matlab -sd $(pwd)/src -desktop
