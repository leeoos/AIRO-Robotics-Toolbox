#! /bin/bash

export ROB2LIB_PATH=$(pwd)"/lib"

# starting matlab inside src dir
# insert desired opening dir here
setsid matlab -sd $(pwd) -desktop -softwareopengl

