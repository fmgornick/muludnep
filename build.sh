#!/bin/sh

cd "$(dirname "$0")"

clang++ -std=c++17 -O3 -Wall \
    -Flibs/mujoco \
    -Iinc \
    -Llib_darwin \
    -Wl,-rpath,$dir/libs/mujoco \
    src/main.cpp -o muludnep \
    -lglfw -limgui \
    -framework mujoco -framework OpenGL
