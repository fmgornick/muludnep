#!/bin/sh

cd "$(dirname "$0")"
dir=$PWD

clang++ -std=c++17 -O3 -Wall \
    -F$dir/libs/mujoco \
    -I$dir/inc \
    -L$dir/libs/imgui \
    -Wl,-rpath,@executable_path/libs/imgui \
    -Wl,-rpath,$dir/libs/mujoco \
    src/main.cpp -o muludnep \
    -framework mujoco -framework OpenGL \
    -lglfw -limgui
