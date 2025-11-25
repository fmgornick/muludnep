#!/bin/sh

cd "$(dirname "$0")"
dir=$PWD

clang -std=c17 -O3 -Wall \
    -F$dir/libs/mujoco \
    -I$dir/libs/glfw/3.4/include \
    -I$dir/libs/mujoco/mujoco.framework/Versions/Current \
    -Wl,-rpath,$dir/libs/mujoco \
    src/main.c -o muludnep -framework mujoco -lglfw
