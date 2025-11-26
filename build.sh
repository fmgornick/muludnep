#!/bin/sh

cd "$(dirname "$0")"
dir=$PWD

clang++ -std=c++17 -g -Wall \
    -F$dir/libs/mujoco \
    -I$dir/libs/glfw/3.4/include \
    -I$dir/libs/imgui \
    -I$dir/libs/imgui/backends \
    -I$dir/libs/mujoco/mujoco.framework/Versions/Current \
    -L$dir/libs/imgui \
    -Wl,-rpath,@executable_path/libs/imgui \
    -Wl,-rpath,$dir/libs/mujoco \
    src/main.cpp -o muludnep \
    -framework mujoco -framework Cocoa -framework IOKit -framework CoreVideo -framework OpenGL \
    -lglfw -limgui
