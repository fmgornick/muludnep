#!/bin/sh

cd "$(dirname "$0")"
OS="$(uname -s)"
ARCH="$(uname -m)"

if [ "$OS" = "Darwin" ] && [ "$ARCH" = "arm64" ]; then
    clang++ -std=c++17 -O3 -Wall \
        -Iinc \
        -Llib_arm64_darwin \
        src/main.cpp -o muludnep \
        -Wl,-rpath,./lib_arm64_darwin \
        -framework Cocoa -framework IOKit -framework OpenGL \
        -lmujoco -limgui -lglfw3
elif [ "$OS" = "Linux" ] && [ "$ARCH" = "x86_64" ]; then
    g++ -std=c++17 -O3 -Wall \
        -Iinc \
        -Llib_x64_linux \
        src/main.cpp -o muludnep \
        -Wl,-rpath,./lib_x64_linux \
        -lGL -lmujoco -limgui -lglfw3
else
    echo "unsupported platform: $OS ($ARCH)"
    exit 1
fi
