#!/bin/sh

cd "$(dirname "$0")"
OS="$(uname -s)"
ARCH="$(uname -m)"

if [ "$OS" = "Linux" ] && [ "$ARCH" = "x86_64" ]; then
    g++ -std=c++17 -O3 -Wall \
        -Iinc \
        -Llib_linux_x86_64 \
        src/main.cpp -o muludnep \
        -Wl,-rpath,lib_linux_x86_64 \
        -lGL -lmujoco -limgui -lglfw3
elif [ "$OS" = "Darwin" ] && [ "$ARCH" = "arm64" ]; then
    clang++ -std=c++17 -O3 -Wall \
        -Iinc \
        -Llib_darwin_aarch64 \
        src/main.cpp -o muludnep \
        -Wl,-rpath,lib_darwin_aarch64 \
        -framework Cocoa -framework IOKit -framework OpenGL \
        -lmujoco -limgui -lglfw3
else
    echo "unsupported platform: $OS ($ARCH)"
    exit 1
fi
