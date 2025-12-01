#!/bin/sh

cd "$(dirname "$0")"

OS="$(uname -s)"
ARCH="$(uname -m)"

if [ "$OS" = "Darwin" ] && [ "$ARCH" = "arm64" ]; then
    clang++ -std=c++17 -O3 -Wall \
        -Iinc \
        src/main.cpp -o muludnep \
        lib_arm64_darwin/* \
        -framework Cocoa \
        -framework IOKit \
        -framework OpenGL
elif [ "$OS" = "Linux" ] && [ "$ARCH" = "x86_64" ]; then
    clang++ -std=c++17 -O3 -Wall -flto \
        -Iinc \
        src/main.cpp -o muludnep \
        lib_x64_linux/* \
        -lGL
else
    echo "unsupported platform: $OS ($ARCH)"
    exit 1
fi
