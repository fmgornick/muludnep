#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <stdint.h>
#include <stdio.h>

#define true 1
#define false 0
#define dbreak() __builtin_debugtrap()
#define assert(cond, msg)                                                 \
    if (!(cond))                                                          \
    {                                                                     \
        printf("%s:%d - (%s): %s\n", __func__, __LINE__, (#cond), (msg)); \
        exit(1);                                                          \
    }

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef uint64_t u64;
typedef float f32;
typedef double f64;
typedef i32 bool;

typedef struct world {
    // MuJoCo info
    mjModel *model;
    mjData *data;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scene;
    mjrContext context;

    // object ids
    i32 cart_x_id;
    i32 cart_y_id;
    i32 pole_geom_id;
    i32 hinge_id;

    // GLFW objects
    i32 width;
    i32 height;
    GLFWwindow *window;

    // mouse interaction
    bool button_left;
    bool button_middle;
    bool button_right;
    f64 lastx;
    f64 lasty;
} world;
