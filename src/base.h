#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <stdint.h>
#include <stdio.h>

#define WIDTH 1600
#define HEIGHT 900

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

typedef struct world {
    // MuJoCo info
    mjModel *model;
    mjData *data;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scene;
    mjrContext context;

    // object ids
    i32 platform_id;
    i32 pole_id;
    i32 joint_x_id;
    i32 joint_y_id;
    i32 hinge_x_id;
    i32 hinge_y_id;

    f64 pole_mass;
    f64 platform_mass;

    // 0 pos   - pole position on (X|Y)-axis
    // 1 theta - pole angle from (YZ|XZ)-plane
    // 2 vel   - pole velocity along (X|Y)-axis
    // 3 rvel  - pole rotational velocity from (YZ|XZ)-plane
    f64 state_x[4];
    f64 state_y[4];
    f64 tip[3];

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

    // void key_callback(GLFWwindow *window, int key, int scancode, int act, int mods);
    // void mouse_button_callback(GLFWwindow *window, int button, int act, int mods);
    // void cursor_pos_callback(GLFWwindow *window, double xpos, double ypos);
    // void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
} world;
