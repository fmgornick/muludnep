#pragma once

#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <stdint.h>
#include <stdio.h>

#define WIDTH 1600
#define HEIGHT 900
#define PANEL_WIDTH 300.0f
#define nstate 8
#define nact 2

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
    i32 platform_x_qpos_id;
    i32 platform_y_qpos_id;
    i32 hinge_x_qpos_id;
    i32 hinge_y_qpos_id;
    i32 platform_x_qvel_id;
    i32 platform_y_qvel_id;
    i32 hinge_x_qvel_id;
    i32 hinge_y_qvel_id;

    f64 pole_mass;
    f64 platform_mass;

    Eigen::Matrix<f64, nstate, nstate> Q;
    Eigen::Matrix<f64, nact, nstate> K;
    Eigen::Matrix<f64, nstate, 1> x;
    Eigen::Matrix<f64, nact, 1> u;

    // UI objects
    i32 width;
    i32 height;
    f32 panel_width;
    GLFWwindow *window;

    // mujoco sim interaction
    bool button_left;
    bool button_middle;
    bool button_right;
    f64 lastx;
    f64 lasty;

    // panel vars
    f32 pole_start_angle_x;
    f32 pole_start_angle_y;
    bool pole_start_angle_random;
} world;
