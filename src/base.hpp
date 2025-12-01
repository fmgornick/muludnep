#pragma once

#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <stdint.h>

#define WIDTH 1600
#define HEIGHT 900
#define PANEL_WIDTH 500.0f
#define nstate 4
#define nact 1

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
    i32 platform_x_qvel_id;
    i32 platform_y_qvel_id;
    i32 hinge_x_qpos_id;
    i32 hinge_y_qpos_id;
    i32 hinge_x_qvel_id;
    i32 hinge_y_qvel_id;

    f64 pole_mass;
    f64 platform_mass;

    Eigen::Matrix<f64, nstate, nstate> Q;
    Eigen::Matrix<f64, nact, nstate> K;
    Eigen::Matrix<f64, nstate, 1> x;
    Eigen::Matrix<f64, nstate, 1> y;
    f64 ux;
    f64 uy;

    // UI objects
    i32 width = WIDTH;
    i32 height = HEIGHT;
    f32 panel_width = PANEL_WIDTH;
    GLFWwindow *window;

    // mujoco sim interaction
    bool button_left;
    bool button_middle;
    bool button_right;
    f64 lastx;
    f64 lasty;

    // panel vars
    f32 q_pos_penalty = 10.0f;
    f32 q_angle_penalty = 1000.0f;
    f32 q_vel_penalty = 1.0f;
    f32 q_angvel_penalty = 100.0f;
    bool q_updating;
    bool q_updated;
    f32 pole_start_angle_x;
    f32 pole_start_angle_y;
    bool pole_start_angle_random;
    bool focus_robot;
} world;

const char scene[] = "<mujoco model=\"cartpole\">"
                     "    <compiler angle=\"radian\"/>"
                     "    <option timestep=\"0.01\"/>"
                     "    <asset>"
                     "        <texture type=\"skybox\" builtin=\"gradient\" rgb1=\"0.3 0.5 0.7\" rgb2=\"0 0 0\" width=\"512\" height=\"3072\"/>"
                     "        <texture type=\"2d\" name=\"groundplane\" builtin=\"checker\" mark=\"edge\" rgb1=\"0.2 0.3 0.4\" rgb2=\"0.1 0.2 0.3\" "
                     "markrgb=\"0.8 0.8 0.8\" width=\"300\" height=\"300\"/>"
                     "        <material name=\"groundplane\" texture=\"groundplane\" texuniform=\"true\" texrepeat=\"5 5\" reflectance=\"0.2\"/>"
                     "    </asset>"
                     "    <worldbody>"
                     "        <geom name=\"ground\" size=\"0 0 0.05\" type=\"plane\" material=\"groundplane\"/>"
                     "        <body name=\"platform\" pos=\"0 0 0.1\">"
                     "            <joint name=\"platform_x\" type=\"slide\" axis=\"1 0 0\"/>"
                     "            <joint name=\"platform_y\" type=\"slide\" axis=\"0 1 0\"/>"
                     "            <geom name=\"platform_geom\" type=\"cylinder\" size=\"0.3 0.1\" rgba=\"0.5 0.6 0.8 1\"/>"
                     "            <body name=\"pole\" pos=\"0 0 0.1\">"
                     "                <joint name=\"hinge_x\" type=\"hinge\" axis=\"1 0 0\"/>"
                     "                <joint name=\"hinge_y\" type=\"hinge\" axis=\"0 1 0\"/>"
                     "                <geom name=\"pole_geom\" type=\"capsule\" fromto=\"0 0 0 0 0 1\" size=\"0.05\" rgba=\"0.8 0.6 0.5 1\"/>"
                     "            </body>"
                     "        </body>"
                     "        </worldbody>"
                     "    <actuator>"
                     "        <motor joint=\"platform_x\" ctrlrange=\"-10000 10000\"/>"
                     "        <motor joint=\"platform_y\" ctrlrange=\"-10000 10000\"/>"
                     "    </actuator>"
                     "</mujoco>";
