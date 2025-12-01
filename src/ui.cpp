#include "base.hpp"
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <imgui.h>
#include <string.h>

void init_ui(world *w);
void destroy_ui(world *w);
void reset_pole_orientation(world *w);
void resize_callback(GLFWwindow *window, i32 width, i32 height);
void key_callback(GLFWwindow *window, i32 key, i32 scancode, i32 act, i32 mods);
void mouse_button_callback(GLFWwindow *window, i32 button, i32 act, i32 mods);
void cursor_pos_callback(GLFWwindow *window, f64 xpos, f64 ypos);
void scroll_callback(GLFWwindow *window, f64 xoffset, f64 yoffset);
void draw_sim(world *w);
void draw_panel(world *w);

void
init_ui(world *w)
{
    assert(glfwInit());
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    // glfw window
    w->window = glfwCreateWindow(w->width, w->height, "Inverted Pendulum", NULL, NULL);
    glfwMakeContextCurrent(w->window);
    glfwSetWindowUserPointer(w->window, w);
    glfwSwapInterval(1);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    ImGui::StyleColorsDark();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(w->window, true);
    ImGui_ImplOpenGL3_Init("#version 120");

    // callbacks
    glfwSetFramebufferSizeCallback(w->window, resize_callback);
    glfwSetKeyCallback(w->window, key_callback);
    glfwSetMouseButtonCallback(w->window, mouse_button_callback);
    glfwSetCursorPosCallback(w->window, cursor_pos_callback);
    glfwSetScrollCallback(w->window, scroll_callback);

    // mujoco data+model
    char error[1000];
    mjSpec *spec = mj_parseXMLString(scene, NULL, error, 1000);
    w->model = mj_compile(spec, NULL);
    assert(w->model);
    w->data = mj_makeData(w->model);
    assert(w->data);
    // defaults
    mjv_defaultScene(&w->scene);
    mjr_defaultContext(&w->context);
    mjv_defaultCamera(&w->cam);
    mjv_defaultOption(&w->opt);
    mjv_makeScene(w->model, &w->scene, 2000);
    mjr_makeContext(w->model, &w->context, mjFONTSCALE_150);

    reset_pole_orientation(w);
}

void
destroy_ui(world *w)
{
    mjv_freeScene(&w->scene);
    mjr_freeContext(&w->context);
    mj_deleteData(w->data);
    mj_deleteModel(w->model);
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwTerminate();
}

void
reset_pole_orientation(world *w)
{
    if (w->pole_start_angle_random)
    {
        f64 max_angle = mjPI / 6;
        w->data->qpos[w->hinge_x_qpos_id] = ((f64)rand() / RAND_MAX * 2.0 - 1.0) * max_angle;
        w->data->qpos[w->hinge_y_qpos_id] = ((f64)rand() / RAND_MAX * 2.0 - 1.0) * max_angle;
    }
    else
    {
        w->data->qpos[w->hinge_x_qpos_id] = w->pole_start_angle_x * (mjPI / 180.0f);
        w->data->qpos[w->hinge_y_qpos_id] = w->pole_start_angle_y * (mjPI / 180.0f);
    }
}

void
resize_callback(GLFWwindow *window, i32 width, i32 height)
{
    world *w = (world *)glfwGetWindowUserPointer(window);
    w->width = width;
    w->height = height;
}

void
key_callback(GLFWwindow *window, i32 key, i32 scancode, i32 act, i32 mods)
{
    world *w = (world *)glfwGetWindowUserPointer(window);
    ImGui_ImplGlfw_KeyCallback(window, key, scancode, act, mods);
    if (act == GLFW_PRESS && key == GLFW_KEY_W)
    {
        w->data->qfrc_applied[w->hinge_x_qpos_id] = -5.0f;
    }
    else if (act == GLFW_PRESS && key == GLFW_KEY_A)
    {
        w->data->qfrc_applied[w->hinge_y_qpos_id] = -5.0f;
    }
    else if (act == GLFW_PRESS && key == GLFW_KEY_S)
    {
        w->data->qfrc_applied[w->hinge_x_qpos_id] = 5.0f;
    }
    else if (act == GLFW_PRESS && key == GLFW_KEY_D)
    {
        w->data->qfrc_applied[w->hinge_y_qpos_id] = 5.0f;
    }
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(w->model, w->data);
        reset_pole_orientation(w);
        mj_forward(w->model, w->data);
    }
    // escape: close window
    else if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
    // q: close window
    else if (act == GLFW_PRESS && key == GLFW_KEY_Q)
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

void
mouse_button_callback(GLFWwindow *window, i32 button, i32 act, i32 mods)
{
    world *w = (world *)glfwGetWindowUserPointer(window);
    if (ImGui::GetIO().MousePos.x - 5 < w->panel_width)
    {
        ImGui_ImplGlfw_MouseButtonCallback(window, button, act, mods);
    }
    else
    {
        // update button state
        w->button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        w->button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        w->button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

        f64 xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        w->lastx = xpos;
        w->lasty = ypos;
    }
}

void
cursor_pos_callback(GLFWwindow *window, f64 xpos, f64 ypos)
{
    world *w = (world *)glfwGetWindowUserPointer(window);
    ImGui_ImplGlfw_CursorPosCallback(window, xpos, ypos);
    // no buttons down: nothing to do
    if (!w->button_left && !w->button_middle && !w->button_right)
    {
        return;
    }

    // compute mouse displacement, save
    f64 dx = xpos - w->lastx;
    f64 dy = ypos - w->lasty;
    w->lastx = xpos;
    w->lasty = ypos;

    // get current window size
    i32 width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (w->button_right)
    {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (w->button_left)
    {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else
    {
        action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(w->model, action, dx / height, dy / height, &w->scene, &w->cam);
}

void
scroll_callback(GLFWwindow *window, f64 xoffset, f64 yoffset)
{
    world *w = (world *)glfwGetWindowUserPointer(window);
    if (ImGui::GetIO().MousePos.x - 5 < w->panel_width)
    {
        ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
    }
    else
    {
        if ((w->cam.distance < 1.0 && yoffset < 0.0) || (w->cam.distance > 100.0 && yoffset > 0.0))
        {
            return;
        }
        else
        {
            // emulate vertical mouse motion = 5% of window height
            mjv_moveCamera(w->model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &w->scene, &w->cam);
        }
    }
}

void
draw_sim(world *w)
{
    mjrRect viewport = { 0, 0, 0, 0 };
    glfwGetFramebufferSize(w->window, &viewport.width, &viewport.height);

    mj_step(w->model, w->data);
    mjv_updateScene(w->model, w->data, &w->opt, NULL, &w->cam, mjCAT_ALL, &w->scene);
    mjr_render(viewport, &w->scene, &w->context);

    if (w->focus_robot)
    {
        w->cam.lookat[0] = w->data->qpos[w->platform_x_qpos_id];
        w->cam.lookat[1] = w->data->qpos[w->platform_y_qpos_id];
    }
}

void
draw_panel(world *w)
{
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(w->panel_width, w->height));

    ImGui::Begin("Inverted Pendulum Scene", NULL,
                 ImGuiWindowFlags_NoMove |         //
                     ImGuiWindowFlags_NoCollapse | //
                     ImGuiWindowFlags_NoBringToFrontOnFocus);
    w->panel_width = ImGui::GetWindowWidth();

    if (ImGui::CollapsingHeader("LQR Controller", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Text("Set LQR Penalties");
        ImGui::Separator();
        bool updating = false;
        if (ImGui::SliderFloat("Position penalty", &w->q_pos_penalty, 0.0f, 1000.0f)) updating = true;
        if (ImGui::SliderFloat("Velocity penalty", &w->q_vel_penalty, 0.0f, 1000.0f)) updating = true;
        if (ImGui::SliderFloat("Angle penalty", &w->q_angle_penalty, 0.0f, 1000.0f)) updating = true;
        if (ImGui::SliderFloat("Angular velocity penalty", &w->q_angvel_penalty, 0.0f, 1000.0f)) updating = true;
        if (updating)
        {
            w->q_updating = true;
            w->Q(0, 0) = w->q_pos_penalty;
            w->Q(1, 1) = w->q_angle_penalty;
            w->Q(2, 2) = w->q_vel_penalty;
            w->Q(3, 3) = w->q_angvel_penalty;
        }
        else if (w->q_updating && !ImGui::IsMouseDown(0))
        {
            w->q_updating = false;
            w->q_updated = true;
        }
        ImGui::Separator();

        ImGui::Text("Penalty Matrix Q");
        ImGui::Text("%8.3f %8.3f %8.3f %8.3f", w->Q(0, 0), w->Q(0, 1), w->Q(0, 2), w->Q(0, 3));
        ImGui::Text("%8.3f %8.3f %8.3f %8.3f", w->Q(1, 0), w->Q(1, 1), w->Q(1, 2), w->Q(1, 3));
        ImGui::Text("%8.3f %8.3f %8.3f %8.3f", w->Q(2, 0), w->Q(2, 1), w->Q(2, 2), w->Q(2, 3));
        ImGui::Text("%8.3f %8.3f %8.3f %8.3f", w->Q(3, 0), w->Q(3, 1), w->Q(3, 2), w->Q(3, 3));
        ImGui::Separator();
        ImGui::Text("LQR Gain Matrix K");
        ImGui::Text("%8.3f %8.3f %8.3f %8.3f", w->K(0, 0), w->K(0, 1), w->K(0, 2), w->K(0, 3));
        if (ImGui::Button("Reset Q"))
        {
            w->q_pos_penalty = 10.0f;
            w->q_angle_penalty = 1000.0f;
            w->q_vel_penalty = 1.0f;
            w->q_angvel_penalty = 100.0f;
            w->Q(0, 0) = w->q_pos_penalty;
            w->Q(1, 1) = w->q_angle_penalty;
            w->Q(2, 2) = w->q_vel_penalty;
            w->Q(3, 3) = w->q_angvel_penalty;
            w->q_updated = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Set K = 0"))
        {
            w->K.setZero();
        }
    }

    if (ImGui::CollapsingHeader("State & Control", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Text("Position         : (%8.3f m,     %8.3f m    )", w->x(0), w->y(0));
        ImGui::Text("Angle            : (%8.3f rad,   %8.3f rad  )", w->x(1), w->y(1));
        ImGui::Text("Velocity         : (%8.3f m/s,   %8.3f m/s  )", w->x(2), w->y(2));
        ImGui::Text("Angular Velocity : (%8.3f rad/s, %8.3f rad/s)", w->x(3), w->y(3));
        ImGui::Text("Control Input    : (%8.3f N,     %8.3f N    )", w->ux, w->uy);
    }

    if (ImGui::CollapsingHeader("Pole Angle", ImGuiTreeNodeFlags_DefaultOpen))
    {
        // Checkbox: randomize start angle
        ImGui::Checkbox("Randomize start angle", &w->pole_start_angle_random);

        if (!w->pole_start_angle_random)
        {
            ImGui::SliderFloat("Pole start angle X", &w->pole_start_angle_x, -30.0f, 30.0f, "%.2f deg");
            ImGui::SliderFloat("Pole start angle Y", &w->pole_start_angle_y, -30.0f, 30.0f, "%.2f deg");
            if (ImGui::Button("Apply Start Angle"))
            {
                reset_pole_orientation(w);
            }
        }
        else
        {
            ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.2f, 1.0f), "Angles will be randomized on reset");
        }
        ImGui::NewLine();
    }

    if (ImGui::Button("Reset Simulation"))
    {
        mj_resetData(w->model, w->data);
    }
    ImGui::Checkbox("Focus Camera on Robot", &w->focus_robot);

    ImGui::End();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
