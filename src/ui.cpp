#include "base.hpp"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <string.h>

void init_ui(world *w);
void update_state(world *w);
void destroy_ui(world *w);
void reset_pole_orientation(world *w);
void resize_callback(GLFWwindow *window, int width, int height);
void key_callback(GLFWwindow *window, int key, int scancode, int act, int mods);
void mouse_button_callback(GLFWwindow *window, int button, int act, int mods);
void cursor_pos_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

void
init_ui(world *w)
{
    assert(glfwInit());
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    // glfw window
    w->width = WIDTH;
    w->height = HEIGHT;
    w->panel_width = PANEL_WIDTH;
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
    w->model = mj_loadXML("scene.xml", NULL, error, 1000);
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
    update_state(w);
}

void
update_state(world *w)
{
    // // x state vector
    // w->state_x[0] = w->data->xpos[3 * w->pole_id + 0];
    // w->state_x[1] = w->data->qpos[w->hinge_x_id];
    // w->state_x[2] = w->data->cvel[6 * w->pole_id + 3];
    // w->state_x[3] = w->data->cvel[6 * w->pole_id + 0];
    //
    // // y state vector
    // w->state_y[0] = w->data->xpos[3 * w->pole_id + 1];
    // w->state_y[1] = w->data->qpos[w->hinge_y_id];
    // w->state_y[2] = w->data->cvel[6 * w->pole_id + 4];
    // w->state_y[3] = w->data->cvel[6 * w->pole_id + 1];
    //
    // // tip   base   rotation  initial tip orientation
    // // x     px     r0 r1 r2  0
    // // y  =  py  +  r3 r4 r5  0
    // // z     pz     r6 r7 r8  1
    // w->tip[0] = w->data->xpos[3 * w->pole_id + 0] + w->data->xmat[9 * w->pole_id + 2];
    // w->tip[1] = w->data->xpos[3 * w->pole_id + 1] + w->data->xmat[9 * w->pole_id + 5];
    // w->tip[2] = w->data->xpos[3 * w->pole_id + 2] + w->data->xmat[9 * w->pole_id + 8];
    //
    // // clear forcces applied in frame;
    // f64 control_x = -(w->K[0] * w->state_x[0] + w->K[1] * w->state_x[1] + w->K[2] * w->state_x[2] + w->K[3] * w->state_x[3]);
    // f64 control_y = -(w->K[0] * w->state_y[0] + w->K[1] * w->state_y[1] + w->K[2] * w->state_y[2] + w->K[3] * w->state_y[3]);
    //
    // w->data->xfrc_applied[6 * w->joint_x_id] = control_x;
    // w->data->xfrc_applied[6 * w->joint_y_id + 1] = control_y;

    // memset(w->data->xfrc_applied, 0, sizeof(f64) * 6 * w->model->nbody);
    //
    // int j_platform_x = mj_name2id(w->model, mjOBJ_JOINT, "platform_x");
    // int j_platform_y = mj_name2id(w->model, mjOBJ_JOINT, "platform_y");
    // int j_hinge_x = mj_name2id(w->model, mjOBJ_JOINT, "hinge_x");
    // int j_hinge_y = mj_name2id(w->model, mjOBJ_JOINT, "hinge_y");
    //
    // int platform_x_qpos_id = w->model->jnt_qposadr[j_platform_x];
    // int platform_y_qpos_id = w->model->jnt_qposadr[j_platform_y];
    // int hinge_x_qpos_id = w->model->jnt_qposadr[j_hinge_x];
    // int hinge_y_qpos_id = w->model->jnt_qposadr[j_hinge_y];
    //
    // int platform_x_qvel_id = w->model->jnt_dofadr[j_platform_x];
    // int platform_y_qvel_id = w->model->jnt_dofadr[j_platform_y];
    // int hinge_x_qvel_id = w->model->jnt_dofadr[j_hinge_x];
    // int hinge_y_qvel_id = w->model->jnt_dofadr[j_hinge_y];
    //
    // VectorX x0(nstate);
    // read_state_from_sim(d, platform_x_qpos_id, platform_y_qpos_id, hinge_x_qpos_id, hinge_y_qpos_id, platform_x_qvel_id, platform_y_qvel_id, hinge_x_qvel_id,
    //                     hinge_y_qvel_id, x0);
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
        f64 max_angle = mjPI / 10;
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
resize_callback(GLFWwindow *window, int width, int height)
{
    world *w = (world *)glfwGetWindowUserPointer(window);
    w->width = width;
    w->height = height;
}

void
key_callback(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    world *w = (world *)glfwGetWindowUserPointer(window);
    ImGui_ImplGlfw_KeyCallback(window, key, scancode, act, mods);
    if (act == GLFW_PRESS && key == GLFW_KEY_W)
    {
        w->data->qfrc_applied[w->hinge_y_qpos_id] = 5.0f;
    }
    else if (act == GLFW_PRESS && key == GLFW_KEY_A)
    {
        w->data->qfrc_applied[w->hinge_x_qpos_id] = -5.0f;
    }
    else if (act == GLFW_PRESS && key == GLFW_KEY_S)
    {
        w->data->qfrc_applied[w->hinge_y_qpos_id] = -5.0f;
    }
    else if (act == GLFW_PRESS && key == GLFW_KEY_D)
    {
        w->data->qfrc_applied[w->hinge_x_qpos_id] = 5.0f;
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
mouse_button_callback(GLFWwindow *window, int button, int act, int mods)
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
cursor_pos_callback(GLFWwindow *window, double xpos, double ypos)
{
    world *w = (world *)glfwGetWindowUserPointer(window);
    ImGui_ImplGlfw_CursorPosCallback(window, xpos, ypos);
    // no buttons down: nothing to do
    if (!w->button_left && !w->button_middle && !w->button_right)
    {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - w->lastx;
    double dy = ypos - w->lasty;
    w->lastx = xpos;
    w->lasty = ypos;

    // get current window size
    int width, height;
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
scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
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

    ImGui::Begin("Scene", NULL, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus);
    w->panel_width = ImGui::GetWindowWidth();

    if (ImGui::CollapsingHeader("State", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Text("(pos, angle, vel, wvel)");
        ImGui::Separator();
        ImGui::Text("X: (%.3f, %.3f, %.3f, %.3f)", w->x(0), w->x(2), w->x(4), w->x(6));
        ImGui::Text("Y: (%.3f, %.3f, %.3f, %.3f)", w->x(1), w->x(3), w->x(5), w->x(7));
        ImGui::NewLine();
    }

    if (ImGui::CollapsingHeader("Pole Angle", ImGuiTreeNodeFlags_DefaultOpen))
    {
        // Checkbox: randomize start angle
        ImGui::Checkbox("Randomize start angle", &w->pole_start_angle_random);

        if (!w->pole_start_angle_random)
        {
            ImGui::SliderFloat("Pole start angle X", &w->pole_start_angle_x, -25.0f, 25.0f, "%.2f deg");
            ImGui::SliderFloat("Pole start angle Y", &w->pole_start_angle_y, -25.0f, 25.0f, "%.2f deg");
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

    i32 id = 0;
    if (ImGui::CollapsingHeader("Q", ImGuiTreeNodeFlags_DefaultOpen))
    {
        for (i32 i = 0; i < w->Q.rows(); i++)
        {
            for (i32 j = 0; j < w->Q.cols(); j++)
            {
                ImGui::PushID(id++);         // Unique ID for each element
                ImGui::SetNextItemWidth(50); // Optional: control width of input field
                ImGui::InputDouble("##matrix_element", &w->Q(i, j), 0.0f, 0.0f, "%.2f");
                ImGui::PopID();
                if (j < w->Q.cols() - 1)
                { // Don't add SameLine after the last element in a row
                    ImGui::SameLine();
                }
            }
        }
        ImGui::NewLine();
    }

    if (ImGui::CollapsingHeader("K", ImGuiTreeNodeFlags_DefaultOpen))
    {
        for (i32 i = 0; i < w->K.rows(); i++)
        {
            for (i32 j = 0; j < w->K.cols(); j++)
            {
                ImGui::PushID(id++);         // Unique ID for each element
                ImGui::SetNextItemWidth(50); // Optional: control width of input field
                ImGui::InputDouble("##matrix_element", &w->K(i, j), 0.0f, 0.0f, "%.2f");
                ImGui::PopID();
                if (j < w->K.cols() - 1)
                { // Don't add SameLine after the last element in a row
                    ImGui::SameLine();
                }
            }
        }
        ImGui::NewLine();
    }

    ImGui::Separator();
    ImGui::Text("X control: %lf", w->data->ctrl[0]);
    ImGui::Text("Y control: %lf", w->data->ctrl[1]);

    ImGui::End();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
