#include "base.h"

void key_callback(GLFWwindow *window, int key, int scancode, int act, int mods);
void mouse_button_callback(GLFWwindow *window, int button, int act, int mods);
void cursor_pos_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

void
init_renderer(world *w, int width, int height)
{
    // set dimensions
    w->width = width;
    w->height = height;
    // init glfw window
    assert(glfwInit(), "initialize window");
    w->window = glfwCreateWindow(w->width, w->height, "Inverted Pendulum", NULL, NULL);
    glfwSetWindowUserPointer(w->window, w);
    // add callbacks
    glfwMakeContextCurrent(w->window);
    glfwSetKeyCallback(w->window, key_callback);
    glfwSetMouseButtonCallback(w->window, mouse_button_callback);
    glfwSetCursorPosCallback(w->window, cursor_pos_callback);
    glfwSetScrollCallback(w->window, scroll_callback);
}

void
key_callback(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    world *w = glfwGetWindowUserPointer(window);
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(w->model, w->data);
        mj_forward(w->model, w->data);
    }
    else if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
    else if (act == GLFW_PRESS && key == GLFW_KEY_Q)
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

void
mouse_button_callback(GLFWwindow *window, int button, int act, int mods)
{
    world *w = glfwGetWindowUserPointer(window);
    // update button state
    w->button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    w->button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    w->button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
}

void
cursor_pos_callback(GLFWwindow *window, double xpos, double ypos)
{
    world *w = glfwGetWindowUserPointer(window);
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
    world *w = glfwGetWindowUserPointer(window);
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
