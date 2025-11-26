#include "ui.cpp"
#include <unistd.h>

i32
main()
{
    world w = { 0 };
    init_ui(&w);
    // randomize_pole_orientation(&w);

    // w.data->ctrl[w.joint_x_id] = 5.0;
    // w.data->ctrl[w.joint_y_id] = 5.0;

    while (!glfwWindowShouldClose(w.window))
    {
        draw_sim(&w);
        draw_panel(&w);

        glfwSwapBuffers(w.window);
        glfwPollEvents();
        // usleep(10000);
    }

    // Cleanup
    destroy_ui(&w);
    glfwTerminate();

    return 0;
}
