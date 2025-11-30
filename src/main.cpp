#include "math.cpp"
#include "ui.cpp"
#include <unistd.h>

i32
main()
{
    world w = { 0 };
    init_ui(&w);
    init_math(&w);

    while (!glfwWindowShouldClose(w.window))
    {
        if (w.q_updated)
        {
            compute_lqr_gain(&w);
            w.q_updated = false;
        }
        control(&w);
        draw_sim(&w);
        draw_panel(&w);
        glfwSwapBuffers(w.window);
        glfwPollEvents();
        usleep(10000);
    }

    destroy_ui(&w);
    return 0;
}
