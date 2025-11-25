#include "render.c"
#include "sim.c"
#include <unistd.h>

i32
main()
{
    world w = { 0 };
    init_renderer(&w, 1600, 900);
    init_sim(&w);

    printf("%d\n", w.cart_x_id);
    printf("%d\n", w.cart_y_id);
    printf("%d\n", w.hinge_id);
    w.data->ctrl[w.cart_x_id] = 5.0;
    w.data->ctrl[w.cart_y_id] = 5.0;

    // Main loop
    while (!glfwWindowShouldClose(w.window))
    {
        mj_step(w.model, w.data);

        // Render ---------------------------------------------------
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(w.window, &viewport.width, &viewport.height);

        mjv_updateScene(w.model, w.data, &w.opt, NULL, &w.cam, mjCAT_ALL, &w.scene);
        mjr_render(viewport, &w.scene, &w.context);

        glfwSwapBuffers(w.window);
        glfwPollEvents();

        usleep(10000);
    }

    // Cleanup
    free_sim(&w);
    glfwTerminate();

    return EXIT_SUCCESS;
}
