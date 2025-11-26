#include "ui.cpp"
#include <unistd.h>

i32
main()
{
    world w = { 0 };
    init_ui(&w);
    randomize_pole_orientation(&w);

    // w.data->ctrl[w.joint_x_id] = 5.0;
    // w.data->ctrl[w.joint_y_id] = 5.0;

    while (!glfwWindowShouldClose(w.window))
    {
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(w.window, &viewport.width, &viewport.height);

        mj_step(w.model, w.data);
        update_state(&w);
        mjv_updateScene(w.model, w.data, &w.opt, NULL, &w.cam, mjCAT_ALL, &w.scene);
        mjr_render(viewport, &w.scene, &w.context);

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::ShowDemoWindow();
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(w.window);
        glfwPollEvents();
        // usleep(10000);
    }

    // Cleanup
    destroy_ui(&w);
    glfwTerminate();

    return 0;
}
