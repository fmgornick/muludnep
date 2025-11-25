#include "base.h"

void
init_sim(world *w)
{
    // load model
    char error[1000];
    w->model = mj_loadXML("cartpole.xml", NULL, error, 1000);
    assert(w->model, error);
    // load data
    w->data = mj_makeData(w->model);
    assert(w->data, "make data object");
    // init MuJoCo visualization structures
    mjv_defaultScene(&w->scene);
    mjr_defaultContext(&w->context);
    mjv_defaultCamera(&w->cam);
    mjv_defaultOption(&w->opt);
    mjv_makeScene(w->model, &w->scene, 2000);
    mjr_makeContext(w->model, &w->context, mjFONTSCALE_150);

    w->cart_x_id = mj_name2id(w->model, mjOBJ_JOINT, "cart_x");
    w->cart_y_id = mj_name2id(w->model, mjOBJ_JOINT, "cart_y");
    w->pole_geom_id = mj_name2id(w->model, mjOBJ_GEOM, "pole_geom");
    w->hinge_id = mj_name2id(w->model, mjOBJ_JOINT, "hinge");
}

void
free_sim(world *w)
{
    mjv_freeScene(&w->scene);
    mjr_freeContext(&w->context);
    mj_deleteData(w->data);
    mj_deleteModel(w->model);
}
