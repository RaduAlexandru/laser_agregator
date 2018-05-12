#include <iostream>
//#include <memory>
//#include <chrono>
//#include <thread>

//GL
#include <GL/glad.h>
#include <GLFW/glfw3.h>

// //libigl
//set this to supress libigl viewer help
//#define IGL_VIEWER_VIEWER_QUIET
#include <igl/viewer/Viewer.h>


//ImGui
#include <imgui.h>
#include "imgui_impl_glfw_gl3.h"

//loguru
#define LOGURU_IMPLEMENTATION 1
#define LOGURU_NO_DATE_TIME 1
#define LOGURU_NO_UPTIME 1
#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

//My stuff
#include "laser_agregator/Gui.h"
#include "laser_agregator/Core.h"
#include "laser_agregator/Profiler.h"

void switch_callbacks(
        std::shared_ptr<igl::viewer::Viewer> view); //Need to switch the callbacks so that the input goes to either libigl or imgui

int main(int argc, char *argv[]) {

    loguru::init(argc, argv);
    loguru::g_stderr_verbosity = -1; //By default don't show any logs except warning, error and fatals

    ros::init(argc, argv, "laser_mesher");

    LOG_S(INFO) << "Hello from main!";

    //Objects
    std::shared_ptr<igl::viewer::Viewer> view(new igl::viewer::Viewer());
    view->launch_init();  //creates the actual opengl window so that imgui can attach to it
    std::shared_ptr<Profiler> profiler(new Profiler());
    std::shared_ptr<Core> core(new Core(view,profiler));
    std::shared_ptr<Gui> gui(new Gui(core, view,profiler));
    gui->init_fonts(); //needs to be initialized inside the context



    //Eyecandy options
    view->core.background_color << 0.2, 0.2, 0.2, 1.0;
    view->core.show_lines = false;
    view->core.point_size = 2;
    view->ngui->window()->setVisible(false);
    // viewer.core.line_color << 1.0, 0.2, 0.2, 1.0;


    // Mesh mesh=core->read_mesh_from_file("data/maxsimple.obj");
    // core->set_mesh(mesh);


    while (ros::ok()) {

        // LOG_SCOPE(INFO,"main_loop");
        glfwPollEvents();
        ImGui_ImplGlfwGL3_NewFrame();

        switch_callbacks(view);  //needs to be done inside the context otherwise it segments faults

        gui->update();
        core->update();
        if (core->m_viewer_initialized) {
            view->draw();
        } else {
            view->core.clear_framebuffers(); //if there is no mesh to draw then just put the color of the background
        }


        std::this_thread::sleep_for(std::chrono::milliseconds(5));


        ImGui::Render();
        glfwSwapBuffers(view->window);

    }




    // Cleanup
    view->launch_shut();
    ImGui_ImplGlfwGL3_Shutdown();
    glfwTerminate();

    return 0;
}


void switch_callbacks(std::shared_ptr<igl::viewer::Viewer> view) {
    bool hovered_imgui = ImGui::IsMouseHoveringAnyWindow();
    if (hovered_imgui) {
        glfwSetMouseButtonCallback(view->window, ImGui_ImplGlfwGL3_MouseButtonCallback);
        glfwSetScrollCallback(view->window, ImGui_ImplGlfwGL3_ScrollCallback);
        glfwSetKeyCallback(view->window, ImGui_ImplGlfwGL3_KeyCallback);
        glfwSetCharCallback(view->window, ImGui_ImplGlfwGL3_CharCallback);
        glfwSetCharModsCallback(view->window, nullptr);
    } else {
        glfwSetMouseButtonCallback(view->window, glfw_mouse_press);
        glfwSetScrollCallback(view->window, glfw_mouse_scroll);
        glfwSetKeyCallback(view->window, glfw_key_callback);
        glfwSetCharModsCallback(view->window, glfw_char_mods_callback);
        glfwSetCharCallback(view->window, nullptr);
    }
}
