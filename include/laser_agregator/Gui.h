#pragma once

//c++
#include <iostream>
#include <memory>

//imgui
#include <imgui.h>
#include "IconsFontAwesome.h"

//forward declarations
class Core;
class Profiler;
namespace igl {  namespace viewer { class Viewer; }}

//Since the source directory is not known we get it from the cmake variable {CMAKE_SOURCE_DIR} through target_compile_definitions
#ifdef AWESOMEFONT_DIR
    #define AWESOMEFONT_PATH AWESOMEFONT_DIR
#else
    #define AWESOMEFONT_PATH "."
#endif


#define TIME_SCOPE(name)\
    TIME_SCOPE_2(name,m_profiler);


class Gui{
public:
    Gui(std::shared_ptr<Core> core,
        std::shared_ptr<igl::viewer::Viewer> view,
        std::shared_ptr<Profiler> profiler);
    void update();

    //Needs to be inline otherwise it doesn't work since it needs to be executed from a GL context
    inline void init_fonts(){
        ImGui::GetIO().Fonts->AddFontDefault();
        ImFontConfig config;
        config.MergeMode = true;
        const ImWchar icon_ranges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };
        ImGui::GetIO().Fonts->AddFontFromFileTTF(AWESOMEFONT_PATH, 13.0f, &config, icon_ranges);
    };
    void init_style();
private:

    // ImGuiIO& io= ImGui::GetIO();
    std::shared_ptr<Core> m_core;
    std::shared_ptr<igl::viewer::Viewer> m_view;
    std::shared_ptr<Profiler> m_profiler;

    bool m_show_demo_window;
    bool m_show_profiler_window;
    bool m_show_player_window;


    ImVec4 m_bg_color;
    ImVec4 m_mesh_color;

    ImVec2 foo[10];


};
