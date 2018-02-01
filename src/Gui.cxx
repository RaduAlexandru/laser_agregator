#include "laser_agregator/Gui.h"

//c++
#include <iostream>
#include <unordered_map>
#include <iomanip> // setprecision

//My stuff
#include "laser_agregator/Core.h"
#include "laser_agregator/Profiler.h"
#include "laser_agregator/RosBagPlayer.h"
#include "laser_agregator/MiscUtils.h"

//imgui
#include "imgui_impl_glfw_gl3.h"
#include "curve.hpp"

//loguru
//#include <loguru.hpp>

//libigl
#include <igl/viewer/Viewer.h>

Gui::Gui(std::shared_ptr<Core> core,
         std::shared_ptr<igl::viewer::Viewer> view,
         std::shared_ptr<Profiler> profiler) :
        m_show_demo_window(false),
        m_show_profiler_window(true),
        m_show_player_window(true){
    m_core = core;
    m_view = view;
    m_profiler=profiler;

    ImGui_ImplGlfwGL3_Init(m_view->window, true);

    init_style();

    m_bg_color = ImColor(51, 51, 51, 255);
    foo[0].x = -1;
}

void Gui::update() {

    ImVec2 canvas_size = ImGui::GetIO().DisplaySize;

    ImGuiWindowFlags main_window_flags = 0;
    main_window_flags |= ImGuiWindowFlags_NoMove;
    ImGui::SetNextWindowSize(ImVec2(310, canvas_size.y));
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::Begin("Laser Mesher", nullptr, main_window_flags);
    ImGui::PushItemWidth(135);


    if (ImGui::CollapsingHeader("Viewer")) {
        if (ImGui::Button("CenterCamera")) {
            m_view->core.align_camera_center(m_view->data.V);
        }
        //TODO not the best way of doing it because changing it supposes recomputing the whole mesh again
        if (ImGui::Checkbox("Show points", &m_core->m_show_points)) {
            m_core->m_visualization_should_change=true;
        }
        if (ImGui::Checkbox("Show mesh", &m_core->m_show_mesh)) {
            m_core->m_visualization_should_change=true;
        }
        if (ImGui::Checkbox("Show edges", &m_core->m_show_edges)) {
            m_core->m_visualization_should_change=true;
        }
        if (ImGui::Checkbox("Show sensor_poses", &m_core->m_show_sensor_poses)) {
            m_core->m_visualization_should_change=true;
        }
        if (ImGui::Combo("Color type", &m_core->m_color_type, m_core->m_color_types_desc, IM_ARRAYSIZE(m_core->m_color_types_desc))){
            m_core->m_visualization_should_change=true;
        }

       //Imgui view.core options
       ImGui::Checkbox("Show texture", &m_view->core.show_texture);
       ImGui::Checkbox("Show faces", &m_view->core.show_faces);
       ImGui::Checkbox("Show lines", &m_view->core.show_lines);
       ImGui::Checkbox("Show vertid", &m_view->core.show_vertid);
       ImGui::Checkbox("Show pointid", &m_view->core.show_pointid);
       ImGui::Checkbox("Show faceid", &m_view->core.show_faceid);
       ImGui::Checkbox("Invert_normals", &m_view->core.invert_normals);

       ImGui::SliderFloat("shininess", &m_view->core.shininess, 0.001f, 2.0f);
       ImGui::SliderFloat("lighting_factor", &m_view->core.lighting_factor, 0.0f, 2.0f);
       ImGui::SliderFloat("point_size", &m_view->core.point_size, 1.0, 7.0);
       if(ImGui::SliderFloat("m_cap_max_y", &m_core->m_cap_max_y, 0.1f, 50.0f)){
           m_core->m_visualization_should_change=true;
       }
       if(ImGui::ColorEdit3("Bg color", (float*)&m_bg_color)){
           m_view->core.background_color << m_bg_color.x , m_bg_color.y, m_bg_color.z;
       }

    }


    ImGui::Separator();
    if (ImGui::CollapsingHeader("Mesher")) {
        if(ImGui::Checkbox("Improve mesh", &m_core->m_mesher->m_improve_mesh)){
            m_core->recompute_mesher();
        }
        if (ImGui::SliderInt("m_min_length_horizontal_edge", &m_core->m_mesher->m_min_length_horizontal_edge, 0, 5)) {
           m_core->recompute_mesher();
       }
       if (ImGui::SliderInt("m_max_length_horizontal_edge", &m_core->m_mesher->m_max_length_horizontal_edge, 0, 400)) {
           m_core->recompute_mesher();
       }
        if (ImGui::SliderFloat("m_min_grazing", &m_core->m_mesher->m_min_grazing, 0.0f, 1.0f)) {
           m_core->recompute_mesher();
       }
       if (ImGui::SliderFloat("m_max_tri_length", &m_core->m_mesher->m_max_tri_length, 0.0f, 20.0f)) {
           m_core->recompute_mesher();
       }
       if (ImGui::SliderFloat("m_min_tri_quality", &m_core->m_mesher->m_min_tri_quality, 0.0f, 2.0f)) {
           m_core->recompute_mesher();
       }
    }


    ImGui::Separator();
    if (ImGui::CollapsingHeader("Misc")) {
        ImGui::SliderInt("log_level", &loguru::g_stderr_verbosity, -3, 9);
    }

    ImGui::Separator();
    if (ImGui::CollapsingHeader("Movies")) {
        ImGui::InputFloat("m_animation_time", &m_core->m_animation_time, 0, 0, 2);
        ImGui::InputText("results_path", m_core->m_results_path, IM_ARRAYSIZE(m_core->m_results_path));
        if (ImGui::Button("Write Single PNG")){
            m_core->write_single_png();
        }
        ImGui::SameLine();
        ImGui::InputText("", m_core->m_single_png_filename, IM_ARRAYSIZE(m_core->m_single_png_filename));

        if (ImGui::Button("Write Orbit to PNG")){
            m_core->write_orbit_png();
        }
        

        if (ImGui::Button("Dummy orbit")){
            m_core->orbit();
        }
    }




    ImGui::Separator();
    ImGui::Text(("Nr of points: " + format_with_commas(m_core->m_scene.V.rows())).data());
    ImGui::Text(("Nr of triangles: " + format_with_commas(m_core->m_scene.F.rows())).data());
    ImGui::Text("Average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);


    ImGui::InputText("exported filename", m_core->m_exported_filename, IM_ARRAYSIZE(m_core->m_exported_filename));
    if (ImGui::Button("Write PLY")){
        m_core->write_ply();
    }
    if (ImGui::Button("Write OBJ")){
        m_core->write_obj();
    }
    // static float f = 0.0f;
    // ImGui::Text("Hello, world!");
    // ImGui::SliderFloat("float", &f, 0.0f, 1.0f);
    if (ImGui::Button("Test Window")) m_show_demo_window ^= 1;
    if (ImGui::Button("Profiler Window")) m_show_profiler_window ^= 1;
    if (ImGui::Button("Player Window")) m_show_player_window ^= 1;



    // if (ImGui::Curve("Das editor", ImVec2(400, 200), 10, foo))
    // {
    //   // foo[0].y=ImGui::CurveValue(foo[0].x, 5, foo);
    //   // foo[1].y=ImGui::CurveValue(foo[1].x, 5, foo);
    //   // foo[2].y=ImGui::CurveValue(foo[2].x, 5, foo);
    //   // foo[3].y=ImGui::CurveValue(foo[3].x, 5, foo);
    //   // foo[4].y=ImGui::CurveValue(foo[4].x, 5, foo);
    // }



    ImGui::End();


    if (m_show_profiler_window && m_profiler->timings.size()>1 ){
        ImGuiWindowFlags profiler_window_flags = 0;
        profiler_window_flags |= ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar;
        int nr_timings=m_profiler->timings.size();
        ImVec2 size(310,50*nr_timings);
        ImGui::SetNextWindowSize(size);
        ImGui::SetNextWindowPos(ImVec2(canvas_size.x -size.x , 0));
        ImGui::Begin("Profiler", nullptr, profiler_window_flags);
        ImGui::PushItemWidth(135);


        for(auto &times: m_profiler->timings){
            std::stringstream stream_exp;
            stream_exp << std::fixed << std::setprecision(1) << times.second.exp_avg();
            std::string s_exp = stream_exp.str();
            std::stringstream stream_cma;
            stream_cma << std::fixed <<  std::setprecision(1) << times.second.avg();
            std::string s_cma = stream_cma.str();

//        std::string title = times.first +  "\n" + "(" + s_exp + ")" + "(" + s_cma + ")";
            std::string title = times.first +  "\n" + "avg: " + s_exp + " ms";
            ImGui::PlotLines(title.data(), times.second.data() , times.second.size() ,times.second.get_front_idx() );
        }

        ImGui::End();
    }


    if (m_show_player_window){
        ImGuiWindowFlags player_window_flags = 0;
        player_window_flags |=  ImGuiWindowFlags_NoTitleBar;
        ImVec2 size(125,56);
        ImGui::SetNextWindowSize(size);
        ImGui::SetNextWindowPos(ImVec2(canvas_size.x -size.x , canvas_size.y -size.y ));
        ImGui::Begin("Player", nullptr, player_window_flags);
        ImGui::PushItemWidth(135);
//        ImVec2 w_size= ImGui::GetWindowSize();
//        ImVec2 pos= ImGui::GetWindowPos();
//        std::cout << "size is " << w_size.x << " " << w_size.y << "\n";
//        std::cout << "pos is " << pos.x << " " << pos.y << "\n";

        ImVec2 button_size(25,25);
        const char* icon_play = m_core->m_player->is_paused() ? ICON_FA_PLAY : ICON_FA_PAUSE;
        if(ImGui::Button(icon_play,button_size)){
            m_core->m_player_should_do_one_step=false;
            m_core->m_player->pause();
        }
        ImGui::SameLine();
        if(ImGui::Button(ICON_FA_STEP_FORWARD,button_size)){
            m_core->m_player_should_do_one_step=true;
            //if it's paused, then start it
            if (m_core->m_player->is_paused()){
                m_core->m_player->pause();
            }
        }
        ImGui::SameLine();
        const char* icon_should_continue = m_core->m_player_should_continue_after_step? ICON_FA_FAST_FORWARD : ICON_FA_STOP;
        if(ImGui::Button(icon_should_continue,button_size)){
            m_core->m_player_should_do_one_step=true;
            //if it's paused, then start it
            if (m_core->m_player->is_paused()){
                m_core->m_player->pause();
            }
            m_core->m_player_should_continue_after_step ^= 1;
        }



        ImGui::End();
    }




    // 3. Show the ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
    if (m_show_demo_window) {
        ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
        ImGui::ShowTestWindow(&m_show_demo_window);
    }


}

void Gui::init_style() {
    //based on https://www.unknowncheats.me/forum/direct3d/189635-imgui-style-settings.html
    ImGuiStyle *style = &ImGui::GetStyle();

    style->WindowPadding = ImVec2(15, 15);
    style->WindowRounding = 0.0f;
    style->FramePadding = ImVec2(5, 5);
    style->FrameRounding = 4.0f;
    style->ItemSpacing = ImVec2(12, 8);
    style->ItemInnerSpacing = ImVec2(8, 6);
    style->IndentSpacing = 25.0f;
    style->ScrollbarSize = 8.0f;
    style->ScrollbarRounding = 9.0f;
    style->GrabMinSize = 5.0f;
    style->GrabRounding = 3.0f;

    style->Colors[ImGuiCol_Text] = ImVec4(0.80f, 0.80f, 0.83f, 1.00f);
    style->Colors[ImGuiCol_TextDisabled] = ImVec4(0.24f, 0.23f, 0.29f, 1.00f);
    style->Colors[ImGuiCol_WindowBg] = ImVec4(0.06f, 0.05f, 0.07f, 0.85f);
    style->Colors[ImGuiCol_ChildWindowBg] = ImVec4(0.07f, 0.07f, 0.09f, 1.00f);
    style->Colors[ImGuiCol_PopupBg] = ImVec4(0.07f, 0.07f, 0.09f, 1.00f);
    style->Colors[ImGuiCol_Border] = ImVec4(0.80f, 0.80f, 0.83f, 0.88f);
    style->Colors[ImGuiCol_BorderShadow] = ImVec4(0.92f, 0.91f, 0.88f, 0.00f);
    style->Colors[ImGuiCol_FrameBg] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
    style->Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.24f, 0.23f, 0.29f, 1.00f);
    style->Colors[ImGuiCol_FrameBgActive] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
    style->Colors[ImGuiCol_TitleBg] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
    style->Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
    style->Colors[ImGuiCol_TitleBgActive] = ImVec4(0.07f, 0.07f, 0.09f, 1.00f);
    style->Colors[ImGuiCol_MenuBarBg] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
    style->Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
    style->Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.80f, 0.80f, 0.83f, 0.31f);
    style->Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
    style->Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
    style->Colors[ImGuiCol_ComboBg] = ImVec4(0.19f, 0.18f, 0.21f, 1.00f);
    style->Colors[ImGuiCol_CheckMark] = ImVec4(0.80f, 0.80f, 0.83f, 0.31f);
    style->Colors[ImGuiCol_SliderGrab] = ImVec4(0.80f, 0.80f, 0.83f, 0.31f);
    style->Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
    style->Colors[ImGuiCol_Button] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
    style->Colors[ImGuiCol_ButtonHovered] = ImVec4(0.24f, 0.23f, 0.29f, 1.00f);
    style->Colors[ImGuiCol_ButtonActive] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
    style->Colors[ImGuiCol_Header] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
    style->Colors[ImGuiCol_HeaderHovered] = ImVec4(0.56f, 0.56f, 0.58f, 0.35f);
    style->Colors[ImGuiCol_HeaderActive] = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
    style->Colors[ImGuiCol_Column] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
    style->Colors[ImGuiCol_ColumnHovered] = ImVec4(0.24f, 0.23f, 0.29f, 1.00f);
    style->Colors[ImGuiCol_ColumnActive] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
    style->Colors[ImGuiCol_ResizeGrip] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    style->Colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
    style->Colors[ImGuiCol_ResizeGripActive] = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
    style->Colors[ImGuiCol_CloseButton] = ImVec4(0.40f, 0.39f, 0.38f, 0.16f);
    style->Colors[ImGuiCol_CloseButtonHovered] = ImVec4(0.40f, 0.39f, 0.38f, 0.39f);
    style->Colors[ImGuiCol_CloseButtonActive] = ImVec4(0.40f, 0.39f, 0.38f, 1.00f);
    style->Colors[ImGuiCol_PlotLines] = ImVec4(0.63f, 0.6f, 0.6f, 0.94f);
    style->Colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.25f, 1.00f, 0.00f, 1.00f);
    style->Colors[ImGuiCol_PlotHistogram] = ImVec4(0.63f, 0.6f, 0.6f, 0.94f);
    style->Colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.25f, 1.00f, 0.00f, 1.00f);
    style->Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.25f, 1.00f, 0.00f, 0.43f);
    style->Colors[ImGuiCol_ModalWindowDarkening] = ImVec4(1.00f, 0.98f, 0.95f, 0.73f);
}
