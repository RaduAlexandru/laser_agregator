#pragma once

//c++
#include <iosfwd>
#include <atomic>
#include <memory> //shared_ptr

//My stuff
#include "laser_agregator/Mesh.h"
#include "laser_agregator/MiscUtils.h"
#include "laser_agregator/Scene.h"

//libigl
#include <igl/Hit.h>

#include <Eigen/Sparse>

//forward declarations
class Profiler;
namespace igl{ namespace embree{ class EmbreeIntersector; }};




#define NUM_SCENES_BUFFER 2

class Agregator{
public:
    Agregator();
    ~Agregator();
    void init_params();
    void agregate(const Mesh& local_mesh);
    void passthrough(const Mesh& local_mesh);
    void write_pwn();

    void enable();
    void disable();

    Scene get_last_agregated_mesh();
    bool is_modified(){return m_scene_is_modified;};



    //objects


    //params
    bool m_do_agregation;
    bool m_is_enabled;
    int m_nr_prealocated_points;


    int nr_of_agregations;
    char m_pwn_path[256] = "/media/alex/Data/Master/SHK/c_ws/src/laser_agregator/pwn_clouds";
    char m_pwn_filename[64] = "cloud.pwn";

    //misc
    bool m_show_prev_scene;
    bool m_show_last_two_scenes;

    //databasse
    Eigen::MatrixXd V_agregated;
    Eigen::MatrixXd NV_agregated;
    std::vector<Scene> m_scenes;
    int m_finished_scene_idx; //idx pointing to the most recent finished scene
    int m_working_scene_idx; //idx poiting to the scene we are currently working on
    std::atomic<bool> m_scene_is_modified;
    int m_nr_points_agregated;

    std::shared_ptr<Profiler> m_profiler;


private:
    void update_buffers_with_new_scene(const Scene& scene);  //updates the next scene in the buffer to be also this scene, therefore when we get the next local mesh we will have an updated scene at that working_scene_idx

    void naive_fuse(Scene& scene, const Mesh& local_mesh);

    void preallocate();
    void deallocate();
};



#define TIME_SCOPE(name)\
    TIME_SCOPE_2(name,m_profiler);

#define TIME_START(name)\
    TIME_START_2(name,m_profiler);

#define TIME_END(name)\
    TIME_END_2(name,m_profiler);
