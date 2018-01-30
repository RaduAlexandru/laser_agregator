#include "laser_agregator/Agregator.h"

//c++
#include <iostream>
#include <stack>
#include <numeric>

//my stuff
#include "laser_agregator/Profiler.h"


//libigl
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/remove_unreferenced.h>



//ros
#include <ros/ros.h>
#include "laser_agregator/RosTools.h"


Agregator::Agregator():
        m_scenes(NUM_SCENES_BUFFER),
        m_scene_is_modified(false),
        m_finished_scene_idx(-1),
        m_working_scene_idx(0){

    init_params();

    nr_of_agregations=0;

}

Agregator::~Agregator(){

}

void Agregator::init_params(){
    ros::NodeHandle private_nh("~");

}


void Agregator::agregate(const Mesh& local_mesh){

    TIME_SCOPE("agregate");
    LOG_SCOPE_F(INFO, "agregate");
    LOG(2) << "starting agregate nr " << nr_of_agregations;  //at fuse nr 55 , 73, it creates some points that are extremly far away

    Scene& scene = m_scenes[m_working_scene_idx];  //TODO may need to make it into a reference to speed things up


    //agregate V and NV


    //also store the last agregated mesh
    scene.clear();
    naive_fuse(scene, local_mesh);


    nr_of_agregations++;

    m_finished_scene_idx = m_working_scene_idx;
    m_working_scene_idx = (m_working_scene_idx + 1) % NUM_SCENES_BUFFER;
    m_scene_is_modified = true;
    update_buffers_with_new_scene(scene);


}

void Agregator::passthrough(const Mesh& local_mesh){
    TIME_SCOPE("passthrough");
    LOG_SCOPE_F(INFO, "passthrough");

    Scene& scene = m_scenes[m_working_scene_idx];  //TODO may need to make it into a reference to speed things up

    scene.clear();
    naive_fuse(scene, local_mesh);


    m_finished_scene_idx = m_working_scene_idx;
    m_working_scene_idx = (m_working_scene_idx + 1) % NUM_SCENES_BUFFER;
    m_scene_is_modified = true;
    update_buffers_with_new_scene(scene);

}

void Agregator::update_buffers_with_new_scene(const Scene& scene){
    //m_wordking_scene_idx was already incremented so we just copy in that position
    m_scenes[m_working_scene_idx]=scene;
}


void Agregator::naive_fuse(Scene& scene, const Mesh& local_mesh){
    scene.add(local_mesh);
}



//TODO may introduce a racing condition because between changing the bool to false it may be changed again to true by the other thread
Scene Agregator::get_last_agregated_mesh() {
    m_scene_is_modified = false;
    return m_scenes[m_finished_scene_idx];

}
