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

//boost
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;


Agregator::Agregator():
        m_scenes(NUM_SCENES_BUFFER),
        m_scene_is_modified(false),
        m_finished_scene_idx(-1),
        m_working_scene_idx(0),
        m_nr_points_agregated(0),
        m_nr_prealocated_points(33000000),
        m_do_agregation(true),
        m_is_enabled(true),
        m_nr_points_dropped(0){

    init_params();

    //reserve a big chng of it so we don't need to resize the matrices
    if(m_is_enabled){
        preallocate();
    }


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
    if(m_do_agregation){
        int nr_new_vertices=local_mesh.V.rows();
        if(  (m_nr_points_agregated+nr_new_vertices) > V_agregated.rows()){
            m_nr_points_dropped+=nr_new_vertices;
            LOG(ERROR) << "Not agregating since the buffer is already full. Dropping " << m_nr_points_dropped << "points";
        }else{
            V_agregated.block(m_nr_points_agregated,0, nr_new_vertices, 3)=local_mesh.V;
            NV_agregated.block(m_nr_points_agregated,0, nr_new_vertices, 3)=local_mesh.NV;
            m_nr_points_agregated+=nr_new_vertices;
        }
    }


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

void Agregator::write_pwn(){
    //open a file and write all the points as pwn file
    fs::path dir (m_pwn_path);
    fs::path pwn_name (m_pwn_filename);
    fs::path full_path = dir / pwn_name;
    fs::create_directory(dir);

    std::ofstream myfile;
    myfile.open (full_path.string());
    myfile << m_nr_points_agregated << "\n";
    for (size_t i = 0; i < m_nr_points_agregated; i++) {
        myfile << V_agregated(i,0) << " " << V_agregated(i,1) << " " << V_agregated(i,2) << "\n";
    }
    for (size_t i = 0; i < m_nr_points_agregated; i++) {
        myfile << NV_agregated(i,0) << " " << NV_agregated(i,1) << " " << NV_agregated(i,2) << "\n";
    }
    myfile.close();
}


void Agregator::enable(){
    preallocate();
}
void Agregator::disable(){
    deallocate();
}

void Agregator::preallocate(){
    V_agregated.resize(m_nr_prealocated_points,3);
    NV_agregated.resize(m_nr_prealocated_points,3);
    V_agregated.setZero();
    NV_agregated.setZero();
}
void Agregator::deallocate(){
    V_agregated.resize(0,0);
    NV_agregated.resize(0,0);
    m_nr_points_agregated=0;
}


//TODO may introduce a racing condition because between changing the bool to false it may be changed again to true by the other thread
Scene Agregator::get_last_agregated_mesh() {
    m_scene_is_modified = false;
    return m_scenes[m_finished_scene_idx];

}
