#include "laser_agregator/Scene.h"

//c++

//my stuff
#include "laser_agregator/MiscUtils.h"

//loguru
#include <loguru.hpp>

void Scene::add(const Mesh& local_mesh){
    Mesh::add(local_mesh);

    m_sensor_poses.push_back(local_mesh.sensor_pose);
    m_tfs_alg_worldgl.push_back(local_mesh.m_tf_currframe_alg);

    sensor_pose=local_mesh.sensor_pose;
    m_tf_currframe_alg=local_mesh.m_tf_currframe_alg;

}

void Scene::clear(){
    Mesh::clear();

    m_sensor_poses.clear();
    m_tfs_alg_worldgl.clear();

}


void Scene::commit_sensor_poses(){

    Mesh sensor_poses_mesh;
    sensor_poses_mesh.V.resize(m_sensor_poses.size(),3);
    // sensor_poses_mesh.E.resize(m_sensor_poses.size()-1,2);
    for (size_t i = 0; i < m_sensor_poses.size(); i++) {
        sensor_poses_mesh.V.row(i) = m_sensor_poses[i].translation();
        // if(i>=1){
        //     sensor_poses_mesh.E(i-1,0)= i-1;
        //     sensor_poses_mesh.E(i-1,1)= i;
        // }
    }

    Mesh::add(sensor_poses_mesh);


}


void Scene::split_in_parts(){
  //split in different parts by the increasing number of faces
}


void Scene::sanity_check() const{
    Mesh::sanity_check();
}

std::ostream& operator<<(std::ostream& os, const Scene& s)
{
    os << static_cast<const Mesh &>(s);

    // s.F_nr_refinements.size()?  os << "\t F_nr_refinements has size: " << s.F_nr_refinements.rows() << " x " << s.F_nr_refinements.cols() << "\n"   :   os << "\t F_nr_refinements is empty \n";

    return os;
}

Scene& Scene::operator= (const Mesh& mesh){
    clear();
    Mesh::add(mesh);
}
