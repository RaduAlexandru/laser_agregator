#pragma once

#include "laser_agregator/Mesh.h"

class Scene: public Mesh{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void add(const Mesh& local_mesh);
    void clear();
    void commit_sensor_poses();
    void split_in_parts();

    void sanity_check() const;
    friend std::ostream &operator<<(std::ostream&, const Mesh& m);


    //vector of poses for each of the added local_meshes;
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > m_sensor_poses;
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > m_tfs_alg_worldgl; //vector of transforms for each local mesh from the worldgl to the algorithm frame

    Eigen::MatrixXf m_per_vertex_classes_probs;   // VxNrClasses matrix, for each vertex store the probabilities of being in a certain class (updated on each new fuse)
    Eigen::VectorXd m_per_vertex_weights;  //Vx1 matrix, for each vertex store the weight of it being in said class (updated on each new fuse)

};
