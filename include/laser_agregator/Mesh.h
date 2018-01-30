#pragma once

#include <iostream>

//eigen
#include <Eigen/Dense>

//loguru
#define LOGURU_WITH_STREAMS 1
#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#include "laser_agregator/MiscUtils.h"

#define HORIZONTAL_FOV 360.0
#define VERTICAL_FOV 30.0

class Mesh{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Mesh();
    void add(const Mesh& new_mesh); //Adds another mesh to this one and combines it into one
    void clear();
    bool is_empty()const;
    void apply_transform(Eigen::Affine3d);
    void apply_D();  //applies any changes in the D vector to the positions of the vertices
    void to_image();
    void to_image(Eigen::Affine3d tf_currframe_alg);
    void to_mesh();
    void to_mesh(Eigen::Affine3d tf_currframe_alg);
    void to_3D();   //from a matrix with 2 columns creates one with 3 columns (requiered to go from the delaunay triangulation into a 3d mesh representable in libigl)
    void to_2D();  //from a matrix V with 3 columns, it discards te last one and creates one with 2 columns (in order for an image to be passed to the triangle library)
    void sanity_check() const; //check that all the data inside the mesh is valid, there are enough normals for each face, faces don't idx invalid points etc.


    friend std::ostream &operator<<(std::ostream&, const Mesh& m);


    template <typename DerivedC>
    inline void set_C( const Eigen::MatrixBase<DerivedC> & C_){

        if (C_.cols()==3){
            C = C_.template cast<double>();
        } else if(C_.cols()==1){
            C = C_.template cast<double>().replicate(1,3);
        }else{
            LOG(ERROR) << "Mesh::set_C input C_ needs to have either 1 or 3 columns and it has" << C_.cols();
        }

        //normalize the colors
        double max = C.maxCoeff();
        double min = C.minCoeff();
        double range = std::fabs(max) + std::fabs(min);
        double inv_range = 1.0/range;
        C.array() = (C.array() - min) *inv_range;

    }
    void clear_C();


    int width;
    int height;


    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd C;
    Eigen::MatrixXi E;
    Eigen::MatrixXd D;   //distances of points to the sensor
    Eigen::MatrixXd N_faces; //normals of each face
    Eigen::MatrixXd NV; //normals of each face
    uint64_t t; //timestamp;


    Eigen::Affine3d sensor_pose;  //mapping from baselink to world coordinates
    Eigen::Affine3d m_tf_currframe_alg; //mapping from alg frame to whatever the current frame is

private:

};
