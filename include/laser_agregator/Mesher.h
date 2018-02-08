#pragma once

//c++
#include <iosfwd>
#include <atomic>
#include <memory>

//ros
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>


//My stuff
#include "laser_agregator/Mesh.h"
#include "laser_agregator/point_types.h"


//forward declarations
class Profiler;
class Edge;


#define NUM_MESHES_BUFFER 5


#define TIME_SCOPE(name)\
    TIME_SCOPE_2(name,m_profiler);


class Mesher{
public:
    Mesher();
    void compute_mesh(pcl::PointCloud<PointXYZIDR>::Ptr cloud);

    Mesh get_mesh();
    bool is_modified(){return m_mesh_is_modified;};



    int m_smoothing_iters;
    float m_smoothing_stepsize;
    float m_smoothing_max_movement;

    float m_normal_thresh;
    float m_edge_merge_thresh;
    float m_triangle_area;
    float m_triangle_angle;
    bool m_show_as_image;
    bool m_show_delaunay;
    float m_edge_grazing_angle_thresh_horizontal;
    float m_edge_grazing_angle_thresh_vertical;
    float m_min_grazing;
    float m_max_tri_length;
    float m_min_tri_quality;
    bool m_create_faces;
    bool m_improve_mesh;
    bool m_adaptive_edge_length;
    bool m_do_random_edge_stopping;
    float m_random_edge_stopping_thresh;
    bool m_compute_naive_mesh;

    //triangle params
    bool m_triangle_silent;
    bool m_triangle_fast_arithmetic;
    bool m_triangle_robust_interpolation;


    int m_min_length_horizontal_edge;
    int m_max_length_horizontal_edge;


    std::vector<Mesh> m_meshes; //need to use a buffer of meshes because the Mesher needs to keep in memory both the calculated mesh and the one its currently working on
    int m_finished_mesh_idx; //idx pointing to the most recent finished mesh
    int m_working_mesh_idx; //idx poiting to the mesh we are currently working on
    pcl::PointCloud<PointXYZIDR>::Ptr last_cloud;

    std::shared_ptr<Profiler> m_profiler;

private:
    void init_params();
    void simplify(pcl::PointCloud<PointXYZIDR>::Ptr cloud);
    void naive_mesh(pcl::PointCloud<PointXYZIDR>::Ptr cloud);
    Eigen::MatrixXd pcl2eigen(pcl::PointCloud<PointXYZIDR>::Ptr cloud);
    void smooth_mesh(Mesh& mesh);
    void improve_mesh(Mesh& mesh);
    void remove_faces_with_low_confidence(Mesh& mesh);
    bool print_non_manifold_edges(std::vector<bool>& is_face_non_manifold, std::vector<bool>& is_vertex_non_manifold, const Eigen::MatrixXi& F_in);
    void remove_unreferenced_verts(Mesh& mesh);

    void create_naive_mesh(Mesh &mesh, const pcl::PointCloud<PointXYZIDR>::Ptr cloud);
    Eigen::MatrixXi create_edges(Mesh& mesh, row_type_b& is_vertex_an_edge_endpoint);
    void delaunay(Mesh& mesh, const row_type_b& is_vertex_an_edge_endpoint);

    std::atomic<bool> m_mesh_is_modified;

};
