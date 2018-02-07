#pragma once

#include <iostream>

#include <Eigen/Dense>

#include "laser_agregator/Mesh.h"

class Edge{
public:
    Edge(Mesh& mesh);
    double get_length();
    Eigen::Vector2i get_eigen();  //returns the edge as an eigen vector2i with indexes into the mesh corresponding to the 2 points that define the mesh
    Eigen::Vector3d get_dir();    //get the direction in 3d that the edge spans going from startpoint to endpoint
    Eigen::Vector3d get_dir_normalized();
    double similarity(Edge& edge);
    bool is_valid();              //the edge is valid if both of the indexes it point to into the mesh are valid
    bool has_start_idx();
    bool has_end_idx();
    void invalidate();
    void stop_and_continue(); //stops this edge but creates inside this one the one that would continue from it
    void copy(Edge& new_edge);
    bool is_at_grazing_angle (const double grazing_angle_thresh);
    void check_and_push(std::vector<Edge>& edges_vec, const double grazing_angle_thresh); //add the edge only if it's below a certain grazing angle threshold with respect to the view vector going form the sensor to the endpoint of the beam

    void set_start_idx(int idx);
    void set_end_idx(int idx);

    int get_start_idx();
    int get_end_idx();

    void push_into_vec(std::vector<Edge>& edges_vec);

    bool need_to_split_at_beggining;
    bool need_to_split_at_finale;

private:
    Mesh& m_mesh;
    Eigen::Vector2i m_edge;
    Eigen::Vector3d m_first_dir;   //the first time the edge becomes valid (have te two points) it will set the dir. It shouldn't be computed each time the edge gets updated otherwise it would slowly deviate from the first initial direction
    int m_length; //represent the nr of segment (so an edge with two consecutive points will have length 1)

};
