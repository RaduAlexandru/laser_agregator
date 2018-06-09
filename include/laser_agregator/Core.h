#pragma once
//C++
#include <iosfwd>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <memory>

//My stuff
#include "laser_agregator/Mesh.h"
#include "laser_agregator/Scene.h"



//ROS
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <tf/transform_listener.h>
#include "laser_agregator/point_types.h"

#define NUM_CLASSES 66

//forward declarations
class Mesher;
class Agregator;
class Profiler;
class RosBagPlayer;
namespace igl {  namespace opengl {namespace glfw{ class Viewer; }}}


#define TIME_SCOPE(name)\
    TIME_SCOPE_2(name,m_profiler);

#define TIME_START(name)\
    TIME_START_2(name,m_profiler);


class Core{
public:
    Core(std::shared_ptr<igl::opengl::glfw::Viewer> view, std::shared_ptr<Profiler> profiler);
    void update();
    void init_params();
    void read_data();

    Mesh read_mesh_from_file(std::string file_path);
    void set_mesh( Mesh& mesh);
    void set_points( Mesh& mesh);
    void set_edges( Mesh& mesh);
    Eigen::MatrixXd color_points(const Mesh& mesh)const;  //creates color for all the points in the mesh depending on the m_color_type value
    void write_ply();
    void write_obj();
    void recompute_mesher();
    void fix_cloud_orientation(pcl::PointCloud<PointXYZIDR>::Ptr cloud);
    void remove_point_in_the_gap(pcl::PointCloud<PointXYZIDR>::Ptr cloud); //NEEDS TO BE CALLED WHEN THE CLOUD IS IN ALGORITHM FRAME


    //objects dependencies
    std::shared_ptr<igl::opengl::glfw::Viewer> m_view;
    std::shared_ptr<Profiler> m_profiler;
    std::shared_ptr<Mesher> m_mesher;
    std::shared_ptr<Agregator> m_agregator;


    //Misc
    Scene m_scene;
    pcl::PointCloud<PointXYZIDR>::Ptr m_last_cloud;
    char m_exported_filename[64] = "./scene";
    uint64_t m_last_timestamp;
    int m_nr_callbacks;
    int m_skip;


    //ros
    int m_nr_cams;
    std::string m_frame_world;
    std::string m_frame_cam_left;
    std::string m_frame_cam_right;
    std::vector<std::string> m_cam_img_topics;
    std::vector<std::string> m_cam_info_topics;
    std::string m_pose_file;
    std::string m_bag_args;
    std::shared_ptr<RosBagPlayer> m_player;
    tf::TransformListener m_tf_listener;
    float m_view_direction;


    bool m_viewer_initialized;
    bool m_player_should_do_one_step;
    bool m_player_should_continue_after_step;
    std::atomic<bool> m_visualization_should_change;
    std::mutex m_mutex_recon;


    //visualizer params
    bool m_show_points;
    bool m_show_mesh;
    bool m_show_edges;
    int m_color_type;
    bool m_show_sensor_poses;
    const char* m_color_types_desc[8] =
      {
              "Jet color",
              "Gray scale",
              "Distance to sensor",
              "By idx in the V vector",
              "Ambient occlusion",
              "AO(Gold)",
              "Default",
              "Gold"
      };
    float m_cap_max_y;
    float m_point_size;
    Eigen::Vector3f m_mesh_color;


    //transforms
    Eigen::Affine3d m_tf_alg_vel; //transformation from velodyne frame to the algorithm frame
    Eigen::Affine3d m_tf_baselink_vel;
    Eigen::Affine3d m_tf_worldGL_worldROS;
    std::unordered_map<uint64_t, Eigen::Affine3d> m_worldROS_baselink_map;
    std::vector<uint64_t>m_timestamps_original_vec;
    std::vector<Eigen::Affine3d>m_worldROS_baselink_vec;
    std::unordered_map<uint64_t,int> m_scan_nr_map;
    bool m_exact_pose;

    //agregating all info
    Eigen::MatrixXd V_all;
    Eigen::MatrixXd NV_all;


    //Misc stuff that help with presenting the mesh
    float m_animation_time;
    char m_results_path[256] = "./results/";
    char m_single_png_filename[64] = "img.png";
    int m_orbit_frame_counter;
    bool m_write_viewer_at_end_of_pipeline;
    int m_magnification;
    void write_single_png();
    void write_orbit_png();  //does a full orbit and writes pngs after each incremental step
    void init_orbit(int& nr_steps, Eigen::Matrix3f& rotation_increment);
    void orbit();  // does a full robit but doesn't write anything
    void make_incremental_step_in_orbit(const Eigen::Matrix3f& rotation_increment);  // does a small step corresponding to an orbit
    void subsample_points();

    //decimate
    int m_decimation_nr_faces;
    float m_decimation_cost_thresh;
    void decimate(Mesh& mesh, const int nr_target_faces, const float decimation_cost_thresh);

private:

    // void callback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    //without cam info
    // void callback(const sensor_msgs::CompressedImageConstPtr& img_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    // only RGB
    // void callback(const sensor_msgs::CompressedImageConstPtr& img_msg);
    //only laser
    void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void read_pose_file();
    void create_transformation_matrices();
    bool get_pose_at_timestamp(Eigen::Affine3d& pose, uint64_t timestamp);
    bool get_tf(Eigen::Affine3d& tf, const std::string& origin_frame, const std::string& dest_frame, const ros::Time query_time );

};
