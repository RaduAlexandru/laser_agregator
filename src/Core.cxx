//C++
#include <iostream>

//My stuff
#include "laser_agregator/Core.h"
#include "laser_agregator/MiscUtils.h"
#include "laser_agregator/Profiler.h"
#include "laser_agregator/RosBagPlayer.h"
#include "laser_agregator/Mesher.h"
#include "laser_agregator/Agregator.h"

//libigl
#include <igl/viewer/Viewer.h>
#include <igl/readOBJ.h>
#include <igl/readPLY.h>
#include <igl/writePLY.h>
#include <igl/writeOBJ.h>
#include <igl/remove_unreferenced.h>

//ROS
#include "laser_agregator/RosTools.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>

//pcl
#include <pcl/common/transforms.h>


//boost
#include <boost/filesystem.hpp>



Core::Core(std::shared_ptr<igl::viewer::Viewer> view, std::shared_ptr<Profiler> profiler) :
        m_viewer_initialized(false),
        m_show_points(false),
        m_show_mesh(true),
        m_show_sensor_poses(true),
        m_player(new RosBagPlayer),
        m_mesher(new Mesher),
        m_agregator(new Agregator),
        m_player_should_do_one_step(false),
        m_player_should_continue_after_step(false),
        m_visualization_should_change(false),
        m_color_type(5),
        m_cap_max_y(20),
        m_nr_callbacks(0){

    m_view = view;
    m_profiler=profiler;
    m_mesher->m_profiler=profiler;
    m_agregator->m_profiler=profiler;

    init_params();
    read_pose_file();
    create_transformation_matrices();


    m_player->play(m_bag_args);


    //start thread that continously reads ros laser messages and meshes them
    boost::thread t(&Core::read_data, this);
}


void Core::update() {


    if (m_agregator->is_modified() || m_visualization_should_change) {

        LOG_F(INFO, "Core:: saw that the scene was modified");

        if(m_agregator->is_modified() && m_player->is_paused() &&  m_player_should_continue_after_step){
            m_player_should_do_one_step=true; //so that when it starts the callback it puts the bag back into pause
            m_player->pause(); //starts the bag
        }

        m_scene=m_agregator->get_last_agregated_mesh();

        m_view->data.clear();

        if (m_show_mesh) {
            set_mesh(m_scene);  // the scene is internally broken down into various independent meshes
        }
        // if(m_show_sensor_poses){
        //     m_scene.commit_sensor_poses(); //add to the scene new points corresponding to the sensor poses
        // }
        if (m_show_points) {
            set_points(m_scene);
        }




        m_visualization_should_change=false;
    }

}


void Core::init_params() {
    //read the parameters from the launch file
    ros::NodeHandle private_nh("~");
    //ros
    m_nr_cams = getParamElseThrow<int>(private_nh, "nr_cams");
    m_frame_world = getParamElseDefault<std::string>(private_nh, "world_frame", "world");
    m_frame_cam_left = getParamElseDefault<std::string>(private_nh, "cam_frame_left", "camera");
    m_frame_cam_right = getParamElseDefault<std::string>(private_nh, "cam_frame_right", "camera");
    m_bag_args = getParamElseThrow<std::string>(private_nh, "bag_args");

    //verbosity
    loguru::g_stderr_verbosity = getParamElseDefault<int>(private_nh, "loguru_verbosity", -1);

    //visualizer
    m_show_points = getParamElseDefault<bool>(private_nh, "show_points", false);
    m_show_mesh = getParamElseDefault<bool>(private_nh, "show_mesh", true);
    m_show_edges  = getParamElseDefault<bool>(private_nh, "show_edges", false);

    //TODO read all the other parameters from the launch file
}

void Core::read_data() {
    //subscribe to the topics
    //keep spinning (it's running in a different thread)

    LOG(INFO) << "started ros communication";
    loguru::set_thread_name("ROS mesher");
    ros::NodeHandle private_nh("~");

    message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub(private_nh, "/cam_img_0", 3);
    // message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(private_nh, "/cam_info_0", 3);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(private_nh, "/laser", 3);

    //with camera info
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> AsyncPolicy;
    // message_filters::Synchronizer<AsyncPolicy> sync(AsyncPolicy(10), image_sub, info_sub, cloud_sub);
    // sync.registerCallback(boost::bind(&Core::callback,this, _1, _2,_3 ));

    //without camera info
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::PointCloud2> AsyncPolicy;
    message_filters::Synchronizer<AsyncPolicy> sync(AsyncPolicy(10), image_sub, cloud_sub);
    sync.registerCallback(boost::bind(&Core::callback, this, _1, _2));


    //only rgb
    // ros::Subscriber sub = private_nh.subscribe("/cam_img_0", 1000, &Core::callback, this);



    ros::spin();

    m_player->kill();
    LOG(INFO) << "finished ros communication";

}

void Core::callback(const sensor_msgs::CompressedImageConstPtr& img_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {


    LOG_S(INFO) << "calback------------------------------";
    std::lock_guard<std::mutex> lock(m_mutex_recon);

    if(m_player_should_do_one_step){
        m_player_should_do_one_step=false;
        m_player->pause();
    }


    //get the laser data into a point cloud
    pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*cloud_msg, *temp_cloud);
    pcl::PointCloud<PointXYZIDR>::Ptr cloud(new pcl::PointCloud<PointXYZIDR>);
    pcl::fromPCLPointCloud2(*temp_cloud, *cloud);

    m_last_timestamp=(uint64_t)std::round(cloud_msg->header.stamp.toNSec()/100000000.0);

    Eigen::Affine3d sensor_pose;
    if (!get_pose_at_timestamp(sensor_pose,m_last_timestamp) ){
                LOG(WARNING) << "Not found any pose at timestamp " << m_last_timestamp << " Discarding mesh";
        return;
    }

    //mesh it
    fix_cloud_orientation(cloud);
    m_last_cloud=cloud;
    //Mesh the point cloud
    Mesh local_mesh;
    //create one with all the points
    // m_mesher->create_simple_mesh2(local_mesh,cloud);
    // igl::per_face_normals(local_mesh.V, local_mesh.F, local_mesh.N_faces);
    // m_mesher->remove_faces_with_low_confidence(local_mesh);

    //create one with subsampled points
    m_mesher->simplify(cloud);
    local_mesh=m_mesher->get_mesh();


    local_mesh.apply_transform(m_tf_alg_vel.inverse());  //now we have it in vel frame
    local_mesh.apply_transform(m_tf_baselink_vel);   //from velodyne to baselink
    local_mesh.apply_transform(sensor_pose);   //from baselik to world ros
    local_mesh.apply_transform(m_tf_worldGL_worldROS);  //worldros to world gl so we can show it correctly and not from the bottom up


    //ONLY DO IN THE CASE OF SIMPLE MESH ( the delunay one already has removed unreferenced vertices)
    // //get rid of points that are not references by F, otherwise we won't have normals for them
    // Eigen::MatrixXd V_clean;
    // Eigen::MatrixXi F_clean;
    // Eigen::MatrixXi I;
    // igl::remove_unreferenced(local_mesh.V, local_mesh.F, V_clean, F_clean, I);
    // std::cout << "after remove unreferenced, mesh is "<< V_clean.rows() << '\n';
    // local_mesh.V=V_clean;
    // local_mesh.F=F_clean;

    // //get also normals
    // Eigen::MatrixXd NV;
    // igl::per_vertex_normals(V_clean, F_clean, NV);



    // m_view->data.clear();
    // set_mesh(local_mesh);

    m_agregator->agregate(local_mesh);


    // //Aggregate all of them
    // Eigen::MatrixXd V_new(V_all.rows() + V_clean.rows(), 3);
    // V_new << V_all, V_clean;
    // Eigen::MatrixXd NV_new(NV_all.rows() + NV.rows(), 3);
    // NV_new << NV_all, NV;
    // V_all=V_new;
    // NV_all=NV_new;

    std::cout << "callback_agregate_all finished" << '\n';

    if(m_player->is_paused() &&  m_player_should_continue_after_step){
        m_player_should_do_one_step=true; //so that when it starts the callback it puts the bag back into pause
        m_player->pause(); //starts the bag
    }

}

void Core::fix_cloud_orientation(pcl::PointCloud<PointXYZIDR>::Ptr cloud) {

    pcl::transformPointCloud (*cloud, *cloud, m_tf_alg_vel);

}

bool Core::get_tf(Eigen::Affine3d& tf, const std::string& origin_frame, const std::string& dest_frame, const ros::Time query_time ) {
    tf::StampedTransform tf_transform;
    try {
        m_tf_listener.lookupTransform( dest_frame, origin_frame, query_time, tf_transform );
    } catch ( tf::TransformException exc ) {
        LOG(WARNING) << "exc.what()";
        return false;
    }
    transformTFToEigen( tf_transform, tf );
    return true;
}




void Core::recompute_mesher(){
    VLOG(2) << "recompute mesher start";
    std::unique_lock<std::mutex> lock(m_mutex_recon, std::try_to_lock);
    if(lock.owns_lock()) {
        VLOG(2) << "recompute mesher has lock";
        m_mesher->simplify(m_last_cloud);
        Mesh local_mesh = m_mesher->get_mesh();

        local_mesh.t=m_last_timestamp;

        Eigen::Affine3d sensor_pose;
        if (!get_pose_at_timestamp(sensor_pose,local_mesh.t) ){
            LOG(WARNING) << "recompute_mesher: Not found any pose at timestamp " << local_mesh.t << " Discarding mesh";
            return;
        }

        local_mesh.apply_transform(m_tf_alg_vel.inverse());  //now we have it in vel frame
        local_mesh.apply_transform(m_tf_baselink_vel);   //from velodyne to baselink
        local_mesh.apply_transform(sensor_pose);  //from baselink to worldROS
        local_mesh.apply_transform(m_tf_worldGL_worldROS);  //worldros to world gl so we can show it correctly and not from the bottom up


        //only put in the case that the mesher calculated the simple one and not the delaunay mesh
        // //get rid of points that are not references by F, otherwise we won't have normals for them
        // Eigen::MatrixXd V_clean;
        // Eigen::MatrixXi F_clean;
        // Eigen::MatrixXi I;
        // igl::remove_unreferenced(local_mesh.V, local_mesh.F, V_clean, F_clean, I);
        // std::cout << "after remove unreferenced, mesh is "<< V_clean.rows() << '\n';
        // local_mesh.V=V_clean;
        // local_mesh.F=F_clean;

        m_agregator->passthrough(local_mesh);

        // m_fuser->fuse_with_prev_scene(local_mesh);
        m_mutex_recon.unlock();
    }else{
        VLOG(2) << "recompute mesher does not have lock";
    }
}



Mesh Core::read_mesh_from_file(std::string file_path) {

   Mesh mesh;

   std::string fileExt = file_path.substr(file_path.find_last_of(".") + 1);
   if (fileExt == "off") {
       igl::readOFF(file_path, mesh.V, mesh.F);
   } else if (fileExt == "ply") {
       igl::readPLY(file_path, mesh.V, mesh.F);
   } else if (fileExt == "obj") {
       igl::readOBJ(file_path, mesh.V, mesh.F);
   }

   return mesh;
}


void Core::set_mesh(Mesh &mesh) {
    if(mesh.is_empty()){
        return;
    }


   m_view->data.set_mesh(mesh.V, mesh.F);
   if (mesh.C.rows() == mesh.V.rows() || mesh.C.rows() == mesh.F.rows()) {
       m_view->data.set_colors(mesh.C);
   }else{
       m_view->data.set_colors(color_points(mesh));
   }


   if (!m_viewer_initialized) {
       m_viewer_initialized = true;
       m_view->core.align_camera_center(mesh.V, mesh.F);
   }
}

void Core::set_points(Mesh &mesh) {
    if(mesh.is_empty()){
        return;
    }

    // if there are none, then make some colors based on height
   if (mesh.C.rows() != mesh.V.rows()) {
       m_view->data.set_points(mesh.V, color_points(mesh));
   } else {
       m_view->data.set_points(mesh.V, mesh.C);
   }


   if (!m_viewer_initialized) {
       m_viewer_initialized = true;
       m_view->core.align_camera_center(mesh.V);
   }
}

void Core::set_edges(Mesh &mesh) {
    if(mesh.is_empty()){
        return;
    }

    //make some colors
    Eigen::MatrixXd C(mesh.E.rows(), 3);
    for (size_t i = 0; i < C.rows(); i++) {
        C(i, 0) = 1.0;
        C(i, 1) = 0.0;
        C(i, 2) = 0.0;
    }

   m_view->data.set_edges(mesh.V, mesh.E, C);


   if (!m_viewer_initialized) {
       m_viewer_initialized = true;
       m_view->core.align_camera_center(mesh.V);
   }
}


Eigen::MatrixXd Core::color_points(const Mesh& mesh)const{
    std::cout << "colot point with m_color_type " << m_color_type << '\n';
    Eigen::MatrixXd C = mesh.V;
    double min_y, max_y;
    min_y = mesh.V.col(1).minCoeff();
    max_y = mesh.V.col(1).maxCoeff();
    max_y=std::min(max_y, (double)m_cap_max_y); //cap the max y so we can see the colors better in the case one point has very high Y coord

    if(m_color_type==0){
        for (size_t i = 0; i < C.rows(); i++) {
            std::vector<float> color_vec = jet_color(mesh.V(i, 1), min_y, max_y);
            C(i, 0) = color_vec[0];
            C(i, 1) = color_vec[1];
            C(i, 2) = color_vec[2];
        }

    }else if(m_color_type==1){
        for (size_t i = 0; i < C.rows(); i++) {
             float gray_val = lerp(mesh.V(i,1), min_y, max_y, 0.0, 1.0 );
             C(i,0)=C(i,1)=C(i,2)=gray_val;
        }

    }else if(m_color_type==2){

        double min_d, max_d;
        min_d=mesh.D.minCoeff();
        max_d=mesh.D.maxCoeff();
        for (size_t i = 0; i < C.rows(); i++) {
            C(i,0)=C(i,1)=C(i,2)=  lerp(mesh.D(i), min_d, max_d, 0.0, 1.0 );;
        }

    }else if(m_color_type==3){
        Eigen::VectorXi C_idx(mesh.V.rows());
        C_idx.setLinSpaced(mesh.V.rows(), 0 ,mesh.V.rows()-1);
        C = C_idx.template cast<double>().replicate(1,3);
        //normalize the colors
        double max = C.maxCoeff();
        double min = C.minCoeff();
        double range = std::fabs(max) + std::fabs(min);
        double inv_range = 1.0/range;
        C.array() = (C.array() - min) *inv_range;

    //AO
    }else if(m_color_type==4){
        //TODO
        // Eigen::VectorXd ao;
        // m_fuser->ambient_occlusion(ao, m_scene, 100);
        // C = ao.replicate(1,3);

    //default
    }else if(m_color_type==5) {
        C.col(0).setConstant(0.41);
        C.col(1).setConstant(0.58);
        C.col(2).setConstant(0.59);
    }

    return C;
}

void Core::write_ply(){
    strcat (m_exported_filename,".ply");
    igl::writePLY(m_exported_filename, m_scene.V, m_scene.F);
}

void Core::write_obj(){
    strcat (m_exported_filename,".obj");
    igl::writeOBJ(m_exported_filename, m_scene.V, m_scene.F);
}

void Core::read_pose_file(){
    std::ifstream infile( "/media/alex/Data/Master/SHK/c_ws/src/laser_mesher/data/graph_viewer_scan_poses_00.txt" );
    uint64_t scan_nr;
    uint64_t timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond quat;


    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        iss >> scan_nr >> timestamp
            >> position(0) >> position(1) >> position(2)
            >> quat.w() >> quat.x() >> quat.y() >> quat.z();
//        std::cout << "input is \n" << scan_nr << " " << timestamp << " " << position << " " << quat.matrix()  << "\n";
        Eigen::Affine3d pose;
        pose.matrix().block<3,3>(0,0)=quat.toRotationMatrix();
        pose.matrix().block<3,1>(0,3)=position;
        timestamp=(uint64_t)std::round(timestamp/100000.0); ////TODO this is a dirty hack to reduce the discretization of time because the timestamps don't exactly match
//        VLOG(2) << "recorded tmestamp is " << timestamp;
//        VLOG(2) << "recorded scan_nr is " << scan_nr;
        m_worldROS_baselink_map[timestamp]=pose;
    }

}

bool Core::get_pose_at_timestamp(Eigen::Affine3d& pose, uint64_t timestamp){

    auto got = m_worldROS_baselink_map.find (timestamp);

    if ( got == m_worldROS_baselink_map.end() ){
        LOG(WARNING) << "get_pose: pose query for the scan does not exist at timestamp" << timestamp;
        return false;
    }else{
        pose = got->second;
//        VLOG(2) << "returning pose at scan_nr  \n" << pose.matrix();
        return true;
    }

}


void Core::create_transformation_matrices(){


    /*
     *  All frames are right handed systems
     *  The gap of the laser is always on the far side of the camera, somewhere on the negative Z axis in the alg frame
     *
     *                                                                                  Y
     *                                                                                  |
     *                                                                                  |
     *                                                                                  |
     *                                                                                  |
     *                                                                                  |
     *  Y-------                                        --->                            ---------- X
     *         /|                                                                      /
     *       /  |                                                                    /
     *     /    |                                                                  /
     *    X     |                                                                 Z
     *          |
     *          Z
     * Velodyne frame represented in the odom frame                             Algorithm frame (and also how the GL frame looks like)
     * This is what we see in rviz                                              Frame which the algorithms in the Mesher use to create the mesh
     * X looking towards camera
     * Z downwards
     * Y to the left
     *
     * The transformation between these two is what the function called fix_cloud_orientation was doing:
     *  Reminder:
     *  cloud->points[idx].x = -y;
     *  cloud->points[idx].y = -z;
     *  cloud->points[idx].z = x;
     *
     * The matrix correspinding to the transformation will be only a rotation matrix
     *  since we only rotate around the origin and don't do any translation
     *
     * Positives angles of rotiation here:
     *   https://stackoverflow.com/questions/31191752/right-handed-euler-angles-xyz-to-left-handed-euler-angles-xyz
     *
     * */

    m_tf_alg_vel.setIdentity();
    Eigen::Matrix3d alg_vel_rot;
    // rot = Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())  //this rotation is done second and rotates around the Z axis of the velodyne frame but after it was rotated by the first rotation.
    //   * Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitY());   //this rotation is done first. Performed on the Y axis of the velodyne frame (after this the y is pointing left, x is up and z is towards the camera)
    // // m_tf_alg_vel.matrix().block<3,3>(0,0)=rot.transpose();

    alg_vel_rot = Eigen::AngleAxisd(-0.5*M_PI+M_PI, Eigen::Vector3d::UnitY())  //this rotation is done second and rotates around the Y axis of alg frame
      * Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX());   //this rotation is done first. Performed on the X axis of alg frame (after this the y is pointing towards camera, x is right and z is down)
    m_tf_alg_vel.matrix().block<3,3>(0,0)=alg_vel_rot;



    m_tf_baselink_vel.setIdentity();
    Eigen::Vector3d baselink_vel_t(-0.000, -0.000, -0.177);
    // Eigen::Quaterniond baselink_vel_quat(-0.692, 0.722, -0.000, -0.000);

    //TODO the quaternion didn't quite work here
    Eigen::AngleAxisd rollAngle(-3.142, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(0.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(-1.614, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> baselink_vel_quat = pitchAngle * yawAngle * rollAngle;

    m_tf_baselink_vel.matrix().block<3,3>(0,0)=baselink_vel_quat.toRotationMatrix();
    m_tf_baselink_vel.matrix().block<3,1>(0,3)=baselink_vel_t;



    /*
     *
     *
     *           Z
     *           |
     *           |       X
     *           |     /
     *           |   /
     *           | /
     *   Y-------
     *

     * ROS world frame
     * Explained here: http://www.ros.org/reps/rep-0103.html
     *
     * */


     m_tf_worldGL_worldROS.setIdentity();
     Eigen::Matrix3d worldGL_worldROS_rot;
     worldGL_worldROS_rot = Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitX());
     m_tf_worldGL_worldROS.matrix().block<3,3>(0,0)=worldGL_worldROS_rot;




}
