//C++
#include <iostream>

//My stuff
#include "laser_agregator/Core.h"
#include "laser_agregator/MiscUtils.h"
#include "laser_agregator/Profiler.h"
#include "laser_agregator/RosBagPlayer.h"
#include "laser_agregator/Mesher.h"
#include "laser_agregator/Agregator.h"
#include "laser_agregator/triangle_utils.h"

//libigl
#include <igl/viewer/Viewer.h>
#include <igl/readOBJ.h>
#include <igl/readPLY.h>
#include <igl/writePLY.h>
#include <igl/writeOBJ.h>
#include <igl/remove_unreferenced.h>
#include <igl/material_colors.h>
#include <igl/embree/ambient_occlusion.h>
#include <igl/embree/EmbreeIntersector.h>


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


//GL //for the Movies package to swap buffers
#include <GL/glad.h>
#include <GLFW/glfw3.h>

//decimate
#include <igl/qslim.h>


//boost
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;



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
        m_mesh_color(1.0, 215.0/255.0, 85.0/255.0),
        m_color_type(7),
        m_cap_max_y(20),
        m_nr_callbacks(0),
        m_animation_time(7),
        m_orbit_frame_counter(0),
        m_write_viewer_at_end_of_pipeline(false),
        m_magnification(1),
        m_decimation_nr_faces(10000),
        m_decimation_cost_thresh(0.5),
        m_skip(1){

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

        if(m_write_viewer_at_end_of_pipeline){
            fs::path dir (m_results_path);
            fs::path png_name (std::to_string(m_orbit_frame_counter)+".png");
            fs::path full_path = dir / png_name;
            fs::create_directory(dir);
            VLOG(2) << "core recording frame nr " << m_orbit_frame_counter;
            write_viewer_to_png(*m_view, full_path.string(), m_magnification);  //writing has to be done from the main thread because it calls opengl stuff
            m_orbit_frame_counter++;
        }else{
            m_orbit_frame_counter=0; // the recording was stopped so we set everything back to 0
        }

        if(m_agregator->is_modified() && m_player->is_paused() &&  m_player_should_continue_after_step){
            m_player_should_do_one_step=true; //so that when it starts the callback it puts the bag back into pause
            m_player->pause(); //starts the bag
        }

        if(m_agregator->is_modified()){  // no need to get the mesh again if we only m_visualization_should_change triggered the update
            m_scene=m_agregator->get_last_agregated_mesh();
        }


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
        if (m_show_edges) {
            set_edges(m_scene);
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
    m_pose_file = getParamElseThrow<std::string>(private_nh, "pose_file");
    m_view_direction = getParamElseThrow<float>(private_nh, "view_direction");
    m_bag_args = getParamElseThrow<std::string>(private_nh, "bag_args");

    m_exact_pose=getParamElseDefault<bool>(private_nh, "exact_pose", false);


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
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::PointCloud2> AsyncPolicy;
    // message_filters::Synchronizer<AsyncPolicy> sync(AsyncPolicy(10), image_sub, cloud_sub);
    // sync.registerCallback(boost::bind(&Core::callback, this, _1, _2));

    //only rgb
    // ros::Subscriber sub = private_nh.subscribe("/cam_img_0", 1000, &Core::callback, this);

    //only laser
    ros::Subscriber sub = private_nh.subscribe("/laser", 4, &Core::callback, this);



    ros::spin();

    m_player->kill();
    LOG(INFO) << "finished ros communication";

}

void Core::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // m_last_timestamp=(uint64_t)std::round(cloud_msg->header.stamp.toNSec()/100000000.0);
    m_last_timestamp=(uint64_t)cloud_msg->header.stamp.toNSec();

    LOG_S(INFO) << "calback----------------------" << m_last_timestamp;



    std::lock_guard<std::mutex> lock(m_mutex_recon);

    if(m_player_should_do_one_step){
        m_player_should_do_one_step=false;
        m_player->pause();
    }

    //skip every X of them
    if(m_nr_callbacks%m_skip!=0){
        if(m_player->is_paused() &&  m_player_should_continue_after_step){ //need to lso continue from here because the core will not update and therefore the bag will remain stopped
            m_player_should_do_one_step=true; //so that when it starts the callback it puts the bag back into pause
            m_player->pause(); //starts the bag
        }
        LOG_S(INFO) << "skip";
        m_nr_callbacks++;
        return;
    }


    //get the laser data into a point cloud
    pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*cloud_msg, *temp_cloud);
    pcl::PointCloud<PointXYZIDR>::Ptr cloud(new pcl::PointCloud<PointXYZIDR>);
    pcl::fromPCLPointCloud2(*temp_cloud, *cloud);



    // std::cout << "processing tietamp " << m_last_timestamp << '\n';
    Eigen::Affine3d sensor_pose;
    if (!get_pose_at_timestamp(sensor_pose,m_last_timestamp) ){
        LOG(WARNING) << "Not found any pose at timestamp " << m_last_timestamp << " Discarding mesh" << "check Gui->Misc->m_exact_pose pose for possible fix";
        if(m_player->is_paused() &&  m_player_should_continue_after_step){ //need to lso continue from here because the core will not update and therefore the bag will remain stopped
            m_player_should_do_one_step=true; //so that when it starts the callback it puts the bag back into pause
            m_player->pause(); //starts the bag
        }
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
    m_mesher->compute_mesh(cloud);
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



    // //write each local mesh into a file
    // std::string file_name="./all_local_meshes/" + std::to_string(m_nr_callbacks) + ".ply";
    // std::cout << "saving to " << file_name << '\n';
    // Eigen::MatrixXd UV_empty;
    // igl::writePLY(file_name, local_mesh.V, local_mesh.F, local_mesh.NV, UV_empty);




    m_agregator->agregate(local_mesh);






    // //Aggregate all of them
    // Eigen::MatrixXd V_new(V_all.rows() + V_clean.rows(), 3);
    // V_new << V_all, V_clean;
    // Eigen::MatrixXd NV_new(NV_all.rows() + NV.rows(), 3);
    // NV_new << NV_all, NV;
    // V_all=V_new;
    // NV_all=NV_new;

    // std::cout << "callback_agregate_all finished" << '\n';

    //by recording from here we will have a delay of one frame because this local mesh has not been updated in the viewew. Not too much of a problem though




    // if(m_player->is_paused() &&  m_player_should_continue_after_step){
    //     m_player_should_do_one_step=true; //so that when it starts the callback it puts the bag back into pause
    //     m_player->pause(); //starts the bag
    // }

    m_nr_callbacks++;

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
        m_mesher->compute_mesh(m_last_cloud);
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
        int num_samples=500;
        Eigen::VectorXd ao;
        igl::embree::EmbreeIntersector ei;
        ei.init(mesh.V.cast<float>(),mesh.F.cast<int>());
        Eigen::MatrixXd N_vertices;
        igl::per_vertex_normals(mesh.V, mesh.F, N_vertices);
        igl::embree::ambient_occlusion(ei, mesh.V, N_vertices, num_samples, ao);
        ao=1.0-ao.array(); //a0 is 1.0 in occluded places and 0.0 in non ocluded so we flip it to have 0.0 (dark) in occluded
        C = ao.replicate(1,3);

    //AO(Gold)
    }else if(m_color_type==5){
        int num_samples=500;
        Eigen::VectorXd ao;
        igl::embree::EmbreeIntersector ei;
        ei.init(mesh.V.cast<float>(),mesh.F.cast<int>());
        Eigen::MatrixXd N_vertices;
        igl::per_vertex_normals(mesh.V, mesh.F, N_vertices);
        igl::embree::ambient_occlusion(ei, mesh.V, N_vertices, num_samples, ao);
        ao=1.0-ao.array(); //a0 is 1.0 in occluded places and 0.0 in non ocluded so we flip it to have 0.0 (dark) in occluded

        VLOG(1) << "C is size " << C.rows() << " " << C.cols();
        VLOG(1) << "ao is size " << ao.rows() << " " << ao.cols();
        C.col(0).setConstant(m_mesh_color(0));
        C.col(1).setConstant(m_mesh_color(1));
        C.col(2).setConstant(m_mesh_color(2));
        VLOG(1) << "doing multiplication";
        // C=C.transpose().array().colwise()*ao.array(); // I dunno why it doesnt fucking work
        for (size_t i = 0; i < C.rows(); i++) {
            double ao_val=ao(i);
            for (size_t j = 0; j < C.cols(); j++) {
                C(i,j)=C(i,j)*ao_val;
            }
        }



    //default
    }else if(m_color_type==6) {
        C.col(0).setConstant(0.41);
        C.col(1).setConstant(0.58);
        C.col(2).setConstant(0.59);

    //GOLD
    }else if(m_color_type==7) {
        C.col(0).setConstant(m_mesh_color(0));
        C.col(1).setConstant(m_mesh_color(1));
        C.col(2).setConstant(m_mesh_color(2));
    }

    return C;
}

void Core::write_ply(){
    strcat (m_exported_filename,".ply");
    Eigen::MatrixXd UV_empty;
    if(m_scene.NV.size()){
        igl::writePLY(m_exported_filename, m_scene.V, m_scene.F, m_scene.NV, UV_empty);
    }else{
        igl::writePLY(m_exported_filename, m_scene.V, m_scene.F);
    }

}

void Core::write_obj(){
    strcat (m_exported_filename,".obj");
    igl::writeOBJ(m_exported_filename, m_scene.V, m_scene.F);
}

void Core::read_pose_file(){
    std::ifstream infile( m_pose_file );
    if( !infile  ) {
        LOG(ERROR) << "Can't open pose file";
    }

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
        m_timestamps_original_vec.push_back(timestamp);
        m_worldROS_baselink_vec.push_back(pose);
        timestamp=(uint64_t)std::round(timestamp/100000.0); ////TODO this is a dirty hack to reduce the discretization of time because the timestamps don't exactly match
        m_worldROS_baselink_map[timestamp]=pose;
        m_scan_nr_map[timestamp]=scan_nr;
    }

}

bool Core::get_pose_at_timestamp(Eigen::Affine3d& pose, uint64_t timestamp){

    //Brute force through all and choose the closest one
    int best_timestamp=-1;
    uint64_t closest_diff_time=9999999;
    int idx_best_timestamp=-1;
    for (size_t i = 0; i < m_timestamps_original_vec.size(); i++) {
        uint64_t timestamp_rounded=timestamp/1000.0; //need to reduce a bit the precision becuase in the file it's not that big
        uint64_t diff_time= std::abs(timestamp_rounded - m_timestamps_original_vec[i]);
        if(diff_time<closest_diff_time){
            closest_diff_time=diff_time;
            best_timestamp=m_timestamps_original_vec[i];
            idx_best_timestamp=i;
        }
    }
    std::cout << "best timestamp would be " << best_timestamp << " with closest diff at " <<  closest_diff_time << '\n';
    if(m_exact_pose){
        if(closest_diff_time!=0){
            return false;
        }
    }




    uint64_t timestamp_rounded=(uint64_t)std::round(timestamp/100000000.0);
    auto pose_pair = m_worldROS_baselink_map.find (timestamp_rounded);
    if ( pose_pair == m_worldROS_baselink_map.end() ){
        LOG(WARNING) << "get_pose: pose query for the scan does not exist at timestamp" << timestamp;
        return false;
    }else{
        pose = pose_pair->second;
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

    //make it here to be  Eigen::AngleAxisd(-0.5*M_PI+ M_PI, Eigen::Vector3d::UnitY()
    alg_vel_rot = Eigen::AngleAxisd(-0.5*M_PI+m_view_direction, Eigen::Vector3d::UnitY())  //this rotation is done second and rotates around the Y axis of alg frame
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


//from a mesh destroy half of the points at random (useful for visualizing jet colored point clouds that are very dense)
void Core::subsample_points(){
    row_type_b need_to_be_deleted(m_scene.V.rows(),false);
    for (size_t i = 0; i < m_scene.V.rows()/2; i++) {
        int idx_to_delete= rand_int(0, m_scene.V.rows()-1);
        need_to_be_deleted[idx_to_delete]=true;
    }
    row_type_i V_indir;
    m_scene.V=filter_return_indirection(V_indir, m_scene.V, need_to_be_deleted, false);
    m_scene.F=filter_apply_indirection(V_indir, m_scene.F);
}

void Core::write_single_png(){
    fs::path dir (m_results_path);
    fs::path png_name (m_single_png_filename);
    fs::path full_path = dir / png_name;
    fs::create_directory(dir);
    std::cout << " write_single_png: " << full_path << std::endl;

    write_viewer_to_png(*m_view, full_path.string(), m_magnification);

}

void Core::init_orbit(int& nr_steps, Eigen::Matrix3f& rotation_increment){
    int fps=25;
    nr_steps= fps*m_animation_time;   // I have 30 frames/second and x second at my disposal, how many frames will there be?

    //we have 360 degrees to take in nr_steps, how big should each rotation matrix be
    double angle_increment = 2*M_PI / nr_steps;
    rotation_increment = Eigen::AngleAxisf(angle_increment,  Eigen::Vector3f::UnitY());

}

void Core::orbit(){
    std::cout << "orbit_around_point" << '\n';

    int nr_steps;
    Eigen::Matrix3f rotation_increment;
    init_orbit(nr_steps, rotation_increment);

    for (size_t i = 0; i < nr_steps; i++) {
        make_incremental_step_in_orbit(rotation_increment);
    }

}

void Core::make_incremental_step_in_orbit(const Eigen::Matrix3f& rotation_increment){

    Eigen::Matrix3f new_rot=   m_view->core.trackball_angle.toRotationMatrix()* rotation_increment;
    Eigen::Quaternionf new_quat(new_rot);
    new_quat.normalize();
    m_view->core.trackball_angle=new_quat;
    m_view->draw();
    glfwSwapBuffers(m_view->window);
    // m_visualization_should_change=true;

}

void Core::write_orbit_png(){

    m_orbit_frame_counter=0;
    fs::path dir (m_results_path);
    fs::path png_name (std::to_string(m_orbit_frame_counter)+".png");
    fs::path full_path = dir / png_name;
    fs::create_directory(dir);

    int nr_steps;
    Eigen::Matrix3f rotation_increment;
    init_orbit(nr_steps, rotation_increment);
    for (size_t i = 0; i < nr_steps; i++) {
        make_incremental_step_in_orbit(rotation_increment);

        png_name = fs::path(std::to_string(m_orbit_frame_counter)+".png");
        full_path = dir / png_name;

        std::cout << "writeing viewer buffer to the png " << full_path << '\n';
        write_viewer_to_png(*m_view, full_path.string(), m_magnification );

        m_orbit_frame_counter++;
    }
    m_orbit_frame_counter=0;


}


void Core::decimate(Mesh& mesh, const int nr_target_faces, const float decimation_cost_thresh){

    // delete degenerate faces
    std::vector<bool> is_face_degenerate(mesh.F.rows(),false);
    double eps=1e-5;
    for (size_t i = 0; i < mesh.F.rows(); i++) {
        double dif_0= (mesh.V.row(mesh.F(i,0)) - mesh.V.row(mesh.F(i,1))).norm();
        double dif_1= (mesh.V.row(mesh.F(i,1)) - mesh.V.row(mesh.F(i,2))).norm();
        double dif_2= (mesh.V.row(mesh.F(i,2)) - mesh.V.row(mesh.F(i,0))).norm();
        if( dif_0 < eps || dif_1 < eps || dif_2 < eps ){
            is_face_degenerate[i]=true;
        }
    }
    mesh.F=filter(mesh.F, is_face_degenerate, false);

    //decimate it
    Eigen::VectorXi I;
    Eigen::VectorXi J;
    igl::qslim(mesh.V, mesh.F, nr_target_faces,  mesh.V, mesh.F, J,I);
}
