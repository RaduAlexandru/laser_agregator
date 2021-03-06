#include "laser_agregator/Mesher.h"

//c++
#include <iostream>

//my stuff
#include "laser_agregator/MiscUtils.h"
#include "laser_agregator/Profiler.h"
#include "laser_agregator/Edge.h"
#include "laser_agregator/RosTools.h"
#include "laser_agregator/triangle_utils.h"

//libigl
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/triangle/triangulate.h>
#include <igl/qslim.h>
#include <igl/remove_unreferenced.h>
#include <igl/decimate.h>
#include <igl/edge_lengths.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_border_vertex.h>
#include <igl/edge_flaps.h>
#include <igl/ramer_douglas_peucker.h>
#include "igl/LinSpaced.h"
#include "igl/find.h"
#include "igl/cumsum.h"
#include "igl/histc.h"
#include "igl/slice.h"
#include "igl/project_to_line.h"
#include "igl/EPS.h"
#include "igl/slice_mask.h"

//loguru
#include <loguru.hpp>


Mesher::Mesher() :
        m_mesh_is_modified(false),
        m_meshes(NUM_MESHES_BUFFER),
        m_finished_mesh_idx(-1),
        m_working_mesh_idx(0),
        m_smoothing_iters(4),
        m_smoothing_stepsize(-0.80),
        m_smoothing_max_movement(0.06),
        m_normal_thresh(0.989),
        m_edge_merge_thresh(0.1),
        m_show_as_image(false),
        m_triangle_area(1000.0),
        m_triangle_angle(0.0),
        m_show_delaunay(true),
        m_min_length_horizontal_edge(0),
        m_max_length_horizontal_edge(500),
        m_triangle_silent(true),
        m_triangle_fast_arithmetic(true),
        m_triangle_robust_interpolation(true),
        m_edge_grazing_angle_thresh_horizontal(0.9),
        m_edge_grazing_angle_thresh_vertical(0.9),
        m_min_grazing(0.1),
        m_max_tri_length(6.5),
        m_min_tri_quality(0.015),
        m_create_faces(true),
        m_improve_mesh(true),
        m_adaptive_edge_length(true),
        m_do_random_edge_stopping(true),
        m_random_edge_stopping_thresh(0.8),
        m_compute_naive_mesh(false){

    init_params();


}

void Mesher::init_params(){
    ros::NodeHandle private_nh("~");

    //triangle params
    m_triangle_silent = getParamElseDefault<bool>(private_nh, "triangle_silent", true);
    m_triangle_fast_arithmetic = getParamElseDefault<bool>(private_nh, "triangle_fast_arithmetic", true);
    m_triangle_robust_interpolation = getParamElseDefault<bool>(private_nh, "triangle_robust_interpolation", true);

}


void Mesher::compute_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    if(m_compute_naive_mesh){
        naive_mesh(cloud);
    }else{
        simplify(cloud);
    }
}


void Mesher::simplify(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    LOG_SCOPE(INFO, "create_mesh");
    TIME_SCOPE("create_mesh");


    last_cloud = cloud;
    Mesh &mesh = m_meshes[m_working_mesh_idx];
    mesh.clear();




    //attempt 2 to create a simplified mesh, this time with only edge based confidences
//    create_simple_mesh2(mesh, cloud);  //TODO substitute it with just the intiialization of V and D because we don't use these faces at all
    mesh.width = cloud->width;  //intended to make the mesh have size 16x1800
    mesh.height = cloud->height;
    mesh.V = pcl2eigen(cloud).leftCols(3);
    mesh.D = pcl2eigen(cloud).rightCols(1);
    smooth_mesh(mesh);

    row_type_b is_vertex_an_edge_endpoint;
    // mesh.E  = create_edges(mesh,is_vertex_an_edge_endpoint);
    mesh.E  = create_edges_douglas_peucker(mesh,is_vertex_an_edge_endpoint);

    // create_naive_mesh(mesh, cloud);
    delaunay(mesh, is_vertex_an_edge_endpoint);


    if(m_improve_mesh) improve_mesh(mesh);
    igl::per_face_normals(mesh.V, mesh.F, mesh.N_faces);
    remove_faces_with_low_confidence(mesh);


    remove_unreferenced_verts(mesh);
    igl::per_vertex_normals(mesh.V, mesh.F, igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE, mesh.NV);

    mesh.sanity_check();

    if(mesh.V.rows()!=mesh.D.rows()){
        LOG(WARNING) << "If the mesh has more vertices after delaunay it may mean that some of the red edges are overlapping. \n This most likely happened because the \"gap\" in the laser is not set properly. You may need to fiddle with the view direction and view width in the launch file. \n If you look at the mesh in the algorithm frame (before applying all your transformations after meshing) it the gap should be furthest away from the camera, along the negative Z axis. If not then you need to rotate the mesh by 90 degrees around the Y axis by using the m_tf_alg_vel from the Core (see function create_transformation_matrices)";
    }


    m_finished_mesh_idx = m_working_mesh_idx;
    m_working_mesh_idx = (m_working_mesh_idx + 1) % NUM_MESHES_BUFFER;
    m_mesh_is_modified = true;

}

void Mesher::naive_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    last_cloud = cloud;
    Mesh &mesh = m_meshes[m_working_mesh_idx];
    mesh.clear();

    create_naive_mesh(mesh, cloud);
    VLOG(1) << "after create naive mesh we have nr of verts" << mesh;

    if(m_improve_mesh) {
        improve_mesh(mesh);
    }
    VLOG(1) << "after improve_mesh we have nr of verts" << mesh;
    igl::per_face_normals(mesh.V, mesh.F, mesh.N_faces);
    //for some reason the normals are flipped so we unflip them
    mesh.N_faces=-mesh.N_faces;
    remove_faces_with_low_confidence(mesh);
    VLOG(1) << "after remove_faces_with_low_confidence we have nr of verts" << mesh;

    remove_unreferenced_verts(mesh);
    VLOG(1) << "after remove_unreferenced_verts we have " << mesh;
    igl::per_vertex_normals(mesh.V, mesh.F, mesh.NV);
    //for some reason the normals are flipped so we unflip them
    mesh.NV=-mesh.NV;



    mesh.sanity_check();
    m_finished_mesh_idx = m_working_mesh_idx;
    m_working_mesh_idx = (m_working_mesh_idx + 1) % NUM_MESHES_BUFFER;
    m_mesh_is_modified = true;
}

void Mesher::just_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    last_cloud = cloud;
    Mesh &mesh = m_meshes[m_working_mesh_idx];
    mesh.clear();

    mesh.width = cloud->width;  //intended to make the mesh have size 16x1800
    mesh.height = cloud->height;
    mesh.V = pcl2eigen(cloud).leftCols(3);
    mesh.D = pcl2eigen(cloud).rightCols(1);


    mesh.sanity_check();
    m_finished_mesh_idx = m_working_mesh_idx;
    m_working_mesh_idx = (m_working_mesh_idx + 1) % NUM_MESHES_BUFFER;
    m_mesh_is_modified = true;

}

void Mesher::remove_unreferenced_verts(Mesh& mesh){
    //remove unreferenced vertices to get rid also of the ones at 0,0,0
    row_type_b is_vertex_referenced(mesh.V.rows(),false);
    for (int i = 0; i < mesh.F.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            int idx=mesh.F(i,j);
            is_vertex_referenced[idx]=true;
        }
    }
    row_type_i V_indir;
    mesh.V=filter_return_indirection(V_indir, mesh.V, is_vertex_referenced,true);
    mesh.D=filter(mesh.D, is_vertex_referenced,true);
    mesh.F=filter_apply_indirection(V_indir, mesh.F);
    mesh.E=filter_apply_indirection(V_indir, mesh.E);
}



/*Grabs a point cloud and lays down the points in a row by row manner starting from the top-left. */
Eigen::MatrixXd Mesher::pcl2eigen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    int num_points = cloud->width*cloud->height;

    Eigen::MatrixXd V_d(num_points, 4);  //xyz and distance to sensor
    V_d.setZero();
    // std::cout << "creating a V_d matrix of size " << V_d.rows() << " " << V_d.cols() << '\n';
    // std::cout << "cloud has size " << cloud->width << " " << cloud->height << '\n';

    for (int x_idx = 0; x_idx < cloud->width; x_idx++) {
        for (int y_idx = 0; y_idx < cloud->height; y_idx++) {
            unsigned int idx = y_idx + x_idx * cloud->height;
            // std::cout << "idx is  " << idx << '\n';
            int insertion_idx=x_idx + y_idx * cloud->width; //inserts the points row by row (and not column by column) and going from left to right is you're looking from the sensor position
            // std::cout << "insertion_idx is  " << insertion_idx << '\n';
            if (!std::isnan(cloud->points[idx].x) && !std::isnan(cloud->points[idx].y) && !std::isnan(cloud->points[idx].z)) {
                //insert the point with a different structure so that we have the points not in a cloud of 1800x16 and starting from the upper right but rather of size 16x1800 and starting from bottom left
                // std::cout << "insertin row at " << insertion_idx  << '\n';

                Eigen::Vector3d pos;
                pos << cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z;
                float dist=pos.norm(); //can do this because we have it velodyne frame and the 0,0,0 is in the center
                V_d.row(insertion_idx)(0)=pos(0);
                V_d.row(insertion_idx)(1)=pos(1);
                V_d.row(insertion_idx)(2)=pos(2);
                V_d.row(insertion_idx)(3)=dist;

                 // << cloud->points[idx].x,
                 //        cloud->points[idx].y,
                 //        cloud->points[idx].z,
                 //        0.0; //TODO set to 0 because not we don't have distance
            } else {
                //TODO better handling of invalid points
                // std::cout << "insertin 0s at row " << insertion_idx  << '\n';
                V_d.row(insertion_idx) << 0.0, 0.0, 0.0, 0.0;
            }


        }
    }

    return V_d;

}


void Mesher::smooth_mesh(Mesh& mesh){
    LOG_SCOPE(INFO, "smooth_mesh");


//    Eigen::SparseMatrix<double> L;
//    for (int i = 0; i < m_smoothing_iters; ++i) {
//        igl::cotmatrix(mesh.V,mesh.F,L);
//        mesh.V= mesh.V + m_smoothing_stepsize*L*mesh.V;
//    }

    //another way to do it is to smooth each laser ring separately and smooth only the distance to the sensor value of each of them
    //The intuition is that since the laser has a large difference in the vertical angle, the points in different rings don't necessarily lie on the same surface
    //therefore they wouldn't need to influence each other. However in the same ring the points are quite close together and should probably have similar distances to the sensor


    //brute force without a laplacian
    for (int s_iter = 0; s_iter < m_smoothing_iters; ++s_iter) {
        Eigen::MatrixXd delta_D =mesh.D;
        delta_D.setZero();
        for (int x_idx = 1; x_idx < mesh.width-1; x_idx++) {
            for (int y_idx = 0; y_idx < mesh.height; y_idx++) {


                unsigned int idx_0 = x_idx -1 + y_idx * mesh.width;
                unsigned int idx_1 = x_idx    + y_idx * mesh.width;
                unsigned int idx_2 = x_idx +1 + y_idx * mesh.width;

                //skips the points where one of them is 0
                if (mesh.V.row(idx_0).isZero() || mesh.V.row(idx_1).isZero() || mesh.V.row(idx_2).isZero()){
                    delta_D(idx_1)=0.0;
                }else{
                    //points idx1 should a distance that is the average between idx0 and idx2
                    double movement = mesh.D(idx_1) - (mesh.D(idx_0) + mesh.D(idx_2))/2;
                    if (std::fabs(movement) > m_smoothing_max_movement){ //If the movement is too much, then the points might be on and edge and it shouldt move
                        delta_D(idx_1)=0.0;
                    }else{
                        delta_D(idx_1)=movement;
                    }

                }



            }
        }

        //apply
        for (int i = 0; i < mesh.V.rows(); ++i) {
            mesh.D(i) = mesh.D(i) + m_smoothing_stepsize*delta_D(i);
        }

    }


    //In order for the smoothing to take effect ont he vertices, we need to put the mesh as an image and then back again
//    //TODO, not the most efficient way because maybe we can just substract the D from each vertex (again with some fancy trigonometry)
//    mesh.to_image();
//    mesh.to_mesh();

    mesh.apply_D();



}


void Mesher::improve_mesh(Mesh& mesh){
    TIME_SCOPE("improve_mesh");

    Eigen::MatrixXi E, EF, EV, EI;
    Eigen::VectorXi EMAP_vec;

    igl::edge_flaps(mesh.F, E, EMAP_vec, EF, EI); // boundary has -1 EF and EV;

    // from EI to EV (maps each ege to the adyacent corner vertices on the two flaps)
    EV = EI;
    for(size_t e=0; e<EV.rows(); e++){
      for(auto i:{0,1}){
          if(EI(e,i)!= -1){
              EV(e,i) = mesh.F(EF(e,i), EI(e,i));
          }
      }
    }

    triangle_improving_edge_flip(mesh.V, mesh.F,E,EF,EV,EMAP_vec);
}


void Mesher::remove_faces_with_low_confidence(Mesh& mesh){

    //get how much each faces normal deviates from the ideal one (pointing toward the sensor pose)
    //ideally it would be 1.0, in the worse case it is 0.0
    std::vector<bool> face_too_grazing(mesh.F.rows(),false);
    Eigen::Vector3d sensor_origin=mesh.sensor_pose.translation();
    for (int i = 0; i < mesh.F.rows(); ++i) {
        Eigen::Vector3d normal= mesh.N_faces.row(i);
        Eigen::Vector3d dir= (sensor_origin - Eigen::Vector3d(mesh.V.row(mesh.F(i,0)))).normalized();  //It's a minus becase its origin(0,0,0) - point
        double dot=dir.dot(normal);
        if(dot < m_min_grazing){
            face_too_grazing[i]=true;
        }
    }

    Eigen::MatrixXd edge_lenghts;
    igl::edge_lengths(mesh.V, mesh.F, edge_lenghts);
    std::vector<bool> face_too_streched(mesh.F.rows(),false);
    for (int i = 0; i < mesh.F.rows(); ++i) {
        for (int j = 0; j < mesh.F.cols(); ++j) {
            if(edge_lenghts(i,j) > m_max_tri_length){
                face_too_streched[i]=true;
            }
        }
    }


    // //remove silvers (unfortunatelly some triangles on close walls are aleady silvers and I don't want to remove them)
    // Eigen::VectorXd double_areas;
    // Eigen::MatrixXd squared_edge_lenghts;
    // igl::doublearea(mesh.V, mesh.F, double_areas);
    // igl::squared_edge_lengths(mesh.V, mesh.F, squared_edge_lenghts);
    // Eigen::VectorXd faces_quality(mesh.F.rows());
    // faces_quality.setZero();
    // faces_quality= 2*sqrt(3)* double_areas.array() / squared_edge_lenghts.rowwise().sum().array();
    // std::vector<bool> face_low_quality(mesh.F.rows(),false);
    // for (int i = 0; i < mesh.F.rows(); ++i) {
    //     if(faces_quality(i) < m_min_tri_quality ){
    //         face_low_quality[i]=true;
    //     }
    // }

    std::vector<bool> is_face_low_confidence(mesh.F.rows(),false);
    for (int i = 0; i < mesh.F.rows(); ++i) {
        if(face_too_grazing[i] || face_too_streched[i] ){
            is_face_low_confidence[i]=true;
        }
    }

    mesh.F=filter(mesh.F, is_face_low_confidence, false);
    mesh.N_faces=filter(mesh.N_faces, is_face_low_confidence, false);


}





bool Mesher::compute_non_manifold_edges(std::vector<bool>& is_face_non_manifold, std::vector<bool>& is_vertex_non_manifold,  const Eigen::MatrixXi& F_in){
    // List of edges (i,j,f,c) where edge i<j is associated with corner i of face
    // f

    is_face_non_manifold.resize(F_in.rows(),false);
    int nr_vertices=F_in.maxCoeff()+1;
    is_vertex_non_manifold.resize(nr_vertices,false);
    std::vector<std::vector<int> > TTT;
    for(int f=0;f<F_in.rows();++f)
        for (int i=0;i<3;++i)
        {
            // v1 v2 f ei
            int v1 = F_in(f,i);
            int v2 = F_in(f,(i+1)%3);
            if (v1 > v2) std::swap(v1,v2);
            std::vector<int> r(4);
            r[0] = v1; r[1] = v2;
            r[2] = f;  r[3] = i;
            TTT.push_back(r);
        }
    // Sort lexicographically
    std::sort(TTT.begin(),TTT.end());

    bool is_edge_manifold=true;
    for(int i=2;i<(int)TTT.size();++i)
    {
        // Check any edges occur 3 times
        std::vector<int>& r1 = TTT[i-2];
        std::vector<int>& r2 = TTT[i-1];
        std::vector<int>& r3 = TTT[i];
        if ( (r1[0] == r2[0] && r2[0] == r3[0])
             &&
             (r1[1] == r2[1] && r2[1] == r3[1]) )
        {
            std::cout << "non manifold around the 3 faces (v_idx, v_idx, f_idx, e_idx)"  << std::endl;
            std::cout << " edge_1 " << r1[0] << " " << r1[1] << " " << r1[2] << " " << r1[3] << std::endl;
            std::cout << " edge_2 " << r2[0] << " " << r2[1] << " " << r2[2] << " " << r2[3] << std::endl;
            std::cout << " edge_3 " << r3[0] << " " << r3[1] << " " << r3[2] << " " << r3[3] << std::endl;
            is_face_non_manifold[r1[2]]=true;
            is_face_non_manifold[r2[2]]=true;
            is_face_non_manifold[r3[2]]=true;

            is_vertex_non_manifold[r1[0]]=true;
            is_vertex_non_manifold[r1[1]]=true;
            is_vertex_non_manifold[r2[0]]=true;
            is_vertex_non_manifold[r2[1]]=true;
            is_vertex_non_manifold[r3[0]]=true;
            is_vertex_non_manifold[r3[1]]=true;


            is_edge_manifold=false;
        }
    }
    return is_edge_manifold;
}



void Mesher::create_naive_mesh(Mesh &mesh, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ) {

    LOG_SCOPE(INFO, "create_naive_mesh");
    TIME_SCOPE("create_naive_mesh");

    mesh.width = cloud->width;  //intended to make the mesh have size 16x1800
    mesh.height = cloud->height;
    mesh.V = pcl2eigen(cloud).leftCols(3);
    mesh.D = pcl2eigen(cloud).rightCols(1);
    std::vector<int> indices;
    indices.reserve(40000); //reserve some space so we speed up the pushback


    /*
    current points is at index 0 and we form 2 triangle coming out of it. The indexes are:

    0-------1
    |     /
    |   /
    | /
    2
    triangle 1


           5
         / |
       /   |
     /     |
    3-------4
    triangle 2
    */

    //the index needs to be offset to account for all the other vertices that are in the vbo that we will not render

    for (int x_idx = 0; x_idx < mesh.width; x_idx++) {
        for (int y_idx = 0; y_idx < mesh.height; y_idx++) {

            bool trig_1_invalid = false;
            bool trig_2_invalid = false;

            //get rid of the triangle if any of the coordinates is outside the image
            if ((x_idx + 1) > mesh.width - 1 || (y_idx + 1) > mesh.height - 1) {
                trig_1_invalid = true;
                trig_2_invalid = true;
                continue;
            }

            //triangle 1
            unsigned int idx_0 = x_idx + y_idx * mesh.width;
            unsigned int idx_1 = x_idx + 1 + y_idx * mesh.width;
            unsigned int idx_2 = x_idx + (y_idx + 1) * mesh.width;

            //triangle 2
            unsigned int idx_3 = x_idx + (y_idx + 1) * mesh.width;
            unsigned int idx_4 = x_idx + 1 + (y_idx + 1) * mesh.width;
            unsigned int idx_5 = x_idx + 1 + y_idx * mesh.width;




            //get rid of the triangle if any of the points is invalid
            if (mesh.V.row(idx_0).isZero() || mesh.V.row(idx_1).isZero() || mesh.V.row(idx_2).isZero()) {
                trig_1_invalid = true;
            }
            //triangle 2
            if (mesh.V.row(idx_3).isZero() || mesh.V.row(idx_4).isZero() || mesh.V.row(idx_5).isZero()) {
                trig_2_invalid = true;
            }




            //if we reached here, add the tirangle
            if (!trig_1_invalid) {
                indices.push_back(idx_1);
                indices.push_back(idx_0);
                indices.push_back(idx_2);
                // std::cout << "pushing " << idx_1 << " " << idx_0 << " " << idx_2 << '\n';
            }

            if (!trig_2_invalid) {
                indices.push_back(idx_3);
                indices.push_back(idx_4);
                indices.push_back(idx_5);
                // std::cout << "pushing " << idx_3 << " " << idx_4 << " " << idx_5 << '\n';
            }


        }

    }


    //from indices pass them to the eigen matrix
    mesh.F.resize(indices.size() / 3, 3);
    for (size_t i = 0; i < indices.size(); i = i + 3) {
        mesh.F.row(i / 3) << indices[i], indices[i + 1], indices[i + 2];
    }



}

Eigen::MatrixXi Mesher::create_edges(Mesh& mesh, row_type_b& is_vertex_an_edge_endpoint){

    LOG_SCOPE_F(INFO,"create_edges2");

    is_vertex_an_edge_endpoint.resize(mesh.V.rows(),false);

    Eigen::MatrixXi edges;
    std::vector<Edge> edges_vec;

    //go from left to right
    Edge edge (mesh);

    for (int y_idx = 0; y_idx < mesh.height; y_idx++) {
        for (int x_idx = 0; x_idx < mesh.width; x_idx++) {

            unsigned int idx = x_idx + y_idx * mesh.width;

            //first vertex is a border and we have no first point in the edge
            if (!mesh.V.row(idx).isZero()){
                if (!edge.has_start_idx()){
                    edge.set_start_idx(idx);
                }
                //we have already a first point and we add a new one if the direction made by that edge will not change much
                if (edge.has_start_idx() ){
                    Edge new_edge(mesh);
                    new_edge.set_start_idx( edge.get_end_idx() );
                    new_edge.set_end_idx( idx);
                    double sim= edge.similarity(new_edge);
                    if (sim > m_edge_merge_thresh){
                        edge.set_end_idx(idx);
                    }else{
                        //there is a discontinuity and we add this edge as a final one
                        // edges_vec.push_back(edge);
                        // //now the new edge has to become the current one since that is the one that start a new surface (there was a discontiuity)
                        // edge.copy(new_edge);
                        // edge.stop_and_continue();
                        // continue;


                        //attempt 2 in which we create adyacent points near the endpoints
                        //this edge has a discontinuity with the next one
                        //we create a poit ner the endpoint of this one and mark the next one so that it creates also an edge at the beggining
                        new_edge.need_to_split_at_beggining=true;
                        edge.need_to_split_at_finale=true;
                        edge.push_into_vec(edges_vec);
                        edge.copy(new_edge);
                        // edge.stop_and_continue();
                        continue;

                    }

                    //if we reach a certain maximum length also add it
                    double max_length_thresh;
                    if(m_adaptive_edge_length){
                        max_length_thresh=m_max_length_horizontal_edge/mesh.D(idx); //so the edges that are further away will be shorter
                    }else{
                        max_length_thresh=m_max_length_horizontal_edge;
                    }


                    if (edge.get_length()>max_length_thresh){
                        edge.push_into_vec(edges_vec);
                        edge.stop_and_continue();
                    }


                    //at each step we check if the edge should stop here randomly
                    if(m_do_random_edge_stopping){
                        float val=rand_float(0.0 ,1.0);
                        if(val>m_random_edge_stopping_thresh){
                            edge.push_into_vec(edges_vec);
                            edge.stop_and_continue();
                        }
                    }


                }
            }else{
                //we found a point that is not a border so we can just add whatever we have until now
                edge.push_into_vec(edges_vec);
                edge.invalidate();

            }


        }
        //we reached the edge of the cloud, and we are switching to the next row, add what ever edge you have until now
        edge.push_into_vec(edges_vec);
        edge.invalidate();

    }

    std::vector<Edge> edges_vec_filtered;
    for (int i = 0; i < edges_vec.size(); ++i) {
//
        //TODO may need to somehow unsweep and undo the points on edge
        if (edges_vec[i].get_length() > m_min_length_horizontal_edge && edges_vec[i].is_valid() && !edges_vec[i].is_at_grazing_angle(m_edge_grazing_angle_thresh_horizontal)){
            edges_vec_filtered.push_back(edges_vec[i]);

            //so that the degenerate edges with just 1 single point, even though they are deleted and not added as a red edge, the point should still be in the delunay triangulaton
            if(edges_vec[i].is_valid()){
                is_vertex_an_edge_endpoint[edges_vec[i].get_start_idx()]=true;
                is_vertex_an_edge_endpoint[edges_vec[i].get_end_idx()]=true;
            }
        }
    }


    //Go from edges_vec to an Eigen matrix of edge, add only those that have enough minimum length
    //convert from the edges vector to the eigen matrix
    edges.resize(edges_vec_filtered.size(), 2);
    edges.setZero();
    // VLOG(1) << "copy stuff into edges";
    for (size_t i = 0; i < edges_vec_filtered.size(); i ++) {
        edges.row(i)=edges_vec_filtered[i].get_eigen();
    }

    // //get all the points that the edges have swept (indexing into V_orig)
    // mesh.E_swept.resize(edges.rows(), row_type_i());
    // for (int i = 0; i < edges.rows(); ++i) {
    //     int start_idx=edges_vec_filtered[i].get_start_idx();
    //     int end_idx=edges_vec_filtered[i].get_end_idx();
    //     for (int j = start_idx; j <=end_idx ; ++j) {
    //         if(!mesh.V.row(j).isZero()){
    //             mesh.E_swept[i].push_back(j);
    //         }
    //     }
    // }

    return edges;

}

Eigen::MatrixXi Mesher::create_edges_douglas_peucker(Mesh& mesh, row_type_b& is_vertex_an_edge_endpoint){

    TIME_SCOPE("create_edges_douglas_peucker");

    is_vertex_an_edge_endpoint.resize(mesh.V.rows(),false);

    std::vector<Eigen::Vector2i> E_vec_final;

    struct Edge_douglas{
        Eigen::Vector2i edge;
        bool is_next_edge_valid=true;
        bool is_previous_edge_valid=true;
    };

    for (int y_idx = 0; y_idx < mesh.height; y_idx++) {
        Eigen::MatrixXd P(mesh.width,3); //a row of point from the scanline
        for (int x_idx = 0; x_idx < mesh.width; x_idx++) {
            unsigned int idx = x_idx + y_idx * mesh.width;
            P.row(x_idx)=mesh.V.row(idx);
        }

        //simplify it
        Eigen::MatrixXd new_points;
        Eigen::VectorXi J;
        igl::ramer_douglas_peucker(P, m_edge_merge_thresh, new_points, J );
        //J point into P and has the points that are taken as edges

        // std::cout << "P has size " << P.rows() << '\n';
        // std::cout << "new points has size " << new_points.rows() << '\n';

        //make edges
        std::vector<Eigen::Vector2i> E_vec;
        for (size_t i = 0; i < J.rows()-1; i++) {
            int idx_edge_start=J(i) + y_idx * mesh.width; // J(i) is just the x_idx so to get back to index into the cloud we add the y_idx * mesh.width
            int idx_edge_end=J(i+1) + y_idx * mesh.width;
            // if(!mesh.V.row( idx_edge_start ).isZero()){
            //     is_vertex_an_edge_endpoint[idx_edge_start]=true;
            // }
            // if(!mesh.V.row( idx_edge_end ).isZero()){
            //     is_vertex_an_edge_endpoint[ idx_edge_end ]=true;
            // }
            if(!mesh.V.row( idx_edge_start ).isZero() && !mesh.V.row( idx_edge_end ).isZero()){
                Eigen::Vector2i edge;
                edge << idx_edge_start, idx_edge_end;
                E_vec.push_back(edge);
                // is_vertex_an_edge_endpoint[J(i)]=true;
                // is_vertex_an_edge_endpoint[J(i+1)]=true;
            }
        }
        // std::cout << "E_vec has size " << E_vec.size() << '\n';
        if(E_vec.size()==0){
            continue;
        }

        //TODO set the is_vertex_an_edge_endpoint
        //set it already here before we remove some of the edges so we retain the points even though they are not connected by red edges
        for (size_t i = 0; i < E_vec.size(); i++) {
            //avoid adding the edpoint of small edges as they will most likely correspond to vegetation
            int edge_steps=std::abs( E_vec[i](1)-E_vec[i](0) );
            if(edge_steps>2){
                is_vertex_an_edge_endpoint[E_vec[i](0)]=true;
                is_vertex_an_edge_endpoint[E_vec[i](1)]=true;
            }
        }



        //filter by grazing direction
        std::vector<Edge_douglas> E_douglas_vec;
        for (size_t i = 0; i < E_vec.size(); i++) {
            Eigen::Vector3d dir = (mesh.V.row(E_vec[i](1)) - mesh.V.row(E_vec[i](0))).normalized();
            Eigen::Vector3d view = (Eigen::Vector3d(mesh.V.row(E_vec[i](0))) - mesh.sensor_pose.translation()).normalized();  //the
            double dot = dir.dot(view);
            if (std::fabs(dot)<=m_edge_grazing_angle_thresh_horizontal){  //ideally the dot will be close to 0 so the view and edge and orthogonal
                VLOG(4) << "not at grazing angle, perfectly good edge " << dot;
                Edge_douglas edge_douglas;
                edge_douglas.edge=E_vec[i];
                E_douglas_vec.push_back(edge_douglas);
            }
        }


        // //TODO set the is_vertex_an_edge_endpoint
        // //set it already here before we remove some of the edges so we retain the points even though they are not connected by red edges
        // for (size_t i = 0; i < E_douglas_vec.size(); i++) {
        //     is_vertex_an_edge_endpoint[E_douglas_vec[i].edge(0)]=true;
        //     is_vertex_an_edge_endpoint[E_douglas_vec[i].edge(1)]=true;
        // }

        // std::cout << "E_douglas_vec after filteirng for angle has size " << E_douglas_vec.size() << '\n';
        //get if the next or previous one are valid
        for (size_t i = 1; i < E_douglas_vec.size()-1; i++) {
            Edge_douglas& prev=E_douglas_vec[i-1];
            Edge_douglas& cur=E_douglas_vec[i];
            Edge_douglas& next=E_douglas_vec[i+1];

            int edge_steps=std::abs( E_douglas_vec[i].edge(1)-E_douglas_vec[i].edge(0) );
            if(edge_steps>m_max_length_horizontal_edge){
                //it it does too make steps no need to split it since it will be split automatically by the max edge length
                cur.is_previous_edge_valid=false;
                cur.is_next_edge_valid=false;
                continue;
            }

            // //check if from the previous we continue onto the cur
            // if(prev.edge(1)==cur.edge(0)){
            //     prev.is_next_edge_valid=true;
            //     cur.is_previous_edge_valid=true;
            // }
            //
            // //check if from the cur we continue onto the next
            // if(cur.edge(1)==next.edge(0)){
            //     cur.is_next_edge_valid=true;
            //     next.is_previous_edge_valid=true;
            // }
        }


        //split the edges depending if they have a valid next or prev
        std::vector<Eigen::Vector2i> E_vec_split;
        for (size_t i = 0; i < E_douglas_vec.size(); i++) {
            int edge_steps=std::abs( E_douglas_vec[i].edge(1)-E_douglas_vec[i].edge(0) );
            if(edge_steps > 3){
               //splitting ONLY when going into the next edge
               if(E_douglas_vec[i].is_next_edge_valid && !E_douglas_vec[i].is_previous_edge_valid ){
                   Eigen::Vector2i split_0;
                   Eigen::Vector2i split_1;
                   split_0 << E_douglas_vec[i].edge(0), E_douglas_vec[i].edge(1)-1;
                   split_1 << E_douglas_vec[i].edge(1)-1, E_douglas_vec[i].edge(1);
                   E_vec_split.push_back( split_0 );
                   E_vec_split.push_back( split_1 );
               }
               //splitting when continuing only from the previous edge
               if(E_douglas_vec[i].is_previous_edge_valid && !E_douglas_vec[i].is_next_edge_valid){
                   Eigen::Vector2i split_0;
                   Eigen::Vector2i split_1;
                   split_0 << E_douglas_vec[i].edge(0), E_douglas_vec[i].edge(0)+1;
                   split_1 << E_douglas_vec[i].edge(0)+1, E_douglas_vec[i].edge(1);
                   E_vec_split.push_back( split_0 );
                   E_vec_split.push_back( split_1 );
               }
               //splitting when both are valid
               if(E_douglas_vec[i].is_previous_edge_valid && E_douglas_vec[i].is_next_edge_valid){
                   Eigen::Vector2i split_0;
                   Eigen::Vector2i split_1;
                   Eigen::Vector2i split_2;
                   split_0 << E_douglas_vec[i].edge(0), E_douglas_vec[i].edge(0)+1;
                   split_1 << E_douglas_vec[i].edge(0)+1, E_douglas_vec[i].edge(1)-1;
                   split_2 << E_douglas_vec[i].edge(1)-1, E_douglas_vec[i].edge(1);
                   E_vec_split.push_back( split_0 );
                   E_vec_split.push_back( split_1 );
                   E_vec_split.push_back( split_2 );
               }

               //if no split can be done
               if(!E_douglas_vec[i].is_next_edge_valid && !E_douglas_vec[i].is_previous_edge_valid ){
                   Eigen::Vector2i split_0;
                   split_0 << E_douglas_vec[i].edge(0), E_douglas_vec[i].edge(1);
                   E_vec_split.push_back( split_0 );
               }
            }
        }


        //further split if the edges are still too big
        std::vector<Eigen::Vector2i> E_vec_split_smaller;
        for (size_t i = 0; i < E_vec_split.size(); i++) {
            int edge_steps=std::abs( E_vec_split[i](1)-E_vec_split[i](0) );

            // //if we reach a certain maximum length also add it
            double max_length_thresh;
            if(m_adaptive_edge_length){
                max_length_thresh=m_max_length_horizontal_edge/mesh.D(E_vec_split[i](0)); //so the edges that are further away will be shorter
            }else{
                max_length_thresh=m_max_length_horizontal_edge;
            }

            if(edge_steps > max_length_thresh){
                int nr_time_to_split=std::ceil(edge_steps/max_length_thresh);
                // int increment_of_each_sub_edge=m_max_length_horizontal_edge;
                int increment_of_each_sub_edge=std::ceil(edge_steps/nr_time_to_split);
                int start_idx=E_vec_split[i](0);
                int end_idx=E_vec_split[i](1);
                for (int s = 0; s < nr_time_to_split+1; s++) {
                    Eigen::Vector2i sub_edge;
                    sub_edge << start_idx +s*increment_of_each_sub_edge, std::min(start_idx +(s+1)*increment_of_each_sub_edge, end_idx);
                    if(sub_edge(0)!=sub_edge(1)){
                        E_vec_split_smaller.push_back(sub_edge);
                    }
                }
            }else{
                E_vec_split_smaller.push_back(E_vec_split[i]);
            }
        }


        //add the ones from this laser ring to the final edges
        for (size_t i = 0; i < E_vec_split_smaller.size(); i++) {
            E_vec_final.push_back(E_vec_split_smaller[i]);
        }


        // //TODO debug--------------   /add the ones from this laser ring to the final edges
        // for (size_t i = 0; i < E_douglas_vec.size(); i++) {
        //     E_vec_final.push_back(E_douglas_vec[i].edge);
        // }
        //
        // //TODO debug--------------   /add the ones from this laser ring to the final edges
        // for (size_t i = 0; i < E_vec.size(); i++) {
        //     E_vec_final.push_back(E_vec[i].edge);
        // }

    }


    //randomly split the edges of E_vec_final to improve the constraints for poisson
    std::vector<Eigen::Vector2i> E_vec_final_randomly_split;
    for (size_t i = 0; i < E_vec_final.size(); i++) {
        //check if we split this edge or not

        //at each step we check if the edge should stop here randomly
        if(m_do_random_edge_stopping){
            float val=rand_float(0.0 ,1.0);
            if(val>m_random_edge_stopping_thresh){
                //we check if the edge is long enough to be split
                int start_idx=E_vec_final[i](0);
                int end_idx=E_vec_final[i](1);
                if( std::abs(start_idx-end_idx)<2 ){
                    //edge is too small
                    E_vec_final_randomly_split.push_back(E_vec_final[i]);
                    // continue;
                }else{
                    int middle_idx=rand_int(start_idx+1,end_idx-1);
                    // int middle_idx=start_idx+std::abs(start_idx-end_idx)/2;
                    Eigen::Vector2i left_edge;
                    Eigen::Vector2i right_edge;
                    left_edge<< start_idx,middle_idx;
                    right_edge<< middle_idx, end_idx;

                    //push the two randomly split edges
                    E_vec_final_randomly_split.push_back(left_edge);
                    E_vec_final_randomly_split.push_back(right_edge);
                }
            }else{
                //it was not chosen for splitting so we just add the original one
                E_vec_final_randomly_split.push_back(E_vec_final[i]);
            }
        }else{
            //not doing any random stopping
            E_vec_final_randomly_split=E_vec_final;
        }
    }




    //TODO set the is_vertex_an_edge_endpoint
    for (size_t i = 0; i < E_vec_final_randomly_split.size(); i++) {
        is_vertex_an_edge_endpoint[E_vec_final_randomly_split[i](0)]=true;
        is_vertex_an_edge_endpoint[E_vec_final_randomly_split[i](1)]=true;
    }


    // std::cout << "E_vec has size " << E_vec.size() << '\n';
    // Eigen::MatrixXi E= vec2eigen(E_vec);
    // std::cout << "E has rows " << E.rows() << '\n';


    return vec2eigen(E_vec_final_randomly_split);
}

// void Mesher::ramer_douglas_peucker(const Eigen::MatrixXd P, double tol, Eigen::MatrixXd S, Eigen::VectorXi J){
//     // number of vertices
//     const int n = P.rows();
//
//     // Trivial base case
//     if(n <= 1){
//         J = Eigen::VectorXi::Zero(n);
//         S = P;
//         return;
//     }
//
//     // number of dimensions
//     const int m = P.cols();
//     Eigen::Array<bool,Eigen::Dynamic,1> I = Eigen::Array<bool,Eigen::Dynamic,1>::Constant(n,1,true);
//     const auto stol = tol*tol;
//
//     std::function<void(const int,const int)> simplify;
//     simplify = [&I,&P,&stol,&simplify](const int ixs, const int ixe)->void{
//         assert(ixe>ixs);
//         double sdmax = 0;
//         typename Eigen::Matrix<double,Eigen::Dynamic,1>::Index ixc = -1;
//         if((ixe-ixs)>1){
//             double sdes = (P.row(ixe)-P.row(ixs)).squaredNorm();
//             Eigen::Matrix<double,Eigen::Dynamic,1> sD;
//             const auto & Pblock = P.block(ixs+1,0,((ixe+1)-ixs)-2,P.cols());
//             if(sdes<=igl::EPS<double>()){
//                 sD = (Pblock.rowwise()-P.row(ixs)).rowwise().squaredNorm();
//             }else{
//                 Eigen::Matrix<double,Eigen::Dynamic,1> T;
//                 igl::project_to_line(Pblock,P.row(ixs).eval(),P.row(ixe).eval(),T,sD);
//             }
//             sdmax = sD.maxCoeff(&ixc);
//             // Index full P
//             ixc = ixc+(ixs+1);
//         }
//
//         if(sdmax <= stol){
//             if(ixs != ixe-1){
//                 I.block(ixs+1,0,((ixe+1)-ixs)-2,1).setConstant(false);
//             }
//         }else{
//             simplify(ixs,ixc);
//             simplify(ixc,ixe);
//         }
//     };
//
//     simplify(0,n-1);
//     igl::slice_mask(P,I,1,S);
//     igl::find(I,J);
// }

void Mesher::delaunay(Mesh& mesh, const row_type_b& is_vertex_an_edge_endpoint){
    LOG_SCOPE_F(INFO, "delaunay4");


    mesh.to_image(mesh.sensor_pose);
    mesh.to_2D();


    //get rid of the vertices that are not referenced by the edges
    row_type_i V_indir_decimate;
    mesh.V=filter_return_indirection(V_indir_decimate,mesh.V, is_vertex_an_edge_endpoint, true);
    mesh.D=filter(mesh.D, is_vertex_an_edge_endpoint, true);
    mesh.E=filter_apply_indirection(V_indir_decimate, mesh.E);


    // make some points at the borders because otherwise we would need to rely on the c flag of Triangle to make sure our triangulation doesn't dissapeet
    Eigen::MatrixXd border_points(4,2);
    border_points <<    -10.0, -10.0,
            10.0, -10.0,
            10.0, 10.0,
            -10.0, 10.0;

    //boundry points + rest
    Eigen::MatrixXd V_new(mesh.V.rows() + border_points.rows(), 2);
    V_new <<mesh.V, border_points ;

    //make edges at the boundry
    Eigen::MatrixXi E_boundry;
    E_boundry.resize(4, 2);
    E_boundry << V_new.rows()-1, V_new.rows()-2,
            V_new.rows()-2, V_new.rows()-3,
            V_new.rows()-3, V_new.rows()-4,
            V_new.rows()-4, V_new.rows()-1;
    Eigen::MatrixXi E_in=concat(mesh.E, E_boundry);


    std::string params="Q";
    Eigen::MatrixXi E_empty;
    Eigen::MatrixXd H_empty;
    Eigen::MatrixXi F_out;
    igl::triangle::triangulate(V_new, E_in ,H_empty ,params, mesh.V, mesh.F);


    //remove the last 4 point which are the boundary ones we added (when indirecting the faces they will also get removed)
    row_type_b  should_remove_vertex(mesh.V.rows(), false);
    std::fill(should_remove_vertex.end()- 4 , should_remove_vertex.end(), true);
    row_type_i V_indir;
    mesh.V=filter_return_indirection(V_indir, mesh.V, should_remove_vertex, false);
    mesh.F=filter_apply_indirection(V_indir, mesh.F);

    //debug
//    mesh.V=V_new;
//    mesh.E=E_boundry;
//    mesh.F.resize(0,0);


    mesh.to_3D();
    mesh.to_mesh(mesh.sensor_pose);


}




//TODO may introduce a racing condition because between changing the bool to false it may be changed again to true by the other thread
Mesh Mesher::get_mesh() {
    m_mesh_is_modified = false;

    if (m_show_as_image) {
        m_meshes[m_finished_mesh_idx].to_image();
    }

    return m_meshes[m_finished_mesh_idx];
}
