#include "laser_agregator/Mesh.h"

//c++
#include <iostream>
#include <algorithm>

//my stuff
#include "laser_agregator/MiscUtils.h"


Mesh::Mesh():
        sensor_pose(Eigen::Affine3d::Identity()),
        m_tf_currframe_alg(Eigen::Affine3d::Identity())
{
    clear();
}

void Mesh::add(const Mesh& new_mesh) {

    if(this->is_empty()){
        *this=new_mesh;
    }else{
        Eigen::MatrixXd V_new(V.rows() + new_mesh.V.rows(), 3);
        V_new << V, new_mesh.V;
        Eigen::MatrixXi F_new(F.rows() + new_mesh.F.rows(), 3);
        F_new << F, (new_mesh.F.array() + V.rows());
        Eigen::MatrixXd C_new(C.rows() + new_mesh.C.rows(), 3);
        C_new << C, new_mesh.C;
        Eigen::MatrixXi E_new(E.rows() + new_mesh.E.rows(), 2);
        E_new << E, (new_mesh.E.array() + V.rows());
        Eigen::MatrixXd D_new(D.rows() + new_mesh.D.rows(), 1);
        D_new << D, new_mesh.D;
        Eigen::MatrixXd N_faces_new(N_faces.rows() + new_mesh.N_faces.rows(), 3);
        N_faces_new << N_faces, new_mesh.N_faces;
        Eigen::MatrixXd NV_new(NV.rows() + new_mesh.NV.rows(), 3);
        NV_new << NV, new_mesh.NV;


        V = V_new;
        F = F_new;
        C = C_new;
        E = E_new;
        D = D_new;
        N_faces=N_faces_new;
        NV=NV_new;
    }




}

void Mesh::clear() {
    V.resize(0,0);
    F.resize(0,0);
    C.resize(0,0);
    E.resize(0,0);
    D.resize(0,0);
    N_faces.resize(0,0);
    NV.resize(0,0);
}


void Mesh::clear_C() {
    Eigen::MatrixXd C_empty;
    C = C_empty;
}


bool Mesh::is_empty() const {

    if (V.size() == 0) {
        return true;
    } else {
        return false;
    }


}

void Mesh::apply_transform(Eigen::Affine3d trans){

    //  V.transpose() = (trans.linear() * V.transpose()).colwise() + trans.translation();
    //The previous transform also moves the points at 0,0,0 which I consider invalid

    sensor_pose=trans*sensor_pose;
    for (size_t i = 0; i < V.rows(); i++) {
        if(!V.row(i).isZero()){
            V.row(i)=trans.linear()*V.row(i).transpose() + trans.translation();
        }
    }
    if(N_faces.size()) N_faces.transpose() = (trans.linear() * N_faces.transpose());
    if(NV.size()) NV.transpose() = (trans.linear() * NV.transpose());


    m_tf_currframe_alg=trans*m_tf_currframe_alg;

}

void Mesh::apply_D(){

    Eigen::Vector3d origin=sensor_pose.translation();

    for (int i = 0; i < V.rows(); ++i) {
        if (!V.row(i).isZero()){
            //get a direction going from the origin to the vertex
            Eigen::Vector3d dir_normalized = (Eigen::Vector3d(V.row(i)) - origin).normalized();
            //move along that direction with spedsize (D(i)) and then add the origin to get the position back in world coordinatees
            V.row(i) = dir_normalized*D(i) + origin;
        }

    }

}



//assuming that the fov is always 360 or always 16 will lead to errors then we do to_image or to_mesh, because the left most points in the image don't actually correspond to an angle of 0 but rather a veru small one
//void Mesh::compute_effective_fov(fov_h, fov_v, fov_h_start, fov_v_start){
//
//}

//TODO do the to image not with the coordinates of the points in the point cloud but rather the angle that the point have with respect to the axis
void Mesh::to_image() {

    to_image(m_tf_currframe_alg);

}

void Mesh::to_image(Eigen::Affine3d tf_currframe_alg) {

    //TODO can maybe be done more efficiently without the change of frame
    Eigen::MatrixXd V_alg_frame(V.rows(),3);
    Eigen::Affine3d tf_alg_currframe=tf_currframe_alg.inverse(); //this now goes from the current frame to the algorithm frame
    for (size_t i = 0; i < V.rows(); i++) {
        if(!V.row(i).isZero()){
            V_alg_frame.row(i)=tf_alg_currframe.linear()*V.row(i).transpose() + tf_alg_currframe.translation();  //mapping from the current frame to the algorithm one
        }
    }

    for (int i = 0; i < V.rows(); ++i) {
        if (!V_alg_frame.row(i).isZero()) {
            double r, theta, phi;
            r = V_alg_frame.row(i).norm();
            phi = std::atan2(V_alg_frame(i,0), -V_alg_frame(i,2));
            if (phi < 0.0) {     //atan goes from -pi to pi, it's easier to think of it going from 0 to 2pi
                phi += 2 * M_PI;
            }
            theta = (std::asin(V_alg_frame(i, 1) / r));
//            V.row(i) << phi, theta, r;
            V.row(i) << phi, theta, 0.0; // change back to r and not 0.0
            D(i)=r;  //store it back in D so that we can access it to_mesh (some points may not have D in the case when we fuse another mesh in it)

        }
    }

}

void Mesh::to_mesh() {

    to_mesh(m_tf_currframe_alg);

}

void Mesh::to_mesh(Eigen::Affine3d tf_currframe_alg) {

    //the previous one works for normal spherical coordinates but to take into account the fact that the points overlap at the gap we need to use this one
    for (int i = 0; i < V.rows(); ++i) {
        if(!V.row(i).isZero()){
            double r=D(i);
            double theta=V(i,1);
            double phi=V(i,0) - M_PI;   //since in to_image we added the 2m_pi here we subtract pi to put it back in the same place

            V.row(i)(0) = - r* std::cos(theta)*std::sin(phi);  //take cos(theta) instead of sin(theta) because it's the angle from the x axis not the y one
            V.row(i)(1) = r* std::sin(theta);
            V.row(i)(2) = r* std::cos(theta)*std::cos(phi);


            //TODO this should not be needed. We need to trace why exactly are there nans in the D vector when it gets returned from igl::triangulate
            if (!V.row(i).allFinite()){
                V.row(i) << 0.0, 0.0, 0.0;
                D(i) = 0;
            }

        }

    }

    for (size_t i = 0; i < V.rows(); i++) {
        if(!V.row(i).isZero()){
            V.row(i)=tf_currframe_alg.linear()*V.row(i).transpose() + tf_currframe_alg.translation();  //mapping from the current frame to the algorithm one
        }
    }

}


void Mesh::to_3D(){
  Eigen::MatrixXd V_new(V.rows(),3);
  V_new.setZero();
  V_new.leftCols(2)=V;
  V=V_new;
}

void Mesh::to_2D(){
    Eigen::MatrixXd V_new(V.rows(),2);
    V_new=V.leftCols(2);
    V=V_new;
}



void Mesh::sanity_check() const{
    LOG_IF_S(ERROR, V.rows()!=D.rows()) << "V and D don't coincide in size, they are " << V.rows() << " and " << D.rows();
    LOG_IF_S(ERROR, F.rows()!=N_faces.rows()) << "F and N_faces don't coincide in size, they are " << F.rows() << " and " << N_faces.rows();
    LOG_IF_S(ERROR, V.rows()!=NV.rows()) << "V and NV don't coincide in size, they are " << V.rows() << " and " << NV.rows();
    if (F.size()) LOG_IF_S(ERROR, F.maxCoeff()>V.rows()-1) << "F indexes V at invalid poisitions, max coeff is " << F.maxCoeff() << " and V size is" << V.rows();
    if (E.size()) LOG_IF_S(ERROR, E.maxCoeff()>V.rows()-1) << "E indexes V at invalid poisitions, max coeff is " << E.maxCoeff() << " and V size is" << V.rows() ;
}

std::ostream& operator<<(std::ostream& os, const Mesh& m)
{
    os << "\n";

    m.V.size()?  os << "\t V has size: " << m.V.rows() << " x " << m.V.cols() << "\n"   :   os << "\t V is empty \n";
    m.F.size()?  os << "\t F has size: " << m.F.rows() << " x " << m.F.cols() << "\n"   :   os << "\t F is empty \n";
    m.D.size()?  os << "\t D has size: " << m.D.rows() << " x " << m.D.cols() << "\n"   :   os << "\t D is empty \n";
    m.C.size()?  os << "\t C has size: " << m.C.rows() << " x " << m.C.cols() << "\n"   :   os << "\t C is empty \n";
    m.E.size()?  os << "\t E has size: " << m.E.rows() << " x " << m.E.cols() << "\n"   :   os << "\t E is empty \n";
    m.N_faces.size()?  os << "\t N_faces has size: " << m.N_faces.rows() << " x " << m.N_faces.cols() << "\n"   :   os << "\t N_faces is empty \n";
    m.NV.size()?  os << "\t NV has size: " << m.NV.rows() << " x " << m.NV.cols() << "\n"   :   os << "\t NV is empty \n";


    m.F.size()?  os << "\t F min max coeffs: " << m.F.minCoeff() << " " << m.F.maxCoeff() << "\n"   :   os << "\t F is empty \n";
    m.E.size()?  os << "\t E min max coeffs: " << m.E.minCoeff() << " " << m.E.maxCoeff() << "\n"   :   os << "\t E is empty \n";



    return os;
}
