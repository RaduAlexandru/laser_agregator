#include "laser_agregator/Edge.h"


Edge::Edge(Mesh& mesh):
    m_mesh(mesh),
    need_to_split_at_beggining(false),
    need_to_split_at_finale(false){

    m_edge << -1, -1;
    m_length = -1;
    m_first_dir.setZero();


}

double Edge::get_length(){
    return m_length;
}

Eigen::Vector2i Edge::get_eigen(){
    return m_edge;
}

Eigen::Vector3d Edge::get_dir(){
//    return m_mesh.V.row(m_edge(1)) - m_mesh.V.row(m_edge(0));
    return m_first_dir;
}

Eigen::Vector3d Edge::get_dir_normalized(){
    return get_dir().normalized();
}

double Edge::similarity(Edge& edge){

    //if one of them is not valid just return 1.0 as if they are the same vector
    if (!this->is_valid() || !edge.is_valid()){
        return 1.0;
    }

    Eigen::Vector3d my_dir=this->get_dir_normalized();
    Eigen::Vector3d other_dir=edge.get_dir_normalized();

    //if they are actually the same edge, just return 1.0 as if the dot product is maximum
    if (m_edge(0)==edge.get_eigen()(0) && m_edge(1)==edge.get_eigen()(1) ){
        return 1.0;
    }else{
        return my_dir.dot(other_dir);
    }


}

bool Edge::is_valid(){
    if (m_edge(0)!=-1 &&  m_edge(1)!=-1 && !m_first_dir.isZero()){
        return true;
    }else{
        return false;
    }
}

bool Edge::has_start_idx(){
    if (m_edge(0)!=-1) {
        return true;
    }else{
        return false;
    }
}

bool Edge::has_end_idx(){
    if (m_edge(1)!=-1) {
        return true;
    }else{
        return false;
    }
}


void Edge::set_start_idx(int idx){
    m_edge(0)=idx;
}
void Edge::set_end_idx(int idx){
    m_edge(1)=idx;
    m_length=m_edge(1)-m_edge(0) ; //represent the nr of segment (so an edge with two consecutive points will have length 1)

    //if it's the first time we are creating a new_edge then it will have as start point -1 so we should leave no dir
    if (m_edge(0)==-1 || m_edge(1)==-1){
        m_first_dir.setZero();
        return;
    }

    //if it's the first time we set the end idx then we also get the direction
    if (m_first_dir.isZero()){
        m_first_dir =m_mesh.V.row(m_edge(1)) - m_mesh.V.row(m_edge(0));
    }

}

int Edge::get_start_idx(){
    return m_edge(0);
}
int Edge::get_end_idx(){
    return m_edge(1);
}

void Edge::invalidate(){
    m_edge << -1, -1;
    m_length = -1;
    m_first_dir.setZero();
    need_to_split_at_beggining=false;
    need_to_split_at_finale=false;
}

void Edge::stop_and_continue(){
    m_edge(0)=m_edge(1); //start where the last edge finished
    m_edge(1)=m_edge(1);
    m_length = -1;
    m_first_dir.setZero();
    need_to_split_at_beggining=false;
    need_to_split_at_finale=false;
}

void Edge::copy(Edge& new_edge){
    this->m_edge=new_edge.get_eigen();
    this->m_first_dir= new_edge.get_dir();
    this->m_length= new_edge.get_length();
    this->need_to_split_at_beggining= new_edge.need_to_split_at_beggining;

}

bool Edge::is_at_grazing_angle ( const double grazing_angle_thresh){


    Eigen::Vector3d dir  = (m_mesh.V.row(m_edge(1)) - m_mesh.V.row(m_edge(0))).normalized() ; //get_dir returns the first dir of the edge, but we want the current one
    Eigen::Vector3d view = (Eigen::Vector3d(m_mesh.V.row(get_start_idx())) - m_mesh.sensor_pose.translation()).normalized();  //the view direction is towards the first vertex of the edge, does't really matter too much which one we take


    //TODO for very small edges, no need to check the angle this would help with retaining some of the structure

    double dot = dir.dot(view);
    if (std::fabs(dot)<=grazing_angle_thresh){  //ideally the dot will be close to 0 so the view and edge and orthogonal
        VLOG(4) << "not at grazing angle, perfectly good edge " << dot;
        return false;
    }else{
        VLOG(4) << "it's at grazing angle " << dot;
        return true;
//
    }

}

void Edge::check_and_push(std::vector<Edge>& edges_vec, const double grazing_angle_thresh){

    if(!is_valid()){
        return;
    }

    Eigen::Vector3d dir  = (m_mesh.V.row(m_edge(1)) - m_mesh.V.row(m_edge(0))).normalized() ; //get_dir returns the first dir of the edge, but we want the current one
    Eigen::Vector3d view = (Eigen::Vector3d(m_mesh.V.row(get_start_idx())) - m_mesh.sensor_pose.translation()).normalized();  //the view direction is towards the first vertex of the edge, does't really matter too much which one we take


    //TODO for very small edges, no need to check the angle this would help with retaining some of the structure

    double dot = dir.dot(view);
    if (std::fabs(dot)<=grazing_angle_thresh){  //ideally the dot will be close to 0 so the view and edge and orthogonal
        edges_vec.push_back(*this);
//        VLOG(2) << "adding dot is " << dot;
    }else{
//        VLOG(2) << "nt adding dot is " << dot;
    }


}

void Edge::push_into_vec(std::vector<Edge>& edges_vec){

    // // if(need_to_split_at_finale && edge.get_length()>=3){
    // //
    // // }
    // //
    // // if(edge.get_length()>=3){
    // //     Edge edge_split_1(mesh);
    // //     Edge edge_split_2(mesh);
    // //     edge_split_1.set_start_idx(edge.get_start_idx());
    // //     edge_split_1.set_end_idx(edge.get_end_idx()-1);
    // //     edge_split_2.set_start_idx(edge.get_end_idx()-1);
    // //     edge_split_2.set_end_idx(edge.get_end_idx());
    // //     edges_vec.push_back(edge_split_1);
    // //     edges_vec.push_back(edge_split_2);
    // //     if(edge.need_to_split_at_beggining){
    // //         Edge edge_split_0(mesh);
    // //         edge_split_0.set_start_idx(edge.get_start_idx());
    // //         edge_split_0.set_end_idx(edge.get_start_idx()+1);
    // //         edges_vec.push_back(edge_split_0);
    // //     }
    // //
    // // }else{
    //     edges_vec.push_back(*this);
    // // }


    //split two times
    if(need_to_split_at_beggining && need_to_split_at_finale && m_length>=3){
        Edge edge_split_beggining(m_mesh);
        Edge edge_split_middle(m_mesh);
        Edge edge_split_end(m_mesh);
        edge_split_beggining.set_start_idx(this->get_start_idx());
        edge_split_beggining.set_end_idx(this->get_start_idx()+1);
        edge_split_middle.set_start_idx(this->get_start_idx()+1);
        edge_split_middle.set_end_idx(this->get_end_idx()-1);
        edge_split_end.set_start_idx(this->get_end_idx()-1);
        edge_split_end.set_end_idx(this->get_end_idx());
        edges_vec.push_back(edge_split_beggining);
        edges_vec.push_back(edge_split_middle);
        edges_vec.push_back(edge_split_end);
    //split only at the beggining
    }else if(need_to_split_at_beggining && !need_to_split_at_finale && m_length>=2){
        Edge edge_split_beggining(m_mesh);
        Edge edge_split_rest(m_mesh);
        edge_split_beggining.set_start_idx(this->get_start_idx());
        edge_split_beggining.set_end_idx(this->get_start_idx()+1);
        edge_split_rest.set_start_idx(this->get_start_idx()+1);
        edge_split_rest.set_end_idx(this->get_end_idx());
        edges_vec.push_back(edge_split_beggining);
        edges_vec.push_back(edge_split_rest);
    //split only at the finale
    }else if(!need_to_split_at_beggining && need_to_split_at_finale && m_length>=2){
        Edge edge_split_rest(m_mesh);
        Edge edge_split_end(m_mesh);
        edge_split_rest.set_start_idx(this->get_start_idx());
        edge_split_rest.set_end_idx(this->get_end_idx()-1);
        edge_split_end.set_start_idx(this->get_end_idx()-1);
        edge_split_end.set_end_idx(this->get_end_idx());
        edges_vec.push_back(edge_split_rest);
        edges_vec.push_back(edge_split_end);
    }else{
         edges_vec.push_back(*this);
    }

}
