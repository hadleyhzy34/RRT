#include "rrt.h"
#include <fstream>

Node::Node(float _x, float _y): x(_x),y(_y),parent(nullptr){}

Node::Node(Node* node){
    this->x = node->x;
    this->y = node->y;
    this->parent = node->parent;
}

Node* Node::getParent(){return this->parent;}

float Node::getX(){return this->x;}

float Node::getY(){return this->y;}

void Node::setParent(Node* _node){this->parent = _node;}

void Node::setX(float _x){this->x = _x;}

void Node::setY(float _y){this->y = _y;}


RRT::RRT(Node* _start_node, Node* _goal_node, vec_obstacles _obstacle_list, std::pair<float,float> _area)
    :start_node(_start_node),
     goal_node(_goal_node),
     obstacle_list(_obstacle_list),
     area(_area){
         maxIter = 500;
         expand_radius = 0.2;
         goal_sample_rate = 0.05;

         //generate random seed
         std::random_device rd;
         std::mt19937 gen(rd());
         std::uniform_real_distribution<> dis(area.first, area.second);

         //initialize node_list
         node_list.push_back(start_node);
}

Node* RRT::get_random_node(){
    float x = dis(gen);
    float y = dis(gen);
    Node *node = new Node(x,y);
    return node;
}

int RRT::get_nearest_list_index(Node* cur, std::vector<Node*> _node_list){
    float min_dist = std::numeric_limits<float>::max();
    int index;
    float dist;
    for(size_t i = 0; i<_node_list.size(); i++){
        dist = std::pow(_node_list[i]->getX() - cur->getX(), 2) + std::pow(_node_list[i]->getY() - cur->getY(), 2);
        if(dist < min_dist){
            min_dist = dist;
            index = i;
        }
    }
    return index;
}

bool RRT::safe_check(Node* cur, vec_obstacles obstacle_list){
    float dist;
    for(size_t i = 0; i < obstacle_list.size(); i++){
        dist = std::pow(obstacle_list[i][0] - cur->getX(),2) + std::pow(obstacle_list[i][1] - cur->getY(),2);
        if(std::sqrt(dist) <= obstacle_list[i][2]){return false;}
    }
    return true;
}

void RRT::path_planning(){
    int iter = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    //std::cout<<"range of distribution is between: "<<area.first<<" "<<area.second<<"\n";
    std::uniform_real_distribution<float> r(area.first, area.second);
    std::uniform_real_distribution<float> u(0.,1.);
    while(true){
        //random sampling
        Node* random_node = new Node();
        if(u(gen) > goal_sample_rate){
            //random_node = get_random_node();
            random_node->setX(r(gen));
            random_node->setY(r(gen));
        }else{
            random_node->setX(goal_node->getX());
            random_node->setY(goal_node->getY());
        }
        
        //std::cout<<"current random node is: "<<random_node->getX()<<" "<<random_node->getY()<<"\n";
        //find nearest node
        int min_index = get_nearest_list_index(random_node, node_list);

        //expand tree
        Node* nearest_node = new Node(node_list[min_index]);

        Node* new_node = new Node(nearest_node);

        //arc
        float theta = std::atan2(random_node->getY() - nearest_node->getY(), random_node->getX() - nearest_node->getX());

        new_node->setX(nearest_node->getX() + expand_radius * std::cos(theta));
        new_node->setY(nearest_node->getY() + expand_radius * std::sin(theta));
        new_node->setParent(node_list[min_index]);

        if(safe_check(new_node, obstacle_list) == false)continue;

        node_list.push_back(new_node);

        //check_goal
        float dx = new_node->getX() - goal_node->getX();
        float dy = new_node->getY() - goal_node->getY();
        float dist = (dx * dx + dy * dy);

        if(std::sqrt(dist) <= expand_radius){
            std::cout<<"iter is: "<<iter<<"Goal!"<<std::endl;
            traj_list.push_back(new_node);
            break;
        }
        iter++;
        std::cout<<"current iter is: "<<iter<<"\n";
    }
}

void RRT::set_traj_list(){
    Node* cur = traj_list[traj_list.size()-1];
    while(cur->getParent()!=nullptr){
        cur = cur->getParent();
        traj_list.push_back(cur);
    }
}

std::vector<Node*> RRT::get_traj_list(){return traj_list;}

void RRT::save_traj(){
    std::ofstream myfile("/home/hadley/development/c-robotics/path_planning/rrt/traj.csv");
    
    for(int i = traj_list.size()-1; i >= 0; i--){
        myfile << traj_list[i]->getX();
        myfile << ",";
        myfile << traj_list[i]->getY() << std::endl;
    }
}

