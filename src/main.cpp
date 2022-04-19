#include <iostream>
#include <vector>
#include "rrt.h"

int main(){
    std::vector<std::vector<float>> obstacle_list = {
        {5, 1, std::sqrt(2.)/2.},
        {3, 6, std::sqrt(2.)/2.},
        {3, 8, std::sqrt(2.)/2.},
        {1, 1, std::sqrt(2.)/2.},
        {3, 5, std::sqrt(2.)/2.},
        {9, 5, std::sqrt(2.)/2.},
    };

    //set initial parameters
    Node* start_node = new Node(0.,0.);
    Node* goal_node = new Node(6.,8.);
    
    std::pair<float,float> _area(-2., 10.);

    RRT rrt = RRT(start_node, goal_node, obstacle_list, _area);
    
    //rrt global path planning
    rrt.path_planning();
    // 
    rrt.set_traj_list();

    std::vector<Node*> traj = rrt.get_traj_list();
    for(int i = traj.size()-1; i >= 0; i--){
        std::cout<<"i: "<<i<<" x: "<<traj[i]->getX()<<" y: "<<traj[i]->getY()<<"\n";
    }
    
    rrt.save_traj();
    return 0;
}
