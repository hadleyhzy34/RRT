#include <iostream>
#include <vector>
#include <random>

typedef std::vector<std::vector<float>> vec_obstacles;

class Node{
private:
    float x,y;
    Node *parent;
public:
    Node(float x = 0., float y = 0.);
    Node(Node*); //copy constructor
    
    //get method
    Node* getParent();
    float getX();
    float getY();

    //set method
    void setX(float);
    void setY(float);
    void setParent(Node* );
};

class RRT{
public:
    RRT(Node*, Node*, vec_obstacles, std::pair<float,float>);
    Node* get_random_node();
    int get_nearest_list_index(Node*, std::vector<Node*>);
    bool safe_check(Node*, vec_obstacles);
    void path_planning();
    
    //obtain trajectory list
    void set_traj_list();

    //get trajectory list
    std::vector<Node*> get_traj_list();

    //save traj to csv file
    void save_traj();
private:
    Node* start_node;
    Node* goal_node;
    std::pair<float,float> area;
    float expand_radius;
    int maxIter;
    vec_obstacles obstacle_list;
    std::vector<Node*> node_list;
    
    //goal sample rate
    float goal_sample_rate;

    //randomly generate numbers
    std::mt19937 gen;
    std::uniform_real_distribution<> dis;

    //trajectory list
    std::vector<Node*> traj_list;
};
