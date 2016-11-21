
//state definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <project3/purePursuit.h>
#include <project3/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

//map spec
cv::Mat map;
cv::Mat dynamic_map; //use this variable at dynamic mapping
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

//way points
std::vector<point> waypoints;

//path
std::vector<point> path_RRT;

//robot
point robot_pose;
geometry_msgs::Twist cmd_vel;

//point cloud data from kinect
pcl::PointCloud<pcl::PointXYZ> point_cloud;

//FSM state
int state;

//function definition
bool isCollision();
void dynamic_mapping();
void set_waypoints();
void generate_path_RRT();
void callback_state(gazebo_msgs::ModelStatesConstPtr msgs);
void callback_points(sensor_msgs::PointCloud2ConstPtr msgs);
void setcmdvel(double v, double w);

int main(int argc, char** argv){
    ros::init(argc, argv, "rrt_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Subscriber gazebo_pose_sub = n.subscribe("/gazebo/model_states",1,callback_state);
    ros::Subscriber gazebo_kinect_sub = n.subscribe("/camera/depth/points",1,callback_points);
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);
    ros::ServiceClient gazebo_spawn = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::ServiceClient gazebo_set = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    printf("Initialize topics\n");

    // Load Map
    char* user = getlogin();
    map = cv::imread((std::string("/home/")+
                      std::string(user)+
                      std::string("/catkin_ws/src/project3/src/ground_truth_map.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    map_y_range = map.cols;
    map_x_range = map.rows;
    map_origin_x = map_x_range/2.0 - 0.5;
    map_origin_y = map_y_range/2.0 - 0.5;
    world_x_min = -10.0;
    world_x_max = 10.0;
    world_y_min = -10.0;
    world_y_max = 10.0;
    res = 0.05;
    printf("Load map\n");

    // Set Way Points
    set_waypoints();
    printf("Set way points\n");

    // RRT
    generate_path_RRT();
    printf("Generate RRT\n");

    // FSM
    state = INIT;
    bool running = true;
    purePursuit pure_pursuit;
    int look_ahead_idx;
    ros::Rate control_rate(10);

    while(running){
        switch (state) {
        case INIT: {
            look_ahead_idx = 0;

            //visualize path
            for(int i = 0; i < path_RRT.size(); i++){
                gazebo_msgs::SpawnModel model;
                model.request.model_xml = std::string("<robot name=\"simple_ball\">") +
                        std::string("<link name=\"ball\">") +
                        std::string("<inertial>") +
                        std::string("<mass value=\"1.0\" />") +
                        std::string("<origin xyz=\"0 0 0\" />") +
                        std::string("<inertia  ixx=\"1.0\" ixy=\"0.0\"  ixz=\"0.0\"  iyy=\"1.0\"  iyz=\"0.0\"  izz=\"1.0\" />") +
                        std::string("</inertial>") +
                        std::string("<visual>") +
                        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                        std::string("<geometry>") +
                        std::string("<sphere radius=\"0.09\"/>") +
                        std::string("</geometry>") +
                        std::string("</visual>") +
                        std::string("<collision>") +
                        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                        std::string("<geometry>") +
                        std::string("<sphere radius=\"0\"/>") +
                        std::string("</geometry>") +
                        std::string("</collision>") +
                        std::string("</link>") +
                        std::string("<gazebo reference=\"ball\">") +
                        std::string("<mu1>10</mu1>") +
                        std::string("<mu2>10</mu2>") +
                        std::string("<material>Gazebo/Blue</material>") +
                        std::string("<turnGravityOff>true</turnGravityOff>") +
                        std::string("</gazebo>") +
                        std::string("</robot>");

                std::ostringstream ball_name;
                ball_name << i;
                model.request.model_name = ball_name.str();
                model.request.reference_frame = "world";
                model.request.initial_pose.position.x = path_RRT[i].x;
                model.request.initial_pose.position.y = path_RRT[i].y;
                model.request.initial_pose.position.z = 0.7;
                model.request.initial_pose.orientation.w = 0.0;
                model.request.initial_pose.orientation.x = 0.0;
                model.request.initial_pose.orientation.y = 0.0;
                model.request.initial_pose.orientation.z = 0.0;

                gazebo_spawn.call(model);

                ros::spinOnce();
                ros::Rate(10).sleep();
            }
            printf("Spawn path\n");

            //initialize robot position
            geometry_msgs::Pose model_pose;
            model_pose.position.x = waypoints[0].x;
            model_pose.position.y = waypoints[0].y;
            model_pose.position.z = 0.3;
            model_pose.orientation.x = 0.0;
            model_pose.orientation.y = 0.0;
            model_pose.orientation.z = 0.0;
            model_pose.orientation.w = 0.0;

            geometry_msgs::Twist model_twist;
            model_twist.linear.x = 0.0;
            model_twist.linear.y = 0.0;
            model_twist.linear.z = 0.0;
            model_twist.angular.x = 0.0;
            model_twist.angular.y = 0.0;
            model_twist.angular.z = 0.0;

            gazebo_msgs::ModelState modelstate;
            modelstate.model_name = "RosAria";
            modelstate.reference_frame = "world";
            modelstate.pose = model_pose;
            modelstate.twist = model_twist;

            gazebo_msgs::SetModelState setmodelstate;
            setmodelstate.request.model_state = modelstate;

            gazebo_set.call(setmodelstate);
            ros::spinOnce();
            ros::Rate(0.33).sleep();
            printf("Initialize ROBOT\n");

            state = RUNNING;
        } break;

        case RUNNING: {
            //TODO
            /*
             * copy your code from previous project2
             */
            /*
             * add transition part from RUNNING to PATH_PLANNING
             * when meeting obstacle
             */
            double tmp_x = path_RRT[look_ahead_idx].x;
            double tmp_y = path_RRT[look_ahead_idx].y;
            double dist_sq = (robot_pose.x-tmp_x)*(robot_pose.x-tmp_x) + (robot_pose.y-tmp_y)*(robot_pose.y-tmp_y);
            if(isCollision())
                state = PATH_PLANNING;
            else if(dist_sq < 0.04){
                if(++look_ahead_idx == path_RRT.size()-1)
                    state = FINISH;
                control ctrl = pure_pursuit.get_control(robot_pose, path_RRT[look_ahead_idx]);
                setcmdvel(ctrl.v, ctrl.w);
                cmd_vel_pub.publish(cmd_vel);
              }
            ros::spinOnce();
            control_rate.sleep();
        } break;

        case PATH_PLANNING: {
            //TODO
            /*
			 * call void dynamic_mapping();
             * do dynamic mapping from kinect data
             * pop up the opencv window
             * after drawing the dynamic map, transit the state to RUNNING state
             */
            dynamic_mapping();
            cv::namedWindow("kinect");
            cv::imshow("kinect", dynamic_map);
            state = RUNNING;
            ros::spinOnce();
            control_rate.sleep();
        } break;

        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd_vel);
            running = false;

            ros::spinOnce();
            control_rate.sleep();
        } break;

        default: {
        } break;
        }

        printf("curr state : %d\ncurr robot pos : %.2f,%.2f\ncurr robot vel : %.2f,%.2f\n",state,robot_pose.x,robot_pose.y,cmd_vel.linear.x,cmd_vel.angular.z);
    }

    return 0;
}

void generate_path_RRT()
{
    //TODO
    /*
     * copy your code from previous project2
	 * change method to rrt*
     */
    rrtTree* get_rrtTree[waypoints.size()-1];
    for(int i=1; i<waypoints.size(); ++i)
        get_rrtTree[i-1] = new rrtTree(waypoints[i-1], waypoints[i], map, map_origin_x, map_origin_y, res, 12);
    for(int i=1; i<waypoints.size(); ++i){
        int is_done = get_rrtTree[i-1]->generateRRTst(world_x_max, world_x_min, world_y_max, world_y_min, 100, 2.50);
        if(is_done == 0){
            std::vector<point> tmp_path_RRT = get_rrtTree[i-1]->backtracking();
            path_RRT.reserve(path_RRT.size() + tmp_path_RRT.size());
            path_RRT.insert(path_RRT.end(), tmp_path_RRT.begin(), tmp_path_RRT.end());
            tmp_path_RRT.clear();
        } else i -= 1; // Do RRT again.
    }
    for(int i=0; i<waypoints.size()-1; i++)
        delete get_rrtTree[i];
}

void set_waypoints()
{
    point waypoint_candid[4];
    waypoint_candid[0].x = 5.0;
    waypoint_candid[0].y = -8.0;
    waypoint_candid[1].x = -6.0;
    waypoint_candid[1].y = -7.0;
    waypoint_candid[2].x = -8.0;
    waypoint_candid[2].y = 8.0;
    waypoint_candid[3].x = 3.0;
    waypoint_candid[3].y = 7.0;

    int order[] = {3,1,2,3};
    int order_size = 4;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}


void callback_state(gazebo_msgs::ModelStatesConstPtr msgs){
    for(int i; i < msgs->name.size(); i++){
        if(std::strcmp(msgs->name[i].c_str(),"RosAria") == 0){
            robot_pose.x = msgs->pose[i].position.x;
            robot_pose.y = msgs->pose[i].position.y;
            robot_pose.th = tf::getYaw(msgs->pose[i].orientation);
        }
    }
}

void callback_points(sensor_msgs::PointCloud2ConstPtr msgs){
    pcl::fromROSMsg(*msgs,point_cloud);
}

bool isCollision()
{
    //TODO
    /*
     * obstacle emerge in front of robot -> true
     * other wise -> false
     */
    return false;
}


void dynamic_mapping()
{
	//TODO
	/*
	* draw dynamic map using variable dynamic_map and variable point_cloud
	*/
}

void setcmdvel(double v, double w){
    cmd_vel.linear.x = v;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = w;
}
