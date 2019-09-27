/**************************************************************
 * Writer : HAN NU RIM
 * Date : 2019.09.27 FRI
 * Input : Numbers of node with keyboard by User
 *         Start and finish node with keyboard by User
 *         dijkstra_data.txt
 *         1) Data of goals (position x, y/orientation z, w)
 * Output : Turtlebot3 will plan the path from start node to goal node
 *          and take care of it along the path.
 * ************************************************************/
#include "dijkstra.h"

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
// subscribe map data and start/finish node that User want
{
    init_map = new int*[msg->info.height];
    for (int i = 0; i < msg->info.height; i++)
        init_map[i] = new int[msg->info.width];

    for (int i = 0; i < msg->info.height; i++)
        for (int j = 0; j < msg->info.width; j++)
            init_map[i][j] = msg->data[i*msg->info.width + j];
    resolution = msg->info.resolution;
    center = msg->info.width / 2;

    int num;
    cout << "How many goals? : ";
    cin >> num;
    Graph g(num);

    while(ros::ok()) {
        int start, finish;
        cout << "\nWhat is the start point? : ";
        cin >> start;
        cout << "What is the finish point? : ";
        cin >> finish;

        int *path = new int[num];

        path = g.PrintPath(start, finish);
        GoalPublisher(path, g);
    }
}

void GoalPublisher(int *path, class Graph g)
// goal publish
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
            ROS_INFO("Waiting for the move_base action server to come up");

    move_base_msgs::MoveBaseGoal goalOutput;

    goalOutput.target_pose.header.frame_id = "map";

    int i = 0;
    while (path[i] != INF) {
        goalOutput.target_pose.header.stamp = ros::Time::now();
        goalOutput.target_pose.pose.position.x = g.returnval(path[i],0);     // input x position that you want
        goalOutput.target_pose.pose.position.y = g.returnval(path[i],1);     // input y position that you want
        goalOutput.target_pose.pose.orientation.z = g.returnval(path[i],2);  // input z orientation that you want
        goalOutput.target_pose.pose.orientation.w = g.returnval(path[i],3);  // input w orientation that you want

        ac.sendGoal(goalOutput);
        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            i++;
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dijkstra");
  ros::NodeHandle nh;

  ROS_INFO("dijkstra start");

  ros::Subscriber sub = nh.subscribe("map", 1000, mapCallback);

  ros::spin();

  return 0;
}
