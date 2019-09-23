#include "planner.h"
#include <mex.h>

#define MAP_IN prhs[0]
#define ROBOT_IN prhs[1]
#define TARGET_TRAJ prhs[2]
#define TARGET_POS prhs[3]
#define CURR_TIME prhs[4]
#define COLLISION_THRESH prhs[5]

/* Output Arguments */
#define ACTION_OUT plhs[0]

int counter = true;
int steps;
std::vector<Pair> path;
// Planner plan;

Planner::Planner(double *map,
                 int collision_thresh,
                 int x_size,
                 int y_size,
                 int target_steps,
                 double *target_traj)
{
        map_ = map;
        collision_thresh_ = collision_thresh;
        x_size_ = x_size;
        y_size_ = y_size;
        target_steps_ = target_steps;
        target_traj_ = target_traj;
        dijkstra_cost_ = new double[x_size_ * y_size_];
}
void Planner::execute(int robot_pose_X,
                      int robot_pose_Y,
                      int target_pose_X,
                      int target_pose_Y,
                      int curr_time,
                      double *action_ptr)
{       
        if (counter){
                dijkstra(robot_pose_X,robot_pose_Y);
                Pair min_cost_trajectory_point = minimumCostPath();
                path = getPath(min_cost_trajectory_point.first,min_cost_trajectory_point.second);
                steps = path.size()-2;
                counter = false;
        }
        Pair nextStep;
        if (steps > 0){
           nextStep = path[steps];
           steps--;
        }
        else{
                nextStep = path[steps];
        }
        action_ptr[0] = nextStep.first;
        action_ptr[1] = nextStep.second;
        return;
}

int Planner::getMapIndex(const int x, const int y)
{
        return ((y - 1) * x_size_ + (x - 1));
}

bool Planner::isCellValid(const int x, const int y)
{
        if (x >= 1 && x <= x_size_ && y >= 1 && y <= y_size_)
        {
                int index = getMapIndex(x, y);
                if (map_[index] >= 0 && (map_[index] < collision_thresh_))
                        return true;
        }
        return false;
}

std::vector<Pair> Planner::getNeighibors(const int x, const int y)
{
        std::vector<Pair> neighibors;
        for (int i = 0; i < dX_.size(); i++)
        {
                neighibors.emplace_back(std::make_pair(x + dX_[i], y + dY_[i]));
        }
        return neighibors;
}

std::vector<Pair> Planner::getPath(int target_pose_X, int target_pose_Y)
{
        Pair current = std::make_pair(target_pose_X, target_pose_Y);
        std::vector<Pair> path{current};
        int current_index = getMapIndex(current.first,current.second);
        while (came_from_dijkstra_.find(current_index) != came_from_dijkstra_.end())
        {
                current = came_from_dijkstra_[current_index];
                current_index = getMapIndex(current.first,current.second);
                path.emplace_back(current);
        }
        return path;
}

int Planner::getPathLength(int target_pose_X, int target_pose_Y)
{
        Pair current = std::make_pair(target_pose_X, target_pose_Y);
        std::vector<Pair> path{current};
        int current_index = getMapIndex(current.first,current.second);
        while (came_from_dijkstra_.find(current_index) != came_from_dijkstra_.end())
        {
                current = came_from_dijkstra_[current_index];
                current_index = getMapIndex(current.first,current.second);
                path.emplace_back(current);
        }
        return path.size();
}

void Planner::dijkstra(int start_x, int start_y){
        std::fill_n(dijkstra_cost_, x_size_ * y_size_, INT_MAX);
        std::priority_queue<Cell> list;
        int src_index = getMapIndex(start_x,start_y);
        dijkstra_cost_[src_index] = 0.0;
        list.push(Cell(dijkstra_cost_[src_index],start_x,start_y));
        while(!list.empty()){
                Cell current = list.top();
                int current_index = getMapIndex(current.x_,current.y_);
                list.pop();
                for(const auto& neighibor:getNeighibors(current.x_,current.y_)){
                        if(isCellValid(neighibor.first,neighibor.second))
                        {
                                int neighibor_index = getMapIndex(neighibor.first,neighibor.second);
                                double cost =  dijkstra_cost_[current_index] + map_[neighibor_index];
                                if (dijkstra_cost_[neighibor_index] > cost){
                                        dijkstra_cost_[neighibor_index] = cost;
                                        list.push(Cell(cost,neighibor.first,neighibor.second));
                                        came_from_dijkstra_[neighibor_index] = std::make_pair(current.x_,current.y_);
                                }
                        }
                }
        }
        return;
}



Pair Planner::minimumCostPath(){
        std::map<double,Pair> cost_on_trajectory;
        for(int i=0;i<target_steps_;i++){
                int goalposeX = (int) target_traj_[i];
                int goalposeY = (int) target_traj_[i+target_steps_];
                int index = getMapIndex(goalposeX,goalposeY);
                int path_length = getPathLength(goalposeX,goalposeY);
                if (i>path_length)
                        cost_on_trajectory[(i - path_length)*map_[index] + dijkstra_cost_[index]] = std::make_pair(goalposeX,goalposeY);
        }
        Pair min_cost_trajectory_point = cost_on_trajectory[cost_on_trajectory.begin()->first];
        return min_cost_trajectory_point;
}

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])

{

        /* Check for proper number of arguments */
        if (nrhs != 6)
        {
                mexErrMsgIdAndTxt("MATLAB:planner:invalidNumInputs",
                                  "Six input arguments required.");
        }
        else if (nlhs != 1)
        {
                mexErrMsgIdAndTxt("MATLAB:planner:maxlhs",
                                  "One output argument required.");
        }

        /* get the dimensions of the map and the map matrix itself*/
        int x_size = mxGetM(MAP_IN);
        int y_size = mxGetN(MAP_IN);
        double *map = mxGetPr(MAP_IN);

        /* get the dimensions of the robotpose and the robotpose itself*/
        int robotpose_M = mxGetM(ROBOT_IN);
        int robotpose_N = mxGetN(ROBOT_IN);
        if (robotpose_M != 1 || robotpose_N != 2)
        {
                mexErrMsgIdAndTxt("MATLAB:planner:invalidrobotpose",
                                  "robotpose vector should be 1 by 2.");
        }
        double *robotposeV = mxGetPr(ROBOT_IN);
        int robotposeX = (int)robotposeV[0];
        int robotposeY = (int)robotposeV[1];

        /* get the dimensions of the goalpose and the goalpose itself*/
        int targettraj_M = mxGetM(TARGET_TRAJ);
        int targettraj_N = mxGetN(TARGET_TRAJ);

        if (targettraj_M < 1 || targettraj_N != 2)
        {
                mexErrMsgIdAndTxt("MATLAB:planner:invalidtargettraj",
                                  "targettraj vector should be M by 2.");
        }
        double *targettrajV = mxGetPr(TARGET_TRAJ);
        int target_steps = targettraj_M;

        /* get the current position of the target*/
        int targetpose_M = mxGetM(TARGET_POS);
        int targetpose_N = mxGetN(TARGET_POS);
        if (targetpose_M != 1 || targetpose_N != 2)
        {
                mexErrMsgIdAndTxt("MATLAB:planner:invalidtargetpose",
                                  "targetpose vector should be 1 by 2.");
        }
        double *targetposeV = mxGetPr(TARGET_POS);
        int targetposeX = (int)targetposeV[0];
        int targetposeY = (int)targetposeV[1];

        /* get the current timestep the target is at*/
        int curr_time = mxGetScalar(CURR_TIME);

        /* Create a matrix for the return action */
        ACTION_OUT = mxCreateNumericMatrix((mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL);
        double *action_ptr = (double *)mxGetData(ACTION_OUT);

        /* Get collision threshold for problem */
        int collision_thresh = (int)mxGetScalar(COLLISION_THRESH);

        /* Do the actual planning in a subroutine */
        //     planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
        Planner plan(map, collision_thresh, x_size, y_size, target_steps, targettrajV);
        plan.execute(robotposeX, robotposeY, targetposeX, targetposeY, curr_time, action_ptr);
        //     printf("DONE PLANNING!\n");
        return;
}
