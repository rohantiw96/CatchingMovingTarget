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

int Planner::run_flag_;
int Planner::steps_;
std::vector<Pair> Planner::path_;

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
        if (run_flag_)
        {
                dijkstra(robot_pose_X, robot_pose_Y);
                std::vector<int> min_cost_trajectory_point = minimumCostPath();
                path_ = getPath(min_cost_trajectory_point[1], min_cost_trajectory_point[2]);
                for (int i = 0;i<min_cost_trajectory_point[0]-1;i++){

                        path_.insert(path_.begin()+min_cost_trajectory_point[5],std::make_pair(min_cost_trajectory_point[3],min_cost_trajectory_point[4]));
                }
                steps_ = path_.size() - 2;
                run_flag_ = false;
        }
        Pair next_step;
        if (steps_ > 0)
        {
                next_step = path_[steps_];
                steps_--;
        }
        else{
                next_step = path_[steps_];
        }
        printf("The point X = %d ,Y = %d",next_step.first,next_step.second);
        action_ptr[0] = next_step.first;
        action_ptr[1] = next_step.second;
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
        int current_index = getMapIndex(current.first, current.second);
        
        while (came_from_.find(current_index) != came_from_.end())
        {
                current = came_from_[current_index];
                current_index = getMapIndex(current.first, current.second);
                path.emplace_back(current);
        }
        return path;
}

int Planner::getPathLength(int target_pose_X, int target_pose_Y)
{
        Pair current = std::make_pair(target_pose_X, target_pose_Y);
        std::vector<Pair> path{current};
        int current_index = getMapIndex(current.first, current.second);
        double min_cost = std::numeric_limits<double>::max();
        int i = 0;
        while (came_from_.find(current_index) != came_from_.end())
        {
                if (min_cost > map_[current_index]) {
                        min_cost_vector_index_ = i;
                        min_cost_point_x_ = current.first;
                        min_cost_point_y_ = current.second;
                        min_cost = map_[current_index];
                }
                current = came_from_[current_index];
                current_index = getMapIndex(current.first, current.second);
                path.emplace_back(current);
                i++;
        }
        return path.size();
}

void Planner::dijkstra(int start_x, int start_y)
{
        std::fill_n(dijkstra_cost_, x_size_ * y_size_, INT_MAX);
        std::priority_queue<Cell> list;
        int src_index = getMapIndex(start_x, start_y);
        dijkstra_cost_[src_index] = 0.0;
        list.push(Cell(dijkstra_cost_[src_index], start_x, start_y));
        while (!list.empty())
        {
                Cell current = list.top();
                int current_index = getMapIndex(current.x_, current.y_);
                list.pop();
                for (int i=0;i<dX_.size();i++)
                {
                        int neighibor_X =  current.x_+dX_[i];
                        int neighibor_Y = current.y_+dY_[i];
                        if (isCellValid(neighibor_X, neighibor_Y))
                        {
                                int neighibor_index = getMapIndex(neighibor_X, neighibor_Y);
                                double cost = dijkstra_cost_[current_index] + map_[neighibor_index];
                                if (dijkstra_cost_[neighibor_index] > cost)
                                {
                                        dijkstra_cost_[neighibor_index] = cost;
                                        list.push(Cell(cost, neighibor_X, neighibor_Y));
                                        came_from_[neighibor_index] = std::make_pair(current.x_, current.y_);
                                }
                        }
                }
        }
        return;
}

std::vector<int> Planner::minimumCostPath()
{
        std::map<double, std::vector<int>> cost_on_trajectory;
        for (int i = 0; i < target_steps_; i++)
        {
                int goalposeX = (int)target_traj_[i];
                int goalposeY = (int)target_traj_[i + target_steps_];
                int index = getMapIndex(goalposeX, goalposeY);
                int path_length = getPathLength(goalposeX, goalposeY);
                if (i > path_length){
                        int wait_time = i - path_length;
                        // cost_on_trajectory[() * map_[min_cost_point_] + dijkstra_cost_[index]] = std::make_pair(goalposeX, goalposeY);
                        cost_on_trajectory[(wait_time) * map_[getMapIndex(min_cost_point_x_,min_cost_point_y_)] + dijkstra_cost_[index]] = std::vector<int>{wait_time,goalposeX, goalposeY,min_cost_point_x_,min_cost_point_y_,min_cost_vector_index_};
                }
        }
        return cost_on_trajectory[cost_on_trajectory.begin()->first];
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
        if (curr_time == 0.0)
        {
                Planner::run_flag_ = true;
        }
        Planner plan(map, collision_thresh, x_size, y_size, target_steps, targettrajV);
        plan.execute(robotposeX, robotposeY, targetposeX, targetposeY, curr_time, action_ptr);
        //     printf("DONE PLANNING!\n");
        return;
}
