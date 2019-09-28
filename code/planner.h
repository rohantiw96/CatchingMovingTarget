#include <iostream>
#include <queue>
#include <list>
#include <utility>
#include <vector>
#include <math.h>
#include <map>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <chrono>

#define NUMOFDIRS 9

typedef std::pair<int,int> Pair;

struct Cell
{
    double cost_;
    int x_;
    int y_;
    Cell(const double cost, const int x, const int y)
    {
        cost_ = cost;
        x_ = x;
        y_ = y;
    }
    bool operator<(const Cell &val) const { return cost_ > val.cost_; }
};

struct WaitingCell
{
    int x_;
    int y_;
    int wait_time_;
    int min_cost_index_;
    WaitingCell(const double wait_time, const int x,const int y, const int min_cost_index)
    {
        wait_time_ = wait_time;
        x_ = x;
        y_ = y;
        min_cost_index_ = min_cost_index;
    }
};

class Planner
{
private:
    double *map_;
    int collision_thresh_;
    int x_size_;
    int y_size_;
    int target_steps_;
    double *target_traj_;
    double *g_values;
    double *dijkstra_cost_;
    int min_cost_point_x_;
    int min_cost_point_y_;
    int min_cost_vector_index_;
    std::vector<int> dX_{-1, -1, -1, 0, 0, 0, 1, 1, 1};
    std::vector<int> dY_{-1, 0, 1, -1, 0, 1, -1, 0, 1};
    double *g_values_;
    std::unordered_map<int,Pair> came_from_;
    bool goal_reached_;

    void dijkstra(int start_x,int start_y);

    int getMapIndex(const int x,const int y);

    bool isCellValid(const int x,const int y);

    std::vector<Pair> getPath(int target_pose_X,int target_pose_Y);

    int getPathLength(int target_pose_X, int target_pose_Y);

    std::vector<Pair> getNeighibors(const int x,const int y);

    std::vector<int> minimumCostPath();

public:
    static std::vector<Pair> path_;
    static int steps_;
    static int run_flag_;

    Planner(double *map,
            int collision_thresh,
            int x_size,
            int y_size,
            int target_steps,
            double *target_traj);

    void execute(int robot_pose_X,
                 int robot_pose_Y,
                 int target_pose_X,
                 int target_pose_Y,
                 int curr_time,
                 double *action_ptr);
};
