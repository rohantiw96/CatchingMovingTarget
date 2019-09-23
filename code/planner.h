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

#define NUMOFDIRS 9

typedef std::pair<int,int> Pair;

struct Cell
{
    double f_value_;
    int x_;
    int y_;
    Cell(const double f_value, const int x, const int y)
    {
        f_value_ = f_value;
        x_ = x;
        y_ = y;
    }
    bool operator<(const Cell &val) const { return f_value_ > val.f_value_; }
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
    std::vector<int> dX_{-1, -1, -1, 0, 0, 0, 1, 1, 1};
    std::vector<int> dY_{-1, 0, 1, -1, 0, 1, -1, 0, 1};
    double *g_values_;
    std::priority_queue<Cell> open_list_;
    std::unordered_map<int,Cell> closed_list_;
    std::unordered_map<int,Pair> came_from_;
    std::unordered_map<int,Pair> came_from_dijkstra_;
    bool goal_reached_;

    void dijkstra(int start_x,int start_y);
    double calculateHeuristic(int robot_pose_X,
                              int robot_pose_Y,
                              int target_pose_X,
                              int target_pose_Y);

    int getMapIndex(const int x,const int y);

    bool isCellValid(const int x,const int y);

    std::vector<Pair> getPath(int target_pose_X,int target_pose_Y);
    int getPathLength(int target_pose_X, int target_pose_Y);

    std::vector<Pair> getNeighibors(const int x,const int y);

    Pair minimumCostPath();

public:
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
