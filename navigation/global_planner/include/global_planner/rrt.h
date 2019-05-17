#ifndef _RRT_A_H
#define _RRT_A_H
#include <global_planner/traceback.h>
#include <global_planner/planner_core.h>
#include <cmath>
#include <vector>
#include <stdlib.h>
#include <algorithm>
namespace global_planner
{
struct _rrt_point
{
    int parent;
    double x;
    double y;
};
typedef struct _rrt_point rrt_point;
class rrtPlannerC : public Traceback
{
  public:
    rrtPlannerC(PotentialCalculator *p_calc) : Traceback(p_calc) {}
    ~rrtPlannerC();
    bool getPath(float *potential, unsigned char *costs, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float>> &path);

    /**
     * @brief  Get the distance between two rrt_point
     * @note   
     * @param  x: 
     * @param  y: 
     * @retval double distance
     */
    static double distance(rrt_point x, rrt_point y)
    {
        return sqrt(pow((x.x - y.x), 2) + pow((x.y - y.y), 2));
    }

    /**
     * @brief  Get the nearest point in the RRT-tree
     * @note   The tree should not be NULL;
     * @param  &tree: the RRT-tree
     * @param  random_point: 
     * @retval the index of the nearest point 
     */
    int getNearestPoint(std::vector<rrt_point> &tree, rrt_point random_point)
    {
        int result = 0;
        double mini = 999999;
        for (int i = 0; i < tree.size(); i++)
        {
            double dis = distance(tree[i], random_point);
            if (dis < mini)
            {
                result = i;
                mini = dis;
            }
        }
        return result;
    }
    /**
     * @brief  Get a random point in the map
     * @note   the map if from (1873,1793) to (2213,2053)
     * @retval the random point
     */
    rrt_point getRandomPoint(void)
    {

        int ran_x = rand() % 340 + 1873;
        int ran_y = rand() % 260 + 1793;
        //int ran_x = rand() % 4000;
        //int ran_y = rand() % 4000;
        rrt_point rp;
        rp.x = ran_x;
        rp.y = ran_y;
        return rp;
    }
    rrt_point getNewPoint(rrt_point random_point, rrt_point nearest_point, int nearest, double step)
    {
        double delta_x = random_point.x - nearest_point.x;
        double delta_y = random_point.y - nearest_point.y;
        double normal = sqrt(pow(step, 2) / (pow(fabs(delta_x), 2) + pow(fabs(delta_y), 2)));

        double new_x = nearest_point.x + delta_x * normal;
        double new_y = nearest_point.y + delta_y * normal;

        rrt_point new_point = {nearest, new_x, new_y};
        return new_point;
    }
    rrt_point getNewPointPlus(rrt_point random_point, rrt_point nearest_point, rrt_point goal_point, int nearest, double step, double k)
    {
        double near_x = random_point.x - nearest_point.x;
        double near_y = random_point.y - nearest_point.y;
        double normal_near = sqrt(pow(step, 2) / (pow(fabs(near_x), 2) + pow(fabs(near_y), 2)));

        double potential_x = goal_point.x - nearest_point.x;
        double potential_y = goal_point.y - nearest_point.y;
        double normal_end = sqrt(pow(step, 2) / (pow(fabs(potential_x), 2) + pow(fabs(potential_y), 2)));

        double x = nearest_point.x + near_x * normal_near + potential_x * normal_end * k;
        double y = nearest_point.y + near_y * normal_near + potential_y * normal_end * k;

        rrt_point new_point = {nearest, x, y};
        return new_point;
    }
    bool checkPathOK(std::pair<float, float> a, std::pair<float, float> b, unsigned char *costs)
    {
        float ax = a.first, ay = a.second;
        float bx = b.first, by = b.second;
        //ROS_INFO("ax:%f ay:%f bx:%f by:%f", ax, ay, bx, by);
        float d = 0.5;
        if (bx < ax)
            d = -d;
        float k = ((by - ay) / (bx - ax));
        float ix = ax, iy = ay;
        for (ix = ax; std::abs(ix - bx) > 0.5; ix = ix + d)
        {
            if (costs[getIndex(ix, iy)] > 0)
            {
                return false;
            }
            iy = iy + d * k;
        }
        return true;
    }
    bool optimal(std::vector<std::pair<float, float>> &path, unsigned char *costs, size_t index)
    {
        //ROS_INFO("cut a line, index is %d", index);
        if (index >= (path.size() - 3))
            return true;
        while (checkPathOK(path[index], path[index + 1], costs))
        {
            if (index >= (path.size() - 3))
                break;
            path.erase(path.begin() + index + 1);
        }
        if (index >= (path.size() - 3))
            return true;
        optimal(path, costs, index + 1);
        return true;
    }
    double calDistance(std::vector<std::pair<float, float>> &path)
    {
        double distance = 0;
        double dx, dy;
        std::pair<float, float> temp, next;
        for (int i = 0; i < (path.size() - 1); i++)
        {
            temp = path[i];
            next = path[i + 1];
            dx = temp.first - next.first;
            dy = temp.second - next.second;
            distance += sqrt(pow(dx, 2) + pow(dy, 2));
        }
        return distance;
    }

  private:
    int ns_;
    double step;
    double k;
    bool smooth;
    bool use_potential;
};
} // namespace global_planner
#endif