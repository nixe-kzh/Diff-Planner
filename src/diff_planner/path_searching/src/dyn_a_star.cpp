#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < pool_size_(0); i++)
        for (int j = 0; j < pool_size_(1); j++)
            for (int k = 0; k < pool_size_(2); k++)
                delete grid_node_map_[i][j][k];
}

void AStar::InitializeGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    pool_size_ = pool_size;
    center_index_ = pool_size / 2;

    grid_node_map_ = new GridNodePtr **[pool_size_(0)];
    for (int i = 0; i < pool_size_(0); i++)
    {
        grid_node_map_[i] = new GridNodePtr *[pool_size_(1)];
        for (int j = 0; j < pool_size_(1); j++)
        {
            grid_node_map_[i][j] = new GridNodePtr[pool_size_(2)];
            for (int k = 0; k < pool_size_(2); k++)
            {
                grid_node_map_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}

double AStar::DiagonalHeuristic(GridNodePtr first_node, GridNodePtr second_node)
{
    double dx = abs(first_node->index(0) - second_node->index(0));
    double dy = abs(first_node->index(1) - second_node->index(1));
    double dz = abs(first_node->index(2) - second_node->index(2));

    double h = 0.0;
    int diagonal = min(min(dx, dy), dz);
    dx -= diagonal;
    dy -= diagonal;
    dz -= diagonal;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diagonal + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diagonal + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diagonal + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::ManhattanHeuristic(GridNodePtr first_node, GridNodePtr second_node)
{
    double dx = abs(first_node->index(0) - second_node->index(0));
    double dy = abs(first_node->index(1) - second_node->index(1));
    double dz = abs(first_node->index(2) - second_node->index(2));

    return dx + dy + dz;
}

double AStar::EuclideanHeuristic(GridNodePtr first_node, GridNodePtr second_node)
{
    return (second_node->index - first_node->index).norm();
}

vector<GridNodePtr> AStar::RetrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->came_from != NULL)
    {
        current = current->came_from;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndicesAndAdjustEndpoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (!CoordinateToIndex(start_pt, start_idx) || !CoordinateToIndex(end_pt, end_idx))
        return false;

    int occupancy;
    if (CheckOccupancy(IndexToCoordinate(start_idx)))
    {
        // ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            // cout << "start_pt=" << start_pt.transpose() << endl;
            if (!CoordinateToIndex(start_pt, start_idx))
            {
                return false;
            }

            occupancy = CheckOccupancy(IndexToCoordinate(start_idx));
            if (occupancy == -1)
            {
                ROS_WARN("[Astar] Start point outside the map region.");
                return false;
            }
        } while (occupancy);
    }

    if (CheckOccupancy(IndexToCoordinate(end_idx)))
    {
        // ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            // cout << "end_pt=" << end_pt.transpose() << endl;
            if (!CoordinateToIndex(end_pt, end_idx))
            {
                return false;
            }

            occupancy = CheckOccupancy(IndexToCoordinate(end_idx));
            if (occupancy == -1)
            {
                ROS_WARN("[Astar] End point outside the map region.");
                return false;
            }
        } while (CheckOccupancy(IndexToCoordinate(end_idx)));
    }

    return true;
}

AStarResult AStar::Search(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();
    ++search_round_;

    step_size_ = step_size;
    inverse_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndicesAndAdjustEndpoints(start_pt, end_pt, start_idx, end_idx))
    {
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return AStarResult::kInitializationError;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr start_node = grid_node_map_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr end_node = grid_node_map_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, GridNodeComparator> empty;
    open_set_.swap(empty);

    GridNodePtr neighbor_node = NULL;
    GridNodePtr current = NULL;

    end_node->index = end_idx;

    start_node->index = start_idx;
    start_node->search_round = search_round_;
    start_node->g_score = 0;
    start_node->f_score = Heuristic(start_node, end_node);
    start_node->state = GridNode::kOpenSet; //put start node in open set
    start_node->came_from = NULL;
    open_set_.push(start_node); //put start in open set

    double tentative_g_score;

    int iteration_count = 0;
    while (!open_set_.empty())
    {
        iteration_count++;
        current = open_set_.top();
        open_set_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == end_node->index(0) && current->index(1) == end_node->index(1) && current->index(2) == end_node->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",iteration_count, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            grid_path_ = RetrievePath(current);
            return AStarResult::kSuccess;
        }
        current->state = GridNode::kClosedSet; //move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    Vector3i neighbor_index;
                    neighbor_index(0) = (current->index)(0) + dx;
                    neighbor_index(1) = (current->index)(1) + dy;
                    neighbor_index(2) = (current->index)(2) + dz;

                    if (neighbor_index(0) < 1 || neighbor_index(0) >= pool_size_(0) - 1 || neighbor_index(1) < 1 || neighbor_index(1) >= pool_size_(1) - 1 || neighbor_index(2) < 1 || neighbor_index(2) >= pool_size_(2) - 1)
                    {
                        continue;
                    }

                    neighbor_node = grid_node_map_[neighbor_index(0)][neighbor_index(1)][neighbor_index(2)];
                    neighbor_node->index = neighbor_index;

                    bool was_explored = neighbor_node->search_round == search_round_;

                    if (was_explored && neighbor_node->state == GridNode::kClosedSet)
                    {
                        continue; //in closed set.
                    }

                    neighbor_node->search_round = search_round_;

                    if (CheckOccupancy(IndexToCoordinate(neighbor_node->index)))
                    {
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_g_score = current->g_score + static_cost;

                    if (!was_explored)
                    {
                        //discover a new node
                        neighbor_node->state = GridNode::kOpenSet;
                        neighbor_node->came_from = current;
                        neighbor_node->g_score = tentative_g_score;
                        neighbor_node->f_score = tentative_g_score + Heuristic(neighbor_node, end_node);
                        open_set_.push(neighbor_node); //put neighbor in open set and record it.
                    }
                    else if (tentative_g_score < neighbor_node->g_score)
                    { //in open set and need update
                        neighbor_node->came_from = current;
                        neighbor_node->g_score = tentative_g_score;
                        neighbor_node->f_score = tentative_g_score + Heuristic(neighbor_node, end_node);
                    }
                }
        ros::Time time_2 = ros::Time::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return AStarResult::kSearchError;
        }
    }

    ros::Time time_2 = ros::Time::now();

    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), iteration_count);

    return AStarResult::kSearchError;
}

vector<Vector3d> AStar::GetPath()
{
    vector<Vector3d> path;

    for (auto node : grid_path_)
        path.push_back(IndexToCoordinate(node->index));

    reverse(path.begin(), path.end());
    return path;
}
