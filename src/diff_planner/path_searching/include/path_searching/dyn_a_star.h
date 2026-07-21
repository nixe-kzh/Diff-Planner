#ifndef DIFF_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_DYN_A_STAR_H_
#define DIFF_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_DYN_A_STAR_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>

constexpr double kInfinity = 1 >> 20;
struct GridNode;
typedef GridNode *GridNodePtr;

enum AStarResult
{
	kSuccess,
	kInitializationError,
	kSearchError
};

struct GridNode
{
	enum NodeState
	{
		kOpenSet = 1,
		kClosedSet = 2,
		kUndefined = 3
	};

	int search_round{0}; // Distinguish every call
	enum NodeState state
	{
		kUndefined
	};
	Eigen::Vector3i index;

	double g_score{kInfinity}, f_score{kInfinity};
	GridNodePtr came_from{NULL};
};

class GridNodeComparator
{
public:
	bool operator()(GridNodePtr first_node, GridNodePtr second_node)
	{
		return first_node->f_score > second_node->f_score;
	}
};

class AStar
{
private:
	GridMap::Ptr grid_map_;

	inline void CoordinateToGridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	double DiagonalHeuristic(GridNodePtr first_node, GridNodePtr second_node);
	double ManhattanHeuristic(GridNodePtr first_node, GridNodePtr second_node);
	double EuclideanHeuristic(GridNodePtr first_node, GridNodePtr second_node);
	inline double Heuristic(GridNodePtr first_node, GridNodePtr second_node);

	bool ConvertToIndicesAndAdjustEndpoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	inline Eigen::Vector3d IndexToCoordinate(const Eigen::Vector3i &index) const;
	inline bool CoordinateToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	// bool (*check_occupancy)(const Eigen::Vector3d &pos);

	inline int CheckOccupancy(const Eigen::Vector3d &pos) { return grid_map_->GetInflatedOccupancy(pos); }

	std::vector<GridNodePtr> RetrievePath(GridNodePtr current);

	double step_size_, inverse_step_size_;
	Eigen::Vector3d center_;
	Eigen::Vector3i center_index_, pool_size_;
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> grid_path_;

	GridNodePtr ***grid_node_map_;
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, GridNodeComparator> open_set_;

	int search_round_{0};

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void InitializeGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

	AStarResult Search(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

	std::vector<Eigen::Vector3d> GetPath();
};

inline double AStar::Heuristic(GridNodePtr first_node, GridNodePtr second_node)
{
	return tie_breaker_ * DiagonalHeuristic(first_node, second_node);
}

inline Eigen::Vector3d AStar::IndexToCoordinate(const Eigen::Vector3i &index) const
{
	return ((index - center_index_).cast<double>() * step_size_) + center_;
};

inline bool AStar::CoordinateToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	idx = ((pt - center_) * inverse_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + center_index_;

	if (idx(0) < 0 || idx(0) >= pool_size_(0) || idx(1) < 0 || idx(1) >= pool_size_(1) || idx(2) < 0 || idx(2) >= pool_size_(2))
	{
		ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}

	return true;
};

#endif  // DIFF_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_DYN_A_STAR_H_
