#ifndef RRT_H_
#define RRT_H_

#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <vector>

// 地图尺寸
constexpr int kImageSize = 20;
constexpr int kImageResolution = 50;

struct Point {
  Point(double x, double y) : x(x), y(y) {}
  double x = 0.0;
  double y = 0.0;
};

// 节点信息
class Node {
 public:
  Node(double x, double y) : point_(x, y), parent_(nullptr), cost_(0.0) {}
  const Point& point() const { return point_; }        // 节点坐标
  void set_parent(Node* parent) { parent_ = parent; }  // 设置父节点
  Node* parent() { return parent_; }  // 返回该节点的父节点

 private:
  Point point_;
  std::vector<double> path_x_;  // 路径
  std::vector<double> path_y_;
  Node* parent_ = nullptr;
  double cost_ = 0.0;
};

/*
        Setting Parameter
        start:起点 [x,y]
        goal:目标点 [x,y]
        obstacleList:障碍物位置列表 [[x,y,size],...]
        rand_area: 采样区域 x,y ∈ [min,max]
        play_area: 约束随机树的范围 [x_min,x_max,y_min,y_max]
        robot_radius: 机器人半径
        expand_dis: 扩展的步长
        goal_sample_rate: 采样目标点的概率，百分制.default: 5
   ,即表示5%的概率直接采样目标点
*/
class RRT {
 public:
  /*构造函数*/
  RRT(Node* start_node, Node* goal_node,
      const std::vector<std::vector<double>>& obstacle_list, int max_iterations,
      double step_size = 1.0, int goal_sample_rate = 5)
      : start_node_(start_node),        // 起点 [x,y]
        goal_node_(goal_node),          // 目标点 [x,y]
        obstacle_list_(obstacle_list),  // 障碍物位置列表
        step_size_(step_size),          // 扩展的步长
        goal_sample_rate_(
            goal_sample_rate),  // 采样目标点的概率,即5表示5%的概率直接采样目标点
        max_iterations_(max_iterations),  // 最大迭代次数
        goal_gen_(goal_rd_()),
        // 产生0-100之间的随机数
        goal_dis_(std::uniform_int_distribution<int>(0, 100)),
        area_gen_(area_rd_()),
        area_dis_(std::uniform_real_distribution<double>(0, 15)) {}

  /*计算起始点到目标点的路径*/
  std::vector<Node*> Planning();

 private:
  // 获取随机采样点
  std::vector<double> GetRandomSamplePoint();

  /*计算最邻近节点*/
  Node* GetNearestNode(const std::vector<double>& random_position);

  /*  计算节点连线是否穿过障碍物 */
  bool CollisionCheck(Node*);

  // 按照特定步长计算新节点new_node
  Node* GenerateNewNode(Node* nearest_node,
                        const std::vector<double>& random_position,
                        double extend_step);

  // 回溯父节点，生成最终路径
  std::vector<Node*> GetFinalPath(const std::vector<Node*> node_list);

  // 地图初始化
  void InitializeMap();

  // 可视化
  void VisualizeBackgroundPoint(const cv::Mat& map);

  // 可视化起始点、终点和障碍物
  void VisualizePlanMapPoint(const cv::Mat& map);

  // 当前节点和其父节点连线的可视化
  void PlotLine(Node* node, const cv::Scalar& value);

  // 计算两node点的距离
  double CalcNodeDist(const Node& start_node, const Node& end_node) {
    return sqrt(pow(start_node.point().x - end_node.point().x, 2) +
                pow(start_node.point().y - end_node.point().y, 2));
  }

 private:
  Node* start_node_;
  Node* goal_node_;
  std::vector<std::vector<double>> obstacle_list_;
  std::vector<Node*> node_list_;
  double step_size_;

  int goal_sample_rate_;
  cv::Mat plan_map_;    // 规划地图
  int max_iterations_;  // 最大迭代次数

  /*
   random_device:均匀分布整数随机数生成器,生成非确定性的随机自然数
  uniform_int_distribution:指定范围的非负数,接受两个值，表示随机数的分布范围（闭区间）
  uniform_real_distribution：指定范围的随机实数
  std::mt19937:mt19937类是一个随机数引擎，可以生成高质量的伪随机数序列
  */
  std::random_device goal_rd_;  //    产生种子
  std::mt19937 goal_gen_;  // 伪随机数产生器,用于产生高性能的随机数
  std::uniform_int_distribution<int> goal_dis_;  // 特定范围的非负数

  std::random_device area_rd_;
  std::mt19937 area_gen_;
  std::uniform_real_distribution<double> area_dis_;
};

#endif
