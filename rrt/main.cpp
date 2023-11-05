#include "rrt.h"

int main(int argc, char* argv[]) {
  // (x, y, r)
  // 初始化设置障碍物，( 圆心，半径)
  std::vector<std::vector<double>> obstacle_list{
      {7, 5, 1}, {5, 6, 2}, {5, 8, 2}, {5, 10, 2}, {9, 5, 2}, {11, 5, 2}};

  // 起始点和终点坐标
  Node* start_node = new Node(2.0, 2.0);
  Node* goal_node = new Node(14.0, 15.0);
  int max_iterations = 500;  // 最大迭代次数

  // 这里设置扩展步长为0.5，采样目标点的概率为5%
  RRT rrt(start_node, goal_node, obstacle_list, max_iterations, 0.5, 5);
  rrt.Planning();
  return 0;
}