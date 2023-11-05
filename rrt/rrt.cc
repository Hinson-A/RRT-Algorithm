#include "rrt.h"

// 计算距离随机点最近的节点
Node* RRT::GetNearestNode(const std::vector<double>& random_position) {
  int min_id = -1;
  double min_distance = std::numeric_limits<double>::max();
  for (int i = 0; i < node_list_.size(); i++) {
    double square_distance =
        std::pow(node_list_[i]->point().x - random_position[0], 2) +
        std::pow(node_list_[i]->point().y - random_position[1], 2);
    if (square_distance < min_distance) {
      min_distance = square_distance;
      min_id = i;
    }
  }

  return node_list_[min_id];
}

// 碰撞检测
bool RRT::CollisionCheck(Node* new_node) {
  for (auto item : obstacle_list_) {
    // 候选点到障碍物中心 （圆心）的距离小于半径则碰撞
    if (std::sqrt(std::pow((item[0] - new_node->point().x), 2) +
                  std::pow((item[1] - new_node->point().y), 2)) <= item[2])
      return false;
  }
  return true;
}

// 按照特定步长计算新节点new_node
Node* RRT::GenerateNewNode(Node* nearest_node,
                           const std::vector<double>& random_position,
                           double extend_step) {
  double theta = atan2(random_position[1] - nearest_node->point().y,
                       random_position[0] - nearest_node->point().x);
  Node* new_node = new Node(nearest_node->point().x + step_size_ * cos(theta),
                            nearest_node->point().y + step_size_ * sin(theta));
  new_node->set_parent(nearest_node);
  return new_node;
}

// 获取随机采样点
std::vector<double> RRT::GetRandomSamplePoint() {
  /*
RRT的生长方向有两种：随机&朝向targetNode。程序中，goal_dis在[0,100]范围内等概率的生成一个整数，
以（100-goal_sample_rate）%的概率随机生长，(goal_sample_rate)%的概率朝向目标点生长
*/
  std::vector<double> random_position;
  if (goal_dis_(goal_gen_) > goal_sample_rate_) {
    // 产生随机点
    double randX = area_dis_(goal_gen_);
    double randY = area_dis_(goal_gen_);
    random_position.push_back(randX);
    random_position.push_back(randY);
  } else {
    // 目标点生长
    random_position.push_back(goal_node_->point().x);
    random_position.push_back(goal_node_->point().y);
  }
  return random_position;
}

// 地图初始化
void RRT::InitializeMap() {
  plan_map_ =
      cv::Mat(kImageSize * kImageResolution, kImageSize * kImageResolution,
              CV_8UC3, cv::Scalar(255, 255, 255));
  VisualizePlanMapPoint(plan_map_);
}

/*规划计算函数*/
std::vector<Node*> RRT::Planning() {
  cv::namedWindow("RRT");
  // 初始化地图
  InitializeMap();
  int iter_count = 0;
  std::vector<Node*> path;  // 起始点到目标点的路径
  bool is_find_path = false;
  node_list_.push_back(start_node_);

  while (iter_count < max_iterations_) {
    // Step1: 获取随机采样点
    std::vector<double> random_position = GetRandomSamplePoint();

    // Step2:  计算距离随机点最近的节点
    Node* nearest_node = GetNearestNode(random_position);

    // Step3: 按照特定步长计算新节点
    Node* new_node = GenerateNewNode(nearest_node, random_position, step_size_);

    // Step4:对新节点进行碰撞检测
    if (!CollisionCheck(new_node)) continue;
    node_list_.push_back(new_node);

    // 扩展点的可视化
    cv::Scalar color = cv::Scalar(0, 255, 0);  // green
    PlotLine(new_node, color);

    iter_count++;
    imshow("RRT", plan_map_);
    cv::waitKey(5);

    // Step5: 检查新生成的结点到目标点的距离小于一个步长，则终止树的生长
    if (CalcNodeDist(*new_node, *goal_node_) <= step_size_) {
      is_find_path = true;
      std::cout << "The path has been found!" << std::endl;
      break;
    }
  }

  if (!is_find_path) {
    std::cout << "Error: The path has not been found!" << std::endl;
    return path;
  }
  // Step6: 起始点到目标点的路径回溯及可视化
  path = GetFinalPath(node_list_);

  std::cout << "path size:" << path.size() << std::endl;
  imshow("RRT", plan_map_);
  cv::waitKey(0);

  return path;
}

// 回溯父节点,得到最终路径
std::vector<Node*> RRT::GetFinalPath(const std::vector<Node*> node_list) {
  std::vector<Node*> path;
  path.emplace_back(goal_node_);
  Node* temp_node = node_list.back();

  while (temp_node->parent() != nullptr) {
    cv::Scalar color = cv::Scalar(255, 0, 255);  //
    PlotLine(temp_node, color);
    path.emplace_back(temp_node);
    temp_node = temp_node->parent();
  }
  path.push_back(start_node_);
  return path;
}

// 当前节点和其父节点连线的可视化
void RRT::PlotLine(Node* node, const cv::Scalar& value) {
  line(
      plan_map_,
      cv::Point(static_cast<int>(node->point().x * kImageResolution),
                static_cast<int>(node->point().y * kImageResolution)),
      cv::Point(static_cast<int>(node->parent()->point().x * kImageResolution),
                static_cast<int>(node->parent()->point().y * kImageResolution)),
      value, 10);
}

// 可视化起始点、终点和障碍物
void RRT::VisualizePlanMapPoint(const cv::Mat& map) {
  circle(map,
         cv::Point(start_node_->point().x * kImageResolution,
                   start_node_->point().y * kImageResolution),
         20, cv::Scalar(0, 0, 255), -1);
  circle(map,
         cv::Point(goal_node_->point().x * kImageResolution,
                   goal_node_->point().y * kImageResolution),
         20, cv::Scalar(255, 0, 0), -1);

  for (auto& item : obstacle_list_) {
    circle(map,
           cv::Point(item[0] * kImageResolution, item[1] * kImageResolution),
           item[2] * kImageResolution, cv::Scalar(0, 0, 0), -1);
  }
}
