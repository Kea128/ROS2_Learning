/*
需求：在终端输出文本hello world
流传：
	1.包含头文件
	2.初始化ROS2客户端
	3.创建节点指针
	4.输出日志
	5.释放资源

*/

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS2
  rclcpp::init(argc,argv);
  // 创建节点
  auto node = rclcpp::Node::make_shared("hellovscode_node_cpp");
  // 输出文本
  RCLCPP_INFO(node->get_logger(),"hello vscode!");
  // 释放资源
  rclcpp::shutdown();
  return 0;
}