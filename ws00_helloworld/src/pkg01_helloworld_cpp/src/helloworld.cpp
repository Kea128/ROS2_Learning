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



// 方式1（不推荐）
/* int main(int argc, char ** argv)
{
  // 初始化 ROS2
  rclcpp::init(argc,argv);
  // 创建节点 
  auto node = rclcpp::Node::make_shared("helloworld_node");
  // 输出文本
  RCLCPP_INFO(node->get_logger(),"hello world!");
  // 释放资源
  rclcpp::shutdown();
  return 0;
} */

// 方式2（推荐）
// 自定义类继承 Node
class MyNode: public rclcpp::Node{
public:
    MyNode():Node("hello_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"hello world!(继承的方式)");
    }
};

int main(int argc, char const *argv[])
{
  // 初始化 ROS2
  rclcpp::init(argc,argv);
  // 实例化自定义类
  auto node = std::make_shared<MyNode>();
  // 释放资源
  rclcpp::shutdown();
  return 0;
}

/* 
问题：初始化和资源释放在程序中起什么作用
答：
      1.前提：构建的程序可能由若干步骤或阶段组成：
        初始化-->节点对象-->日志输出-->数据发布-->数据订阅-->...-->资源释放
      2.不同步骤或阶段之间涉及到数据的传递
      3.怎么实现数据的传递？
        使用Context（上下文）对象，这是一个容器，可以存储数据，也可以从中读取数据
      4.初始化其实就是要创建Context对象，资源释放就是要销毁Context对象
*/
