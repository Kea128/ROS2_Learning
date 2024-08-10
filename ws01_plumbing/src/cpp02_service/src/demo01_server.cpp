/*
  需求：编写服务端，接收客户端发送请求，提取其中两个整型数据，相加后将结果响应回客户端。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建服务端；
      3-2.处理请求数据并响应结果。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
*/

#include "base_interfaces_demo/srv/add_ints.hpp"
#include "rclcpp/rclcpp.hpp"

// 自定义节点类
class AddIntsServer : public rclcpp::Node {
 public:
  AddIntsServer() : Node("add_ints_server_node_cpp") {
    RCLCPP_INFO(this->get_logger(), "服务端节点创建");
    service_ = this->create_service<base_interfaces_demo::srv::AddInts>(
        "add_ints", std::bind(&AddIntsServer::add_ints_callback, this,
                              std::placeholders::_1, std::placeholders::_2));
  }

 private:
  void add_ints_callback(
      const base_interfaces_demo::srv::AddInts::Request::SharedPtr request,
      const base_interfaces_demo::srv::AddInts::Response::SharedPtr response) {
    response->sum = request->num1 + request->num2;
  }

  rclcpp::Service<base_interfaces_demo::srv::AddInts>::SharedPtr service_;
};

int main(int argc, char *argv[]) {
  // 初始化ROS2客户端
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddIntsServer>();
  // 调用spin函数,并传入节点对象指针
  rclcpp::spin(node);
  // 释放资源
  rclcpp::shutdown();
  return 0;
}