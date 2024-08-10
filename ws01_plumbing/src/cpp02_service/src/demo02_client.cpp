/*
  需求：编写客户端，发送两个整型变量作为请求数据，并处理响应结果。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建客户端；
      3-2.等待服务连接；
      3-3.组织请求数据并发送；
    4.创建对象指针调用其功能,并处理响应；
    5.释放资源。
*/

#include "base_interfaces_demo/srv/add_ints.hpp"
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;

// 自定义节点类
class AddIntsClient : public rclcpp::Node {
 public:
  AddIntsClient() : Node("add_ints_client_node_cpp") {
    RCLCPP_INFO(this->get_logger(), "客户端节点创建");

    // 3-1.创建客户端对象
    client_ =
        this->create_client<base_interfaces_demo::srv::AddInts>("add_ints");
  }
  // 3-2.等待服务连接
  bool connect_server() {
    // client_->wait_for_service(1s);在指定超时时间内连接服务器，如果连接成功则返回true，否则返回false
    // 循环以1s为超时时间连接服务器，直到连接到服务器则退出循环
    while (!client_->wait_for_service(2s)) {
      // 对ctrl+C进行捕获，退出循环
      // 1.怎么判断ctrl+C按下？ 2.如何处理
      // 正常执行rclcpp::ok()返回true，按下ctrl+C后rclcpp::ok()返回false
      // 按下ctrl+C是结束ROS2程序，意味着腰释放资源，比如:关闭context
      // 后续就不能再调用this->get_logger()
      if (!rclcpp::ok()) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "客户端强行退出");
        return false;
      }

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接中...");
    }
    return true;
  }

  // 3-3.组织请求数据并发送
  // 编写发送请求函数，参数为两个整型变量，返回值为提交请求后服务端的响应结果
  rclcpp::Client<base_interfaces_demo::srv::AddInts>::FutureAndRequestId
  send_request(int num1, int num2) {
    // 3-3.组织请求数据
    /*
      rclcpp::Client<base_interfaces_demo::srv::AddInts>::FutureAndRequestId
      async_send_request(std::shared_ptr<base_interfaces_demo::srv::AddInts_Request>
      request)
     */

    auto request =
        std::make_shared<base_interfaces_demo::srv::AddInts_Request>();
    request->num1 = num1;
    request->num2 = num2;
    return client_->async_send_request(request);
  }

 private:
  rclcpp::Client<base_interfaces_demo::srv::AddInts>::SharedPtr
      client_;  // 客户端对象>
};

int main(int argc, char *argv[]) {
  if (argc != 3) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交两个整数作为参数");
    return 1;
  }

  // 初始化ROS2客户端
  rclcpp::init(argc, argv);

  // 创建客户端对象节点
  auto node = std::make_shared<AddIntsClient>();

  // 调用客户端对象的连接服务器功能
  bool flag = node->connect_server();
  if (!flag) {
    /*
      rclcpp::get_logger("name") 创建logger对象不依赖于context
     */
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "服务连接失败");
    return 0;
  }
  // 调用请求提交函数，接收并处理相应结果
  auto future = node->send_request(std::atoi(argv[1]), atoi(argv[2]));
  // 处理相应
  // 如果成功
  if (rclcpp::spin_until_future_complete(node, future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "请求正常处理");
    RCLCPP_INFO(node->get_logger(), "响应结果:%d!", future.get()->sum);
  } else {
    RCLCPP_ERROR(node->get_logger(), "相应失败");
  }
  // 释放资源
  rclcpp::shutdown();
  return 0;
}