#include "base_interfaces_demo/msg/student.hpp"
#include "rclcpp/rclcpp.hpp"
// 自定义节点类
class ListenerStu : public rclcpp::Node {
 public:
  ListenerStu() : Node("listener_stu_node_cpp") {
    subscription_ =
        this->create_subscription<base_interfaces_demo::msg::Student>(
            "chatter_stu", 10,
            std::bind(&ListenerStu::do_cb, this, std::placeholders::_1));
  }

 private:
  void do_cb(const base_interfaces_demo::msg::Student &msg) {
    RCLCPP_INFO(this->get_logger(), "接收消息:(%s, %d, %.2f)", msg.name.c_str(),
                msg.age, msg.height);
  }

  rclcpp::Subscription<base_interfaces_demo::msg::Student>::SharedPtr
      subscription_;
};

int main(int argc, char *argv[]) {
  // 初始化ROS2客户端
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ListenerStu>();
  // 调用spin函数,并传入节点对象指针
  rclcpp::spin(node);
  // 释放资源
  rclcpp::shutdown();
  return 0;
}