#include "base_interfaces_demo/msg/student.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// 自定义节点类
class TalkerStu : public rclcpp::Node {
 public:
  TalkerStu() : Node("talker_stu_node_cpp") {
    RCLCPP_INFO(this->get_logger(), "自定义消息发布节点创建");
    publisher_ = this->create_publisher<base_interfaces_demo::msg::Student>(
        "chatter_stu", 10);
    timer_ =
        this->create_wall_timer(500ms, std::bind(&TalkerStu::on_timer, this));
  }

 private:
  void on_timer() {
    static base_interfaces_demo::msg::Student stu;
    stu.name = "jack";
    stu.age += 1;
    stu.height = 1.75;
    publisher_->publish(stu);
    RCLCPP_INFO(this->get_logger(), "发布消息:(%s, %d, %.2f)", stu.name.c_str(),
                stu.age, stu.height);
  };
  rclcpp::Publisher<base_interfaces_demo::msg::Student>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  // 初始化ROS2客户端
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TalkerStu>();
  // 调用spin函数,并传入节点对象指针
  rclcpp::spin(node);
  // 释放资源
  rclcpp::shutdown();
  return 0;
}