/*
需求：以某个固定频率发送文本“hello
world!”，文本后缀编号，每发送一条消息，编号递增1。 步骤： 1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建发布方；
      3-2.创建定时器；
      3-3.组织消息并发布。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// 3.自定义节点类;
class Talker : public rclcpp::Node {
 public:
  Talker() : Node("talker_node_cpp"), count(0) {
    RCLCPP_INFO(this->get_logger(), "发布节点创建");
    // 3-1.创建发布方
    /*
      模板：被发布的消息类型；
      参数：
        1.话题名称
        2.QOS（消息队列长度）
      返回值：发布对象指针
     */
    publisher_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    // 3-2.创建定时器
    /*
      参数：
        1.时间间隔
        2.回调函数
      返回值：定时器对象指针
     */
    timer_ = create_wall_timer(1s, std::bind(&Talker::on_timer, this));
  }

 private:
  void on_timer() {
    // 3-3.组织消息并发布
    auto message = std_msgs::msg::String();
    message.data = "hello world!" + std::to_string(count++);
    RCLCPP_INFO(this->get_logger(), "发布消息:%s", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count;
};

int main(int argc, char *argv[]) {
  // 2.初始化 ROS2 客户端；
  rclcpp::init(argc, argv);

  // 创建节点对象指针
  auto node = std::make_shared<Talker>();

  // 4.调用spin函数，并传入节点对象指针；
  rclcpp::spin(node);

  // 5.释放资源。
  rclcpp::shutdown();

  return 0;
}
