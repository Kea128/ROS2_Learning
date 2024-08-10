/*
    需求：编写参数服务端，设置并操作参数。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建参数客户端对象；
            3-2.连接服务器；
            3-3.查询参数；
            3-4.修改参数；
        4.创建节点对象指针，调用参数操作函数；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// 自定义节点类
class ParamClient : public rclcpp::Node {
 public:
  ParamClient() : Node("param_client_node_cpp") {
    RCLCPP_INFO(this->get_logger(), "参数客户端创建");
    // 3-1.创建参数客户端对象；
    // 参数1：当前对象依赖的节点
    // 参数2：参数服务端节点名称
    param_client_ = std::make_shared<rclcpp::SyncParametersClient>(
        this, "param_server_node_cpp");
  }

  // 3-2.连接服务器；
  bool connect_server() {
    while (!param_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "参数服务连接中");
    }
    return true;
  }

  // 3-3.查询参数；
  void get_param() {
    RCLCPP_INFO(this->get_logger(), "-----------参数客户端查询参数-----------");
    double height = param_client_->get_parameter<double>("height");
    RCLCPP_INFO(this->get_logger(), "height = %.2f", height);
    RCLCPP_INFO(this->get_logger(), "car_type 存在吗？%d",
                param_client_->has_parameter("car_type"));
    auto params =
        param_client_->get_parameters({"car_type", "height", "wheels"});
    for (auto &&param : params) {
      RCLCPP_INFO(this->get_logger(), "%s = %s", param.get_name().c_str(),
                  param.value_to_string().c_str());
    }
  }
  // 3-2.修改参数；
  void update_param() {
    RCLCPP_INFO(this->get_logger(), "-----------参数客户端修改参数-----------");
    param_client_->set_parameters(
        {rclcpp::Parameter("car_type", "Mouse"),
         rclcpp::Parameter("height", 2.0),
         //这是服务端不存在的参数，只有服务端设置了rclcpp::NodeOptions().allow_undeclared_parameters(true)时，
         // 这个参数才会被成功设置。
         rclcpp::Parameter("width", 0.15), rclcpp::Parameter("wheels", 6)});

    /*
    问题：服务通讯不是通过服务话题关联吗?为啥参数客户端是通过参数服务端的节点名称关联的？
    答：
       1.参数服务端启动后，底层封装了多个服务通讯的服务端
       2.每个服务端的话题，都是采用了 /服务端节点名称/xxxx 的格式
       3.参数客户端创建后，也会封装多个服务通 讯的客户端
       4.这些客户端与服务端相呼应，也要使用相同的话题，因此客户端创建时需要使用服务端的
    <节点名称>
     */
  }

 private:
  rclcpp::SyncParametersClient::SharedPtr param_client_;
};

int main(int argc, char *argv[]) {
  // 初始化ROS2客户端
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParamClient>();
  bool flag = node->connect_server();
  if (!flag) {
    return 0;
  }
  node->get_param();
  node->update_param();
  node->get_param();
  // 调用spin函数,并传入节点对象指针
  // rclcpp::spin(node);
  // 释放资源
  rclcpp::shutdown();
  return 0;
}