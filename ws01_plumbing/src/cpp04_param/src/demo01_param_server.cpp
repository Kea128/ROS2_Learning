/*
    需求：编写参数服务端，设置并操作参数。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.增；
            3-2.删；
            3-3.改；
            3-4.查；
        4.创建节点对象指针，调用参数操作函数，并传递给spin函数；
        5.释放资源。

*/

#include "rclcpp/rclcpp.hpp"
// 自定义节点类
class ParamServer : public rclcpp::Node {
 public:
  // 如果允许删除参数，需要通过NodeOptions声明
  // 一个节点就可以作为参数服务端存在
  ParamServer()
      : Node("param_server_node_cpp",
             rclcpp::NodeOptions().allow_undeclared_parameters(true)) {
    RCLCPP_INFO(this->get_logger(), "参数服务端创建");
  }
  // 3-1.增；
  void declare_param() {
    // 声明参数并设置默认值
    RCLCPP_INFO(this->get_logger(), "------------------增----------------");
    this->declare_parameter("car_type", "Tiger");
    this->declare_parameter("height", 1.50);
    this->declare_parameter("wheels", 4);
    // set_parameter需要设置
    // rclcpp::NodeOptions().allow_undeclared_parameters(true),否则非法
    this->set_parameter(rclcpp::Parameter("undcl_test", 100));
  }
  // 3-4.查；
  void get_param() {
    RCLCPP_INFO(this->get_logger(), "------------------查----------------");
    // 获取指定
    rclcpp::Parameter car_type = this->get_parameter("car_type");
    RCLCPP_INFO(this->get_logger(), "car_type:%s",
                car_type.as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "height:%.2f",
                this->get_parameter("height").as_double());
    RCLCPP_INFO(this->get_logger(), "wheels:%ld",
                this->get_parameter("wheels").as_int());
    RCLCPP_INFO(this->get_logger(), "undcl_test:%ld",
                this->get_parameter("undcl_test").as_int());
    // 判断包含
    RCLCPP_INFO(this->get_logger(), "包含car_type? %d",
                this->has_parameter("car_type"));
    RCLCPP_INFO(this->get_logger(), "包含car_typesxxxx? %d",
                this->has_parameter("car_typexxxx"));
    // 获取所有
    auto params = this->get_parameters({"car_type", "height", "wheels"});
    for (auto &param : params) {
      RCLCPP_INFO(this->get_logger(), "name = %s, value = %s",
                  param.get_name().c_str(), param.value_to_string().c_str());
    }
  }
  // 3-3.改；
  void update_param() {
    RCLCPP_INFO(this->get_logger(), "------------------改----------------");
    this->set_parameter(rclcpp::Parameter("height", 1.75));
    RCLCPP_INFO(this->get_logger(), "height:%.2f",
                this->get_parameter("height").as_double());
  }
  // 3-2.删；
  void del_param() {
    RCLCPP_INFO(this->get_logger(), "------------------删----------------");
    // this->undeclare_parameter("car_type"); 不能删除声明的参数
    // RCLCPP_INFO(this->get_logger(),"删除操作后，car_type还存在马?
    // %d",this->has_parameter("car_type"));
    RCLCPP_INFO(this->get_logger(), "删除操作前，undcl_test存在马? %d",
                this->has_parameter("undcl_test"));
    this->undeclare_parameter("undcl_test");  //可以删除没有声明然后设置的参数
    RCLCPP_INFO(this->get_logger(), "删除操作前，undcl_test存在马? %d",
                this->has_parameter("undcl_test"));
  }

 private:
};

int main(int argc, char *argv[]) {
  // 初始化ROS2客户端
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParamServer>();
  node->declare_param();
  node->get_param();
  node->update_param();
  node->del_param();

  // 调用spin函数,并传入节点对象指针
  rclcpp::spin(node);
  // 释放资源
  rclcpp::shutdown();
  return 0;
}