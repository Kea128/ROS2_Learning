/*
  需求：编写动作客户端实现，可以提交一个整型数据到服务端，并处理服务端的连续反馈以及最终返回结果。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建动作客户端；
      3-2.发送请求；
      3-3.处理关于目标值的服务端响应(回调函数)；
      3-4.处理连续反馈(回调函数)；
      3-5.处理最终响应(回调函数)。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
*/

// 这里还有bug,取消客户端,服务端不停止计算

#include "base_interfaces_demo/action/progress.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

// 自定义节点类
class ProgressActionClient : public rclcpp::Node {
 public:
  ProgressActionClient() : Node("progress_action_client_node_cpp") {
    RCLCPP_INFO(this->get_logger(), "action客户端节点创建");
    // 3-1.创建动作客户端
    /*
    1/2
    -rclcpp_action::Client<ActionT>::SharedPtr
    -create_client<ActionT, NodeT>(
    -NodeT node,
    -const std::string &name,
    -rclcpp::CallbackGroup::SharedPtr group = nullptr,
    -const rcl_action_client_options_t &options =
    rcl_action_client_get_default_options())
     */
    client_ =
        rclcpp_action::create_client<base_interfaces_demo::action::Progress>(
            this, "get_sum");
  }
  // 3-2.发送请求；
  void send_goal(int num) {
    // 1.需要连接服务端
    if (!client_->wait_for_action_server(3s)) {
      RCLCPP_ERROR(this->get_logger(), "服务连接失败");
      return;
    }

    // 2.发送目标值
    /*
    // client_->async_send_goal(）
    std::shared_future<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Progress>::SharedPtr>
    async_send_goal(
      const base_interfaces_demo::action::Progress::Goal &goal,
      const
    rclcpp_action::Client<base_interfaces_demo::action::Progress>::SendGoalOptions
    &options)
     */

    auto goal = base_interfaces_demo::action::Progress::Goal();
    goal.num = num;
    rclcpp_action::Client<
        base_interfaces_demo::action::Progress>::SendGoalOptions options;
    options.goal_response_callback =
        std::bind(&ProgressActionClient::goal_response_callback, this,
                  std::placeholders::_1);
    options.feedback_callback =
        std::bind(&ProgressActionClient::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);
    options.result_callback = std::bind(&ProgressActionClient::result_callback,
                                        this, std::placeholders::_1);
    auto future = client_->async_send_goal(goal, options);
  }

  // 3-3.处理关于目标值的服务端响应(回调函数)；
  /*
    using GoalResponseCallback = std::function<void (typename
    GoalHandle::SharedPtr)>;
  */
  void goal_response_callback(
      rclcpp_action::ClientGoalHandle<
          base_interfaces_demo::action::Progress>::SharedPtr goal_handle) {
    // 如果是空指针
    if (!goal_handle) {
      RCLCPP_INFO(this->get_logger(), "目标请求被服务端拒绝");
    } else {
      RCLCPP_INFO(this->get_logger(), "目标处理中");
    }
  }

  // 3-4.处理连续反馈(回调函数)；
  /*  using FeedbackCallback =
      std::function<void (
          typename ClientGoalHandle<ActionT>::SharedPtr,
          const std::shared_ptr<const Feedback>)>;
  */
  void feedback_callback(
      rclcpp_action::ClientGoalHandle<
          base_interfaces_demo::action::Progress>::SharedPtr goal_handle,
      const std::shared_ptr<
          const base_interfaces_demo::action::Progress::Feedback>
          feedback) {
    (void)goal_handle;
    double progress = feedback->progress * 100;
    int pro = (int)progress;
    RCLCPP_INFO(this->get_logger(), "当前进度: [%d%%]", pro);
  }

  // 3-5.处理最终响应(回调函数)。
  /*
    using ResultCallback = std::function<void (const WrappedResult & result)>;
   */
  void result_callback(
      const rclcpp_action::ClientGoalHandle<
          base_interfaces_demo::action::Progress>::WrappedResult &result) {
    // result,code
    // 通过状态码判断最终结果状态
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "最终结果：%d ", result.result->sum);
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_INFO(this->get_logger(), "被中断");
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_INFO(this->get_logger(), "被取消");
    } else {
      RCLCPP_INFO(this->get_logger(), "未知异常");
    }
  }

 private:
  rclcpp_action::Client<base_interfaces_demo::action::Progress>::SharedPtr
      client_;
};

int main(int argc, char *argv[]) {
  if (argc != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交一个整形数据！");
    return 1;
  }
  // 初始化ROS2客户端
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProgressActionClient>();
  node->send_goal(atoi(argv[1]));
  // 调用spin函数,并传入节点对象指针
  rclcpp::spin(node);
  // 释放资源
  rclcpp::shutdown();
  return 0;
}