/*
  需求：编写动作服务端实现，可以提取客户端请求提交的整型数据，并累加从1到该数据之间的所有整数以求和，
       每累加一次都计算当前运算进度并连续反馈回客户端，最后，在将求和结果返回给客户端。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；(2.3.4由回调函数实现)
      3-1.创建动作服务端对象
      3-2.处理请求数据（提交的目标值）
      3-3-1.生成连续反馈 & 3-3-2.响应最终结果
      3-4.处理取消任务请求（客户端提交的取消任务的请求）
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
*/

#include "base_interfaces_demo/action/progress.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"  // 动作客户端与服务端依赖于此头文件

// 自定义节点类
class ProgressActionServer : public rclcpp::Node {
 public:
  ProgressActionServer() : Node("progress_action_server_node_cpp") {
    RCLCPP_INFO(this->get_logger(), "action服务端节点创建");
    // 3-1.创建动作服务端对象
    /*
    rclcpp_action::Server<ActionT>::SharedPtr (返回值)
    create_server<ActionT, NodeT>(
    NodeT node,
    const std::string &name,
    处理提交的目标值 ：rclcpp_action::Server<ActionT>::GoalCallback
    handle_goal,
    处理取消任务请求（客户端提交的取消任务的请求）：rclcpp_action::Server<ActionT>::CancelCallback
    handle_cancel,
    生成连续反馈&响应最终结果 ：rclcpp_action::Server<ActionT>::AcceptedCallback
    handle_accepted)
     */
    server_ =
        rclcpp_action::create_server<base_interfaces_demo::action::Progress>(
            this, "get_sum",
            std::bind(&ProgressActionServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&ProgressActionServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&ProgressActionServer::handle_accepted, this,
                      std::placeholders::_1));
  }
  /*
  /// Signature of a callback that accepts or rejects goal requests.
  using GoalCallback = std::function<GoalResponse(const GoalUUID &,
  std::shared_ptr<const typename ActionT::Goal>)>;
  函数返回类型：rclcpp_action::GoalResponse
  参数1：rclcpp_action::GoalUUID
  参数2：std::shared_ptr<const typename ActionT::Goal>
  */
  // 3-2.处理提交的目标值
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const base_interfaces_demo::action::Progress::Goal>
          goal) {
    (void)uuid;  //表示uuid参数未使用，uuid是客户端向服务端提交请求时的身份标识
    //业务逻辑：判断提交的数字是否大于1，是就接受，否则就拒绝
    if (goal->num <= 1) {
      RCLCPP_INFO(this->get_logger(), "提交的目标值必须大于1");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "提交的目标值合法");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /*
  /// Signature of a callback that accepts or rejects requests to cancel a
  goal. using CancelCallback =
      std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
  函数返回类型：rclcpp_action::CancelResponse
  参数1：std::shared_ptr<ServerGoalHandle<ActionT>>
  */
  // 3-4.处理取消任务请求（客户端提交的取消任务的请求）
  rclcpp_action::CancelResponse handle_cancel(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<
          base_interfaces_demo::action::Progress>>
          goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "收到客户端提交的取消任务的请求");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /*
  /// Signature of a callback that is used to notify when the goal has been
  /// accepted.
  using AcceptedCallback =
      std::function<void(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
  函数返回类型：void
  参数1：std::shared_ptr<ServerGoalHandle<ActionT>>
  */
  // 3-3-1.生成连续反馈 && 3-3-2.响应最终结果
  // 该部分为主逻辑，耗时操作，建议在新线程中执行
  // detach()表示使用匿名线程
  // std::thread()的()中表示执行体？
  void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<
                           base_interfaces_demo::action::Progress>>
                           goal_handle) {
    // 新建子线程处理耗时的主逻辑
    std::thread(std::bind(&ProgressActionServer::execute, this, goal_handle))
        .detach();
  }

  void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<
                   base_interfaces_demo::action::Progress>>
                   goal_handle) {
    // 1.生成连续反馈返回给客户端
    // goal_handle->publish_feedback();
    // void
    // publish_feedback(std::shared_ptr<base_interfaces_demo::action::Progress_Feedback>
    // feedback_msg)
    // Progress_Feedback相当于Progress::Feedback
    // 首先获取目标值，然后遍历，遍历中累加，且每循环一次就计算进度，并作为连续反馈发布
    int num = goal_handle->get_goal()->num;
    int sum = 0;
    auto feedback =
        std::make_shared<base_interfaces_demo::action::Progress_Feedback>();
    auto result =
        std::make_shared<base_interfaces_demo::action::Progress_Result>();
    // 设置休眠
    rclcpp::Rate rate(1.0);
    for (int i = 1; i <= num; i++) {
      sum += i;
      double progress = i / (float)num;  //计算进度
      feedback->progress = progress;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "反馈中: [%.2f]", progress);

      // 判断是否有取消任务的请求
      // 如果有取消任务的请求，就终止程序 return
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "收到客户端提交的取消任务的请求");
        // goal_handle->canceled()
        // void
        // canceled(std::shared_ptr<base_interfaces_demo::action::Progress_Result>
        // result_msg)
        result->sum = sum;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "终止任务");
        return;
      }

      rate.sleep();
    }
    // 2.生成最终结果返回给客户端
    // goal_handle->succeed();
    // void
    // succeed(std::shared_ptr<base_interfaces_demo::action::Progress_Result>
    // result_msg)
    if (rclcpp::ok()) {
      result->sum = sum;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "完成任务");
    }
  }

 private:
  rclcpp_action::Server<base_interfaces_demo::action::Progress>::SharedPtr
      server_;  //动作服务端对象
};

int main(int argc, char *argv[]) {
  // 初始化ROS2客户端
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProgressActionServer>();
  // 调用spin函数,并传入节点对象指针
  rclcpp::spin(node);
  // 释放资源
  rclcpp::shutdown();
  return 0;
}