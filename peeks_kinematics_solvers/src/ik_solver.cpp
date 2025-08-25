#include <format>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

namespace peeks_kinematics_solvers {

/**
 * @brief 逆向运动学求解器节点。
 *
 */
class IkSolverNode : public rclcpp::Node {
public:
    /**
     * @brief 构造新的逆向运动学求解器节点的对象。
     *
     */
    IkSolverNode() : Node("ik_solver") {
        // 声明参数
        chainRoot_ = declare_parameter<std::string>("chain_root", "base_link");
        chainTip_ = declare_parameter<std::string>("chain_tip", "tool_link");
        paramCallbackHandle_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) -> rcl_interfaces::msg::SetParametersResult {
                return onSetParameters(params);
            });

        // 设置QoS
        rclcpp::QoS qos{rclcpp::KeepAll{}};
        qos.reliable();

        // 创建话题
        robotDescriptionSubscriber_ = create_subscription<std_msgs::msg::String>(
            "robot_description", qos,
            [this](const std_msgs::msg::String::SharedPtr msg) { onReceiveRobotDescription(msg); });
        ikInputSubscriber_ = create_subscription<geometry_msgs::msg::TransformStamped>(
            "ik_input", qos,
            [this](const geometry_msgs::msg::TransformStamped::SharedPtr msg) { onReceiveIkInput(msg); });
        ikOutputPublisher_ = create_publisher<sensor_msgs::msg::JointState>("ik_output", qos);

        RCLCPP_INFO(
            get_logger(),
            std::format("IK solver node initialized. chain_root='{}', chain_tip='{}'.", chainRoot_, chainTip_).c_str());
    }

private:
    /**
     * @brief 从当前KDL树与链的根部/末端构建运动学链与FK/IK求解器（需要持有锁）。
     *
     * @param errorReason 错误原因。
     * @return true 构建成功。
     * @return false 构建失败。
     */
    bool rebuildKdlChainLocked(std::string& errorReason) {
        if (kdlTree_ == nullptr) {
            errorReason = "KDL tree is not available yet.";
            return false;
        }

        auto newKdlChain{std::make_shared<KDL::Chain>()};
        if (!kdlTree_->getChain(chainRoot_, chainTip_, *newKdlChain)) {
            errorReason = "Failed to extract KDL::Chain with current root/tip.";
            return false;
        }

        // 提取关节名称
        std::vector<std::string> jointNames{};
        jointNames.reserve(newKdlChain->getNrOfJoints());
        for (std::size_t i{}; i < newKdlChain->getNrOfSegments(); ++i) {
            const KDL::Segment& segment = newKdlChain->getSegment(i);
            const KDL::Joint& joint = segment.getJoint();
            if (joint.getType() != KDL::Joint::None) {
                jointNames.emplace_back(joint.getName());
            }
        }

        // 更新数据成员
        kdlChain_ = std::move(newKdlChain);
        fkSolver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(*kdlChain_);
        ikSolverVel_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(*kdlChain_);
        ikSolverPos_ = std::make_shared<KDL::ChainIkSolverPos_NR>(*kdlChain_, *fkSolver_, *ikSolverVel_, 200, 1e-6);
        kdlJointNames_ = std::move(jointNames);
        qLast_.reset();
        return true;
    }

    /**
     * @brief 当设置参数时触发，动态更新chainRoot_和chainTip_，必要时重建运动学链。
     *
     * @param params 所有参数列表。
     * @return rcl_interfaces::msg::SetParametersResult
     */
    rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result{};
        result.successful = true;

        std::string newChainRoot{chainRoot_};
        std::string newChainTip{chainTip_};

        for (const auto& param : params) {
            if (param.get_name() == "chain_root") {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING || param.as_string().empty()) {
                    result.successful = false;
                    result.reason = "chain_root must be a non-empty string.";
                    return result;
                }
                newChainRoot = param.as_string();
            } else if (param.get_name() == "chain_tip") {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING || param.as_string().empty()) {
                    result.successful = false;
                    result.reason = "chain_tip must be a non-empty string.";
                    return result;
                }
                newChainTip = param.as_string();
            }
        }

        // 如果没有变化，直接成功，不需要重建运动学链
        if (newChainRoot == chainRoot_ && newChainTip == chainTip_) {
            return result;
        }

        // 验证并应用（在锁内）
        std::lock_guard<std::mutex> lock{modelMutex_};

        // 先临时保存旧值
        const std::string oldChainRoot{chainRoot_};
        const std::string oldChainTip{chainTip_};

        chainRoot_ = newChainRoot;
        chainTip_ = newChainTip;

        if (kdlTree_ == nullptr) {
            // 没有KDL树，先接受参数，等URDF来时再构建
            RCLCPP_INFO(get_logger(), "Parameters updated. Waiting for /robot_description to build KDL chain.");
            return result;
        }

        std::string reason{};
        if (!rebuildKdlChainLocked(reason)) {
            // 回滚
            chainRoot_ = oldChainRoot;
            chainTip_ = oldChainTip;
            result.successful = false;
            result.reason = reason;
            return result;
        }

        RCLCPP_INFO(get_logger(),
                    std::format("Rebuilt KDL chain due to parameter update. chain_root='{}', chain_tip='{}'.",
                                chainRoot_, chainTip_)
                        .c_str());
        return result;
    }

    /**
     * @brief 将ROS2的Transform转换为KDL::Frame。
     *
     * @param t 输入的ROS2位姿。
     * @return KDL::Frame 输出的KDL位姿。
     */
    static KDL::Frame toKdlFrame(const geometry_msgs::msg::Transform& t) {
        KDL::Vector p{t.translation.x, t.translation.y, t.translation.z};
        KDL::Rotation r{KDL::Rotation::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)};
        return KDL::Frame{r, p};
    }

    /**
     * @brief 当接收到机器人描述时触发，解析URDF为KDL树并提取运动学链。
     *
     * @param msg 接收到的消息，为URDF的文本内容。
     */
    void onReceiveRobotDescription(const std_msgs::msg::String::SharedPtr msg) {
        if (msg == nullptr || msg->data.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty /robot_description message, skip.");
            return;
        }

        auto newKdlTree{std::make_shared<KDL::Tree>()};
        if (!kdl_parser::treeFromString(msg->data, *newKdlTree)) {
            RCLCPP_ERROR(get_logger(), "Failed to parse URDF from /robot_description to KDL::Tree.");
            return;
        }

        std::lock_guard<std::mutex> lock{modelMutex_};

        // 更新KDL树
        kdlTree_ = std::move(newKdlTree);
        std::string reason{};
        if (!rebuildKdlChainLocked(reason)) {
            // 若运动学链构建失败，清空链，等待参数或下一次机器人描述
            kdlChain_.reset();
            fkSolver_.reset();
            ikSolverVel_.reset();
            ikSolverPos_.reset();
            kdlJointNames_.clear();
            qLast_.reset();
            RCLCPP_ERROR(get_logger(),
                         std::format("KDL chain build failed after robot description updated: {}", reason).c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), std::format("KDL chain prepared for IK. joints={}.", kdlJointNames_.size()).c_str());
    }

    /**
     * @brief 当接收到逆向运动学输入时触发，根据末端执行器位姿计算关节角。
     *
     * @param msg 接收到的消息，为输入的末端执行器位姿。
     */
    void onReceiveIkInput(const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
        if (msg == nullptr) {
            return;
        }

        // 锁住模型数据
        std::lock_guard<std::mutex> lock{modelMutex_};
        if (kdlChain_ == nullptr || fkSolver_ == nullptr || ikSolverPos_ == nullptr) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "KDL chain not ready yet. Waiting for /robot_description...");
            return;
        }

        // 校验frame_id与期望的root是否一致
        if (!msg->header.frame_id.empty() && msg->header.frame_id != chainRoot_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                std::format("ik_input frame_id '{}' != chain_root '{}'.", msg->header.frame_id, chainRoot_).c_str());
        }

        KDL::Frame target{toKdlFrame(msg->transform)};

        // 初始猜测，使用上次结果或零
        KDL::JntArray qInit{kdlChain_->getNrOfJoints()};
        if (qLast_.has_value() && qLast_->rows() == qInit.rows()) {
            for (std::size_t i{}; i < qInit.rows(); ++i) {
                qInit(i) = (*qLast_)(i);
            }
        } else {
            for (std::size_t i{}; i < qInit.rows(); ++i) {
                qInit(i) = 0.0;
            }
        }

        // 计算IK
        KDL::JntArray qSol{kdlChain_->getNrOfJoints()};
        int ret{ikSolverPos_->CartToJnt(qInit, target, qSol)};
        if (ret < 0) {
            RCLCPP_ERROR(get_logger(), std::format("IK solver failed with code {}.", ret).c_str());
            return;
        }

        qLast_ = qSol;

        // 发布JointState，名称顺序与KDL运动学链一致
        sensor_msgs::msg::JointState out{};
        out.header.stamp = get_clock()->now();
        out.name = kdlJointNames_;
        out.position.resize(kdlJointNames_.size());
        for (std::size_t i{}; i < kdlChain_->getNrOfJoints(); ++i) {
            out.position[i] = qSol(i);
        }

        ikOutputPublisher_->publish(out);
    }

private:
    // 参数
    std::string chainRoot_;
    std::string chainTip_;

    // 话题
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robotDescriptionSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr ikInputSubscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ikOutputPublisher_;

    // KDL结构
    std::shared_ptr<KDL::Tree> kdlTree_;
    std::shared_ptr<KDL::Chain> kdlChain_;
    std::vector<std::string> kdlJointNames_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fkSolver_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> ikSolverVel_;
    std::shared_ptr<KDL::ChainIkSolverPos_NR> ikSolverPos_;
    std::optional<KDL::JntArray> qLast_;

    // 互斥锁与参数回调句柄
    std::mutex modelMutex_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr paramCallbackHandle_;
};

}  // namespace peeks_kinematics_solvers

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node{std::make_shared<peeks_kinematics_solvers::IkSolverNode>()};

    rclcpp::executors::MultiThreadedExecutor executor{};
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}