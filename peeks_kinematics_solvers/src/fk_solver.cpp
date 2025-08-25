#include <format>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
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
#include <unordered_map>
#include <vector>

namespace peeks_kinematics_solvers {

/**
 * @brief 正向运动学求解器节点。
 *
 */
class FkSolverNode : public rclcpp::Node {
public:
    /**
     * @brief 构造新的正向运动学求解器节点的对象。
     *
     */
    FkSolverNode() : Node("fk_solver") {
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
        fkInputSubscriber_ = create_subscription<sensor_msgs::msg::JointState>(
            "fk_input", qos, [this](const sensor_msgs::msg::JointState::SharedPtr msg) { onReceiveFkInput(msg); });
        fkOutputPublisher_ = create_publisher<geometry_msgs::msg::TransformStamped>("fk_output", qos);

        RCLCPP_INFO(
            get_logger(),
            std::format("FK solver node initialized. chain_root='{}', chain_tip='{}'.", chainRoot_, chainTip_).c_str());
    }

private:
    /**
     * @brief 从当前KDL树与链的根部/末端构建运动学链与FK求解器（需要持有锁）。
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
        kdlJointNames_ = std::move(jointNames);
        jointIndexMap_.reset();
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
            kdlJointNames_.clear();
            jointIndexMap_.reset();
            RCLCPP_ERROR(get_logger(),
                         std::format("KDL chain build failed after robot description updated: {}", reason).c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), std::format("KDL chain prepared for FK. joints={}.", kdlJointNames_.size()).c_str());
    }

    /**
     * @brief 当接收到正向运动学输入时触发，根据关节角计算末端执行器位姿。
     *
     * @param msg 接收到的消息，为输入的各关节角度值。
     */
    void onReceiveFkInput(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg == nullptr) {
            return;
        }

        // 锁住模型数据
        std::lock_guard<std::mutex> lock{modelMutex_};

        if (kdlChain_ == nullptr || fkSolver_ == nullptr) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "KDL chain not ready yet. Waiting for /robot_description...");
            return;
        }

        if (msg->position.empty()) {
            RCLCPP_WARN(get_logger(), "Received JointState without positions.");
            return;
        }

        // 构建名称到索引的映射（仅在首次或尺寸变化时构建）
        if (!jointIndexMap_.has_value() || jointIndexMap_->size() != kdlJointNames_.size()) {
            std::unordered_map<std::string, std::size_t> nameToIndex{};
            nameToIndex.reserve(msg->name.size());
            for (std::size_t i{}; i < msg->name.size(); ++i) {
                nameToIndex[msg->name[i]] = i;
            }
            std::vector<int> jointIndexMap{};
            jointIndexMap.resize(kdlJointNames_.size(), -1);
            for (std::size_t i{}; i < kdlJointNames_.size(); ++i) {
                auto it = nameToIndex.find(kdlJointNames_[i]);
                if (it == nameToIndex.end()) {
                    RCLCPP_ERROR(
                        get_logger(),
                        std::format("Joint '{}' not found in incoming JointState.", kdlJointNames_[i]).c_str());
                    return;
                }
                jointIndexMap[i] = static_cast<int>(it->second);
            }
            jointIndexMap_ = std::move(jointIndexMap);
            RCLCPP_INFO(get_logger(), "Joint index map constructed for FK input.");
        }

        if (msg->position.size() < msg->name.size()) {
            RCLCPP_WARN(get_logger(), "JointState positions shorter than names.");
        }

        // 组装KDL关节数组（按KDL运动学链的顺序）
        KDL::JntArray q{kdlChain_->getNrOfJoints()};
        for (std::size_t i{}; i < kdlChain_->getNrOfJoints(); ++i) {
            int src = jointIndexMap_.value()[i];
            if (src < 0 || static_cast<std::size_t>(src) >= msg->position.size()) {
                RCLCPP_ERROR(get_logger(), std::format("Invalid joint index mapping for joint #{}.", i).c_str());
                return;
            }
            q(i) = msg->position[src];
        }

        // 计算FK
        KDL::Frame eePose{};
        int ret{fkSolver_->JntToCart(q, eePose)};
        if (ret < 0) {
            RCLCPP_ERROR(get_logger(), std::format("FK solver failed with code {}.", ret).c_str());
            return;
        }

        // 转换到TransformStamped并发布
        geometry_msgs::msg::TransformStamped tf{};
        tf.header.stamp = get_clock()->now();
        tf.header.frame_id = chainRoot_;
        tf.child_frame_id = chainTip_;
        tf.transform.translation.x = eePose.p.x();
        tf.transform.translation.y = eePose.p.y();
        tf.transform.translation.z = eePose.p.z();
        double qx{}, qy{}, qz{}, qw{};
        eePose.M.GetQuaternion(qx, qy, qz, qw);
        tf.transform.rotation.x = qx;
        tf.transform.rotation.y = qy;
        tf.transform.rotation.z = qz;
        tf.transform.rotation.w = qw;

        fkOutputPublisher_->publish(tf);
    }

private:
    // 参数
    std::string chainRoot_;
    std::string chainTip_;

    // 话题
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robotDescriptionSubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr fkInputSubscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr fkOutputPublisher_;

    // KDL结构
    std::shared_ptr<KDL::Tree> kdlTree_;
    std::shared_ptr<KDL::Chain> kdlChain_;
    std::vector<std::string> kdlJointNames_;
    std::optional<std::vector<int>> jointIndexMap_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fkSolver_;

    // 互斥锁与参数回调句柄
    std::mutex modelMutex_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr paramCallbackHandle_;
};

}  // namespace peeks_kinematics_solvers

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node{std::make_shared<peeks_kinematics_solvers::FkSolverNode>()};

    rclcpp::executors::MultiThreadedExecutor executor{};
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}