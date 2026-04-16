#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class CubeCollisionObjectNode : public rclcpp::Node
{
public:
	CubeCollisionObjectNode() : Node("cube_collision_object_node")
	{
		object_id_ = this->declare_parameter<std::string>("object_id", "cube_target");
		pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/cube_pose");
		cube_size_ = this->declare_parameter<std::vector<double>>("cube_size", {0.05, 0.05, 0.05});
		frame_id_fallback_ = this->declare_parameter<std::string>("frame_id_fallback", "world");

		if (cube_size_.size() != 3) {
			RCLCPP_WARN(this->get_logger(), "cube_size must have 3 values. Using default 0.05,0.05,0.05.");
			cube_size_ = {0.05, 0.05, 0.05};
		}

		apply_scene_client_ =
			this->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

		pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			pose_topic_, rclcpp::QoS(10),
			std::bind(&CubeCollisionObjectNode::poseCallback, this, _1));

		RCLCPP_INFO(this->get_logger(), "Listening on %s and updating CollisionObject %s",
								pose_topic_.c_str(), object_id_.c_str());
	}

private:
	void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		if (!apply_scene_client_->wait_for_service(200ms)) {
			RCLCPP_WARN_THROTTLE(
				this->get_logger(), *this->get_clock(), 3000,
				"Waiting for /apply_planning_scene service...");
			return;
		}

		moveit_msgs::msg::CollisionObject collision_object;
		collision_object.id = object_id_;
		collision_object.header.frame_id =
			msg->header.frame_id.empty() ? frame_id_fallback_ : msg->header.frame_id;

		shape_msgs::msg::SolidPrimitive primitive;
		primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
		primitive.dimensions = {cube_size_[0], cube_size_[1], cube_size_[2]};

		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(msg->pose);
		collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

		moveit_msgs::msg::PlanningScene planning_scene;
		planning_scene.is_diff = true;
		planning_scene.world.collision_objects.push_back(collision_object);

		auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
		req->scene = planning_scene;

		auto future = apply_scene_client_->async_send_request(
			req,
			[this](rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedFuture response) {
				if (!response.get()->success) {
					RCLCPP_WARN(this->get_logger(), "ApplyPlanningScene returned success=false");
				}
			});

		(void)future;
	}

std::string object_id_;
std::string pose_topic_;
std::string frame_id_fallback_;
std::vector<double> cube_size_;

rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client_;
};

int main(int argc, char ** argv)
{
rclcpp::init(argc, argv);
auto node = std::make_shared<CubeCollisionObjectNode>();
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}