#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

using namespace std::chrono_literals;

class ObstacleCollisionObjectNode : public rclcpp::Node
{
public:
	ObstacleCollisionObjectNode()
	: Node("cube_collision_object_node")
	{
		planning_scene_pub_ =
			this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

		timer_ = this->create_wall_timer(1000ms, [this]() {
			publish_obstacle_once();
		});
	}

private:
	void publish_obstacle_once()
	{
		if (published_) {
			return;
		}

		moveit_msgs::msg::CollisionObject obstacle;
		obstacle.header.frame_id = "fr3_link0";
		obstacle.id = "world_obstacle";

		shape_msgs::msg::SolidPrimitive primitive;
		primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
		primitive.dimensions = {0.35, 0.05, 0.35};

		geometry_msgs::msg::Pose obstacle_pose;
		obstacle_pose.position.x = 0.45;
		obstacle_pose.position.y = 0.0;
		obstacle_pose.position.z = 0.17;
		obstacle_pose.orientation.w = 1.0;

		obstacle.primitives.push_back(primitive);
		obstacle.primitive_poses.push_back(obstacle_pose);
		obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;

		moveit_msgs::msg::PlanningScene planning_scene_msg;
		planning_scene_msg.is_diff = true;
		planning_scene_msg.world.collision_objects.push_back(obstacle);

		planning_scene_pub_->publish(planning_scene_msg);
		published_ = true;
		timer_->cancel();

		RCLCPP_INFO(this->get_logger(), "Published MoveIt collision object: world_obstacle");
	}

	rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	bool published_ = false;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ObstacleCollisionObjectNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
