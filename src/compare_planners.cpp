#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

/** Set default location to collision objects in the plannings scene
*/
void populateScene(const std::string &frame_id){
    // Create conveyor belt
    auto const conveyor_object = [&frame_id]{
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "conveyor";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.2;
        primitive.dimensions[primitive.BOX_Y] = 2;
        primitive.dimensions[primitive.BOX_Z] = 0.4;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
        box_pose.position.x = 0.3;
        box_pose.position.y = 0;
        box_pose.position.z = 0.2;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;
        
        return collision_object;
    }();

    // Create machine
    auto const machine_object = [&frame_id]{
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "machine";

        // Define the size primitives
        const int sideNum = 5;
        double dimX[sideNum] = {0.2 ,  0.2 ,  0.02,  0.02,  0.2 };
        double dimY[sideNum] = {0.2 ,  0.2 ,  0.2 ,  0.2 ,  0.02};
        double dimZ[sideNum] = {0.02,  0.02,  0.2 ,  0.2 ,  0.2 };
        double posX[sideNum] = {   0,     0,  0.09, -0.09,  0   };
        double posY[sideNum] = {-0.5, -0.5 , -0.5 , -0.5 , -0.59};
        double posZ[sideNum] = {0.71,  0.89,  0.8 ,  0.8 ,  0.8 };
        
        for(int i = 0; i < sideNum; i++){
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = dimX[i];
            primitive.dimensions[primitive.BOX_Y] = dimY[i];
            primitive.dimensions[primitive.BOX_Z] = dimZ[i];

            // Define the pose of the box (relative to the frame_id)
            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
            box_pose.position.x = posX[i];
            box_pose.position.y = posY[i];
            box_pose.position.z = posZ[i];

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
        }
        collision_object.operation = collision_object.ADD;
        
        return collision_object;
    }();

    // Create workpiece
    auto const workpiece_object = [&frame_id]{
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "workpiece";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.1;
        primitive.dimensions[primitive.BOX_Y] = 0.1;
        primitive.dimensions[primitive.BOX_Z] = 0.1;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
        box_pose.position.x = 0;
        box_pose.position.y = -0.5;
        box_pose.position.z = 0.77;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;
        
        return collision_object;
    }();

    // Add the collision objects to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(conveyor_object);
    planning_scene_interface.applyCollisionObject(machine_object);
    planning_scene_interface.applyCollisionObject(workpiece_object);
}


int main(int argc, char ** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    // Create logger
    auto const logger = rclcpp::get_logger("HF8");
	    RCLCPP_INFO(logger, "Start planner comparison");
	    
    // Init node and create Move Group Interface
    auto const node = std::make_shared<rclcpp::Node>(
              "planner_comparer",
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
          );
    auto group_interface =  moveit::planning_interface::MoveGroupInterface(node, "manipulator");
    
    // Show the current plannning frame
    auto frame_id = group_interface.getPlanningFrame();
    RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());
          
    // Populate planning scene
    populateScene(frame_id);
          
    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
