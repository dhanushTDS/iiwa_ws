#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <pluginlib/class_loader.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <boost/scoped_ptr.hpp>
#include <iiwa.hpp>
#include <iiwa_cam/PickPose.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>



enum orientation
{
    bottom,
    front,
    back,
    left,
    right
};

/**
 * @brief Convert degree to radiant
 *
 * @param degree 180 degree to -180 degree
 * @return double
 */


inline double degree_to_rad(double degree)
{
    return (degree / 180.0) * M_PI;
}

// vectors used to rotate panels [w, x, y, z]
const std::vector<double> align_x_axis_vector{0.7071068, 0.7071068, 0, 0};  // 90 degree x
const std::vector<double> align_y_axis_vector{0.7071068, 0, -0.7071068, 0}; // -90 degree y

// the translation and rotation calculated from Horn's method
const std::vector<double> camera_to_robot_translation{0, 0, 0};
// [w x y z]
const std::vector<double> camera_to_robot_rotation{1, 0, 0, 0};

const std::vector<double> idea_container_size{0.5, 0.4, 0.254, 0.0127};

std::vector<double> iiwa_pink_origin_joint{degree_to_rad(-85.72), degree_to_rad(21.7), degree_to_rad(1.59), degree_to_rad(-71.34), degree_to_rad(-93.38), degree_to_rad(50.97), degree_to_rad(-85.76)};
// std::vector<double> iiwa_blue_pre_pick_joint{degree_to_rad(-122.15), degree_to_rad(-48.86), degree_to_rad(126.21), degree_to_rad(-77.66), degree_to_rad(-65.49), degree_to_rad(49.67), degree_to_rad(-67.5)};
// std::vector<double> iiwa_blue_ready_pick_joint{degree_to_rad(-148.10), degree_to_rad(-58.63), degree_to_rad(126.21), degree_to_rad(-12.9), degree_to_rad(47.9), degree_to_rad(115.73), degree_to_rad(-24.54)};
// std::vector<double> iiwa_blue_pick_joint{degree_to_rad(-137.25), degree_to_rad(-55.8), degree_to_rad(126.22), degree_to_rad(-38.81), degree_to_rad(41.34), degree_to_rad(99.91), degree_to_rad(-32.94)};
// std::vector<double> iiwa_green_origin_joint{degree_to_rad(41.29), degree_to_rad(22.32), degree_to_rad(-60.56), degree_to_rad(-69.65), degree_to_rad(19.45), degree_to_rad(98.38), degree_to_rad(-14.3)};

static std::unordered_map<std::string, cam::Kuka *> robots;

const std::string iiwa_pink_group = "iiwa_pink_arm";


const std::string workspace_origin_link = "iiwa_pink_link_0";

const int MAX_PLAN_TIMES = 10;
const float SAFE_SPEED = 0.05;
const std::vector<float> damp(3, 0.66);
const std::vector<float> stiff(3, 2250);

moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr = nullptr;
moveit::planning_interface::MoveGroupInterface *iiwa_pink_move_group_ptr = nullptr;
moveit_visual_tools::MoveItVisualTools *visual_tools_ptr = nullptr;

/**
 * @brief Convert vector of quaternion to geometry::Quaternion
 *
 * @param quat_vector [w, x, y, z]
 * @return the quaternion message
 */
geometry_msgs::Quaternion vector_to_quat(const std::vector<double> &quat_vector)
{
    geometry_msgs::Quaternion res;

    res.w = quat_vector[3];
    res.x = quat_vector[0];
    res.y = quat_vector[1];
    res.z = quat_vector[2];

    return res;
}

/**
 * @brief Convert vector of point position to geometry::Point
 *
 * @param point_vector [x, y, z]
 * @return geometry_msgs::Point
 */
geometry_msgs::Point vector_to_point(const std::vector<double> &point_vector)
{
    geometry_msgs::Point res;

    res.x = point_vector[0];
    res.y = point_vector[1];
    res.z = point_vector[2];

    return res;
}

/**
 * @brief Multiply quaternion to get rotation result MOD
 *
 * @param a quaternion a
 * @param b quaternion b
 * @return the result quaternion
 */

geometry_msgs::Quaternion quat_mult_quat(const geometry_msgs::Quaternion& a, const geometry_msgs::Quaternion& b)
{
    geometry_msgs::Quaternion res;

    // Check if either quaternion is uninitialized or invalid
    if (std::isnan(a.w) || std::isnan(a.x) || std::isnan(a.y) || std::isnan(a.z) ||
        std::isnan(b.w) || std::isnan(b.x) || std::isnan(b.y) || std::isnan(b.z))
    {
        // Handle the error here (e.g., throw an exception, log an error message)
        // Return a default or meaningful value depending on your requirements
        return res; // Assuming res is initialized to zeros
    }

    // Perform quaternion multiplication
    res.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    res.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    res.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    res.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

    return res;
}


/**
 * @brief Add points to get translation result MOD
 *
 * @param a point a
 * @param b point b
 * @return the result point
 */
geometry_msgs::Point point_add_point(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
    geometry_msgs::Point res;

    // Perform input validation (optional)
    if (std::isnan(a.x) || std::isnan(a.y) || std::isnan(a.z) ||
        std::isnan(b.x) || std::isnan(b.y) || std::isnan(b.z))
    {
        // Handle the error here (e.g., throw an exception, log an error message)
        // Return a default or meaningful value depending on your requirements
        return res; // Assuming res is initialized to zeros
    }

    // Perform point addition
    res.x = a.x + b.x;
    res.y = a.y + b.y;
    res.z = a.z + b.z;

    return res;
}


/**
 * @brief Point multiply quaternion get rotation result
 *
 * @param p point p
 * @param quat quaternion quat
 * @return the result point
 */
geometry_msgs::Point point_mult_quat(const geometry_msgs::Point &p, const geometry_msgs::Quaternion &quat)
{
    geometry_msgs::Point res;

    res.x = (1 - 2 * quat.y * quat.y - 2 * quat.z * quat.z) * p.x + (2 * quat.x * quat.y - 2 * quat.z * quat.w) * p.y + (2 * quat.x * quat.z + 2 * quat.y * quat.w) * p.z;
    res.y = (2 * quat.x * quat.y + 2 * quat.z * quat.w) * p.x + (1 - 2 * quat.x * quat.x - 2 * quat.z * quat.z) * p.y + (2 * quat.y * quat.z - 2 * quat.x * quat.w) * p.z;
    res.z = (2 * quat.x * quat.z - 2 * quat.y * quat.w) * p.x + (2 * quat.y * quat.z + 2 * quat.x * quat.w) * p.y + (1 - 2 * quat.x * quat.x - 2 * quat.y * quat.y) * p.z;

    return res;
}

/**
 * @brief Point a cross product Point b
 *
 * @param a
 * @param b
 * @return geometry_msgs::Point cross product
 */
geometry_msgs::Point point_cross_product(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
    geometry_msgs::Point res;

    res.x = a.y * b.z - a.z * b.y;
    res.y = -(a.x * b.z - a.z * b.x);
    res.z = a.x * b.y - a.y * b.x;

    return res;
}

/**
 * @brief Give the quaternion according to the rotation matrix
 *
 * @param matrix [[x_x, x_y, x_z], [y_x, y_y, y_z], [z_x, z_y, z_z]]
 * @return geometry_msgs::Quaternion
 */
geometry_msgs::Quaternion rotation_matrix_to_quaternion(const std::vector<geometry_msgs::Point> &matrix)
{
    geometry_msgs::Quaternion res;

    double trace = matrix[0].x + matrix[1].y + matrix[2].z;
    if (trace > 1.0)
    {
        double s = 0.5 / sqrt(trace + 1.0);
        res.w = 0.25 / s;
        res.x = (matrix[1].z - matrix[2].y) * s;
        res.y = (matrix[2].x - matrix[0].z) * s;
        res.z = (matrix[0].y - matrix[1].x) * s;
    }
    else
    {
        if (matrix[0].x > matrix[1].y && matrix[0].x > matrix[2].z)
        {
            double s = 2.0 * sqrt(1 + matrix[0].x - matrix[1].y + matrix[2].z);
            res.w = (matrix[1].z - matrix[2].y) / s;
            res.x = 0.25 * s;
            res.y = (matrix[0].y + matrix[1].x) / s;
            res.z = (matrix[2].x + matrix[0].z) / s;
        }
        else if (matrix[1].y > matrix[2].z)
        {
            double s = 2.0 * sqrt(1 - matrix[0].x + matrix[1].y - matrix[2].z);
            res.w = (matrix[2].x - matrix[0].z) / s;
            res.x = (matrix[0].y + matrix[1].x) / s;
            res.y = 0.25 * s;
            res.z = (matrix[1].z + matrix[2].y) / s;
        }
        else
        {
            double s = 2.0 * sqrt(1 - matrix[0].x - matrix[1].y + matrix[2].z);
            res.w = (matrix[0].y - matrix[1].x) / s;
            res.x = (matrix[2].x + matrix[0].z) / s;
            res.y = (matrix[1].z + matrix[2].y) / s;
            res.z = 0.25 * s;
        }
    }

    return res;
}

geometry_msgs::Quaternion normalize_quaternion(const geometry_msgs::Quaternion &quat)
{
    geometry_msgs::Quaternion res;

    double sqrt_sum = sqrt(pow(quat.w, 2) + pow(quat.x, 2) + pow(quat.y, 2) + pow(quat.z, 2));
    res.w = quat.w / sqrt_sum;
    res.x = quat.x / sqrt_sum;
    res.y = quat.y / sqrt_sum;
    res.z = quat.z / sqrt_sum;

    return res;
}

geometry_msgs::Point normalize_point(const geometry_msgs::Point &p)
{
    geometry_msgs::Point res;

    double sqrt_sum = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
    res.x = p.x / sqrt_sum;
    res.y = p.y / sqrt_sum;
    res.z = p.z / sqrt_sum;

    return res;
}

/**
 * @brief Add floor to the move group interface
 *
 * @return the ids of the collision floor
 */
std::vector<std::string> add_floor()
{
    std::vector<std::string> floor_objects_ids;
    std::vector<moveit_msgs::CollisionObject> floor_objects;

    moveit_msgs::ObjectColor color_msg;
    color_msg.color.a = 1;
    color_msg.color.r = 0.5;
    color_msg.color.g = 0.5;
    color_msg.color.b = 0.5;

    moveit_msgs::CollisionObject floor;
    floor.id = "floor";
    floor.header.frame_id = workspace_origin_link;
    floor.operation = moveit_msgs::CollisionObject::ADD;

    floor.primitives.resize(1);
    floor.primitives[0].type = shape_msgs::SolidPrimitive::BOX;

    floor.primitives[0].dimensions.resize(3);

    floor.primitives[0].dimensions[0] = 5;
    floor.primitives[0].dimensions[1] = 5;
    floor.primitives[0].dimensions[2] = 0.01;

    floor.primitive_poses.resize(1);
    floor.primitive_poses[0].position.z = -0.01;

    floor_objects.push_back(floor);
    floor_objects_ids.push_back(floor.id);

    planning_scene_interface_ptr->applyCollisionObjects(floor_objects, std::vector<moveit_msgs::ObjectColor>{color_msg});

    return floor_objects_ids;
}

void add_joint_constraints(const std::string &robot_name, moveit_msgs::Constraints &constraints)
{
    for (int i = 1; i <= 7; i++)
    {
        moveit_msgs::JointConstraint jcm;

        jcm.joint_name = robot_name + "_joint_" + std::to_string(i);
        jcm.position = 0;
        if (i < 7)
        {
            if (i % 2 == 1)
            {
                jcm.tolerance_above = M_PI * 168 / 180.0;
                jcm.tolerance_below = M_PI * 168 / 180.0;
            }
            else
            {
                jcm.tolerance_above = M_PI * 118 / 180.0;
                jcm.tolerance_below = M_PI * 118 / 180.0;
            }
        }
        else
        {
            jcm.tolerance_above = M_PI * 173 / 180.0;
            jcm.tolerance_below = M_PI * 173 / 180.0;
        }
        jcm.weight = 0.5;

        constraints.joint_constraints.push_back(jcm);
    }
}

/**
 * @brief Add path constraints for the given move group interface
 *
 * @param move_group_interface the move group interface with constraints
 * @param robot_move_group the robot move group name
 * @param quat the constraint orientation
 */
void add_orientation_constraints(moveit::planning_interface::MoveGroupInterface &move_group_interface, const std::string &robot_move_group, const geometry_msgs::Quaternion quat = vector_to_quat(std::vector<double>{0, 0, 1, 0}))
{
    std::string robot_name = robot_move_group.substr(0, robot_move_group.size() - 4);

    moveit_msgs::Constraints constraints;

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = robot_name + "_link_7";
    ocm.header.frame_id = robot_name + "_link_0";

    ocm.orientation = quat;

    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;

    ocm.weight = 1.0;

    constraints.orientation_constraints.push_back(ocm);
    // add_joint_constraints(robot_name, constraints);

    move_group_interface.setPathConstraints(constraints);
}

/**
 * @brief Remove path constraints for the given move group interface
 *
 * @param move_group_interface the move group interface with constraints
 *
 */
void delete_constraints(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
    moveit_msgs::Constraints constraints;
    move_group_interface.setPathConstraints(constraints);
}

bool plan_to_target_pose(moveit::planning_interface::MoveGroupInterface &move_group_interface, moveit_visual_tools::MoveItVisualTools &visual_tools, const std::string &robot_move_group,
                         const std::vector<double> &position, const std::vector<double> &orientation, const float velocity = SAFE_SPEED)
{

    std::string robot_name = robot_move_group.substr(0, robot_move_group.size() - 4);

    geometry_msgs::Pose target_pose;

    target_pose.position = vector_to_point(position);
    target_pose.orientation = vector_to_quat(orientation);

    move_group_interface.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    for (int i = 0; i < MAX_PLAN_TIMES && !success; i++)
    {
        success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Plan to the target pose %s", success ? "SUCCESS" : "FAILED");
        // visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    }

    visual_tools.prompt("Press 'next' to make the ROBOT MOVE !!!");

    if (success)
    {
        robots[robot_name]->exe_joint_traj(plan.trajectory_.joint_trajectory.points, velocity, stiff, damp, cam::Kuka::JOINT_SPLINE_MODE::CARTESIAN_IMPEDANCE_MODE);
    }

    visual_tools.prompt("Press 'next' to continue planning for the next point!!!");

    return success;
}



bool status = false;

void statusCallback(const std_msgs::Bool::ConstPtr& msg)
{
    status = msg->data;
    ROS_INFO("Received status: %s", status ? "True" : "False");
}



// std::vector<double> modifyVector(std::vector<double>& inputVector, double adjustment1, double adjustment2)
// {
//     inputVector[0] += adjustment1;
//     inputVector[1] += adjustment2;
//     return inputVector;
// }

void modifyVector(std::vector<double>& vec, const std::string& str, double number) {
    if (str == "x") {
        vec[0] += number / 1000;
    } else if (str == "y") {
        vec[1] += number / 1000;
    }
}



//Execution 

int main(int argc, char **argv)
{
    float a = .0001;
    ros::init(argc, argv, "rviz_pink_iiwa_node");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    for (int i = 1; i < argc; i++)
        robots.insert(std::pair<std::string, cam::Kuka *>(argv[i], new cam::Kuka(argv[i])));


    // robots["iiwa_pink"]->move_joint_ptp(iiwa_pink_origin_joint);
    planning_scene_interface_ptr = new moveit::planning_interface::PlanningSceneInterface();
    iiwa_pink_move_group_ptr = new moveit::planning_interface::MoveGroupInterface(iiwa_pink_group);

    iiwa_pink_move_group_ptr->setStartStateToCurrentState();

    visual_tools_ptr = new moveit_visual_tools::MoveItVisualTools(workspace_origin_link);
    visual_tools_ptr->enableBatchPublishing();
    visual_tools_ptr->deleteAllMarkers();
    visual_tools_ptr->trigger();

    // //Meh Points
    // std::vector<double> iiwa_pink_fake{0.66903, 0.00174, 0.6092}; // Fake position
    // std::vector<double> home{0.67138,-0.00323, 0.6092}; // Home Position 
    // std::vector<double> above{0.67138,-0.00323, 0.25622}; // Above Position 
    // std::vector<double> inserted{0.67138,-0.00323, 0.22222}; // Inserted position
    // std::vector<double> iiwa_pink_3{0.67138,-0.00323, 0.47122}; //removal 
    // std::vector<double> iiwa_pink_pose_orientation_1{0.9173874, -0.3979527, 0.0001517, 0.0058318};


    // //Points with offset in xdirection to test failure case
    // std::vector<double> iiwa_pink_fake{0.66903, 0.00174, 0.6092}; // Fake position
    // std::vector<double> iiwa_pink_0{0.67064, -0.00376, 0.2092}; // Home Position 
    // std::vector<double> iiwa_pink_1{0.67064, -0.00376, 0.15522}; // Above Position 
    // std::vector<double> iiwa_pink_2{0.67064, -0.00376, 0.12222}; // Inserted position
    // std::vector<double> iiwa_pink_3{0.67064, -0.00376, 0.17122}; //removal 

    // std::vector<double> iiwa_pink_pose_orientation_1{0.9173874, -0.3979527, 0.0001517, 0.0058318};
    // ros::Subscriber status_subscriber = node_handle.subscribe("iiwa_pink/_insertion_status", 10, statusCallback);


//     ros::ServiceClient status_client = node_handle.serviceClient<std_srvs::Trigger>("/insertion_status");
//     std_srvs::Trigger status_srv;

//     bool stop = false;

//     while (!stop)
//     {
//     // Plan above
//     plan_to_target_pose(*iiwa_pink_move_group_ptr, *visual_tools_ptr, iiwa_pink_group, above, iiwa_pink_pose_orientation_1);

//     // Plan inserted
//     plan_to_target_pose(*iiwa_pink_move_group_ptr, *visual_tools_ptr, iiwa_pink_group, inserted, iiwa_pink_pose_orientation_1);
//     // Call the insertion status service

//     if (status_client.call(status_srv))
//     {
//         if (status_srv.response.success)
//         {
//         // Terminate the loop after executing plan (Above)
//         plan_to_target_pose(*iiwa_pink_move_group_ptr, *visual_tools_ptr, iiwa_pink_group, above, iiwa_pink_pose_orientation_1);
//         break;
//         }
//         else
//         {


//         // Call the adj_node service
//         ros::ServiceClient adj_client = node_handle.serviceClient<std_srvs::Trigger>("/adj_node_service");
//         std_srvs::Trigger adj_srv;
//         if (adj_client.call(adj_srv))
//         {
//             std::string message = adj_srv.response.message;
//             if (message == "+x" || message == "-x")
//             {
//             // Call modifyVector
//             modifyVector(above,a, 0);
//             }
//             else if (message == "+y" || message == "-y")
//             {
//             // Call modifyVector
//             modifyVector(inserted, 0, a);
//             }
//         }
//         else
//         {
//             ROS_ERROR("Failed to call adj_node service");
//         }
//         }
//     }
//     else
//     {
//         ROS_ERROR("Failed to call insertion_status service");
//     }

//   ros::spinOnce();
// }


    // Hardcoded Points, obatined Manually.
    //Good Points
    std::vector<double> iiwa_pink_fake{0.66903, 0.00174, 0.6092}; // Fake position
    std::vector<double> iiwa_pink_0{0.669009459, -0.004056958, 0.3092}; // Home Position 
    std::vector<double> iiwa_pink_1{0.669009459, -0.004056958, 0.15342}; // Above Position 
    std::vector<double> iiwa_pink_2{0.669009459, -0.004056958, 0.1285}; // Inserted position
    std::vector<double> iiwa_pink_3{0.669009459, -0.004056958, 0.16122}; //removal 
    std::vector<double> iiwa_pink_pose_orientation_1{ 0.9125889, -0.4084272, 0.0190894, 0.0020645 };

    modifyVector(iiwa_pink_0, "x",   0);
    modifyVector(iiwa_pink_1, "x",   0);
    modifyVector(iiwa_pink_2, "x",   0);
    modifyVector(iiwa_pink_3, "x",   0);

    
    add_orientation_constraints(*iiwa_pink_move_group_ptr, iiwa_pink_group, vector_to_quat(iiwa_pink_pose_orientation_1));
    
    plan_to_target_pose(*iiwa_pink_move_group_ptr, *visual_tools_ptr, iiwa_pink_group, iiwa_pink_0, iiwa_pink_pose_orientation_1);
    ROS_INFO("YAY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    // add_orientation_constraints(*iiwa_pink_move_group_ptr, iiwa_pink_group, vector_to_quat(iiwa_pink_pose_orientation_1));

    plan_to_target_pose(*iiwa_pink_move_group_ptr, *visual_tools_ptr, iiwa_pink_group, iiwa_pink_1, iiwa_pink_pose_orientation_1);
    ROS_INFO("YAY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    plan_to_target_pose(*iiwa_pink_move_group_ptr, *visual_tools_ptr, iiwa_pink_group, iiwa_pink_2, iiwa_pink_pose_orientation_1);
    ROS_INFO("YAY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    // plan_to_target_pose(*iiwa_pink_move_group_ptr, *visual_tools_ptr, iiwa_pink_group, iiwa_pink_3, iiwa_pink_pose_orientation_1);
    // ROS_INFO("YAY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    // plan_to_target_pose(*iiwa_pink_move_group_ptr, *visual_tools_ptr, iiwa_pink_group, iiwa_pink_0, iiwa_pink_pose_orientation_1);
    // ROS_INFO("YAY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    delete_constraints(*iiwa_pink_move_group_ptr);




    ros::waitForShutdown();
    delete (planning_scene_interface_ptr);
    delete (iiwa_pink_move_group_ptr);
    return 0;
}