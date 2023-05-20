#include <cmath>
#include <cstdlib>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "nav2_RRTstar_planner/RRT_star_planner.hpp"

namespace nav2_RRTstar_planner {

void RRTstar::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    // Seed for random numbers comment if not debugging
    std::srand(444);

    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
    node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void RRTstar::cleanup() {
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
        name_.c_str());
}

void RRTstar::activate() {
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
        name_.c_str());
}

void RRTstar::deactivate() {
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
        name_.c_str());
}

nav_msgs::msg::Path RRTstar::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) {
    nav_msgs::msg::Path global_path;

    // Checking if the goal and start state is in the global frame
    if (start.header.frame_id != global_frame_) {
        RCLCPP_ERROR(
            node_->get_logger(), "Planner will only except start position from %s frame",
            global_frame_.c_str());
        return global_path;
    }

    if (goal.header.frame_id != global_frame_) {
        RCLCPP_INFO(
            node_->get_logger(), "Planner will only except goal position from %s frame",
            global_frame_.c_str());
        return global_path;
    }

    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    //----------------------------------------- Path planning algorithm implementation ----------------------------------------------------
    
    parent_.clear();
    parent_[Point{start.pose.position.x, start.pose.position.y}] = std::shared_ptr<Point>{};

    while(true){
        auto random_pt = get_random_point();
        Point closest_pt = find_closest(random_pt);
        Point new_pt = new_point(random_pt, closest_pt);

        if(is_valid(closest_pt, new_pt)){
            parent_[new_pt] = std::make_shared<Point>(closest_pt);

            if(is_valid(new_pt, Point(goal.pose.position.x, goal.pose.position.y))){
                parent_[Point(goal.pose.position.x, goal.pose.position.y)] = std::make_shared<Point>(new_pt);
                break;
            }
        }
    }
    // is_valid(Point(start.pose.position.x, start.pose.position.y), Point(goal.pose.position.x, goal.pose.position.y));

    // RCLCPP_INFO(
    //     node_->get_logger(), "x_start:%f y_start:%f x_end:%f y_end:%f ",
    //     new_pt.x, new_pt.y, start.pose.position.x, start.pose.position.y);

    //-------------------------------------------------------------------------------------------------------------------------------------

    // calculating the number of loops for current value of interpolation_resolution_
    int total_number_of_loop = std::hypot(
                                   goal.pose.position.x - start.pose.position.x,
                                   goal.pose.position.y - start.pose.position.y) /
                               interpolation_resolution_;
    double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
    double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

    for (int i = 0; i < total_number_of_loop; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = start.pose.position.x + x_increment * i;
        pose.pose.position.y = start.pose.position.y + y_increment * i;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose.header.stamp = node_->now();
        pose.header.frame_id = global_frame_;
        global_path.poses.push_back(pose);
    }

    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header.stamp = node_->now();
    goal_pose.header.frame_id = global_frame_;
    global_path.poses.push_back(goal_pose);

    return global_path;
}

bool RRTstar::is_valid(Point a, Point b) {
    auto x_points = linspace(a.x, b.x, 100);
    auto y_points = linspace(a.y, b.y, 100);

    std::vector<Point> points{};
    points.reserve(x_points.size());

    for (int i = 0; i < x_points.size(); i++) {
        points.emplace_back(x_points[i], y_points[i]);
    }

    for (const auto& point : points) {
        unsigned mx{};
        unsigned my{};

        if (!costmap_->worldToMap(point.x, point.y, mx, my)) {
            return false;
        }

        // RCLCPP_INFO(
        //     node_->get_logger(), "x:%f y:%f cost: %d ",
        //     point.x, point.y, costmap_->getCost(mx, my));

        if (costmap_->getCost(mx, my) >= 200) {
            //TODO Add last point before collision
            return false;
        }
    }

    return true;
}

Point RRTstar::get_random_point() {
    float x = (std::rand() / (float)RAND_MAX) * costmap_->getSizeInMetersX();
    float y = (std::rand() / (float)RAND_MAX) * costmap_->getSizeInMetersY();

    return {x, y};
}

Point RRTstar::find_closest(Point pos){

    float min_dist = std::numeric_limits<float>::infinity();
    Point closest;

    for(auto& [key, value]: parent_)
    {
        double dist = std::sqrt(std::pow((pos.x - key.x), 2) + std::pow((pos.y, key.y), 2));

        if(dist < min_dist)
        {
            min_dist = dist;
            closest = key;
        }
    }

    return closest;
}

Point RRTstar::new_point(Point pt, Point closest){

    double step = 0.1;

    double norm = std::hypot(pt.x - closest.x, pt.y - closest.y);
    double new_x = ((pt.x - closest.x) / norm) * step;
    double new_y = ((pt.y - closest.y) / norm) * step;

    Point point = Point(closest.x + new_x, closest.y + new_y);

    return point;
}


std::vector<float> RRTstar::linspace(float start, float stop, std::size_t num_of_points) {
    std::vector<float> linspaced{};
    linspaced.reserve(num_of_points);

    if (num_of_points == 0) {
        return linspaced;
    } else if (num_of_points == 1) {
        linspaced.push_back(start);
        return linspaced;
    }

    auto delta = (stop - start) / (num_of_points - 1);

    for (int i = 0; i < num_of_points - 1; ++i) {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(stop);

    return linspaced;
}

}  // namespace nav2_RRTstar_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_RRTstar_planner::RRTstar, nav2_core::GlobalPlanner)
