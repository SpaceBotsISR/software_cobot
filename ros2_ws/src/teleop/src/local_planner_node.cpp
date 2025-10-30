#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "octomap/OcTree.h"
#include "octomap/octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace {
using KeyId = std::uint64_t;

KeyId keyToId(const octomap::OcTreeKey& key) {
    return (static_cast<KeyId>(key.k[0]) << 32) | (static_cast<KeyId>(key.k[1]) << 16) |
           static_cast<KeyId>(key.k[2]);
}

octomap::point3d keyToCoord(const octomap::OcTree& tree, const octomap::OcTreeKey& key) {
    return tree.keyToCoord(key);
}

double euclidean(const octomap::point3d& a, const octomap::point3d& b) {
    const double dx = static_cast<double>(a.x() - b.x());
    const double dy = static_cast<double>(a.y() - b.y());
    const double dz = static_cast<double>(a.z() - b.z());
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double squaredDistance(const octomap::point3d& a, const octomap::point3d& b) {
    const double dx = static_cast<double>(a.x() - b.x());
    const double dy = static_cast<double>(a.y() - b.y());
    const double dz = static_cast<double>(a.z() - b.z());
    return dx * dx + dy * dy + dz * dz;
}

struct QueueEntry {
    KeyId id{};
    double f_cost{};
};

struct QueueCompare {
    bool operator()(const QueueEntry& lhs, const QueueEntry& rhs) const {
        return lhs.f_cost > rhs.f_cost;
    }
};
}  // namespace

class SimpleLocalPlanner : public rclcpp::Node {
   public:
    SimpleLocalPlanner() : rclcpp::Node("simple_local_planner") {
        const std::string default_map_topic =
            declare_parameter<std::string>("map_topic", "/octomap_full");
        const std::string default_pose_topic =
            declare_parameter<std::string>("pose_topic", "/space_cobot/pose");
        const std::string default_goal_topic =
            declare_parameter<std::string>("goal_topic", "/space_cobot/goal");
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        const std::string path_topic =
            declare_parameter<std::string>("path_topic", "/space_cobot/local_path");

        robot_radius_ = declare_parameter("robot_radius", 0.35);
        occupancy_threshold_ = declare_parameter("occupancy_threshold", 0.5);
        max_search_range_ = declare_parameter("max_search_range", 15.0);
        max_expansions_ = declare_parameter("max_expansions", 60000);
        line_sample_step_ = declare_parameter("line_sample_step", 0.25);
        debug_markers_ = declare_parameter("debug_markers", true);
        marker_topic_ =
            declare_parameter<std::string>("marker_topic", "/space_cobot/local_path_markers");

        if (line_sample_step_ <= 0.0) {
            RCLCPP_WARN(get_logger(),
                        "line_sample_step must be positive. Resetting to the map resolution once "
                        "it is available.");
            line_sample_step_ = 0.0;
        }

        map_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
            default_map_topic, rclcpp::QoS(1).transient_local(),
            std::bind(&SimpleLocalPlanner::mapCallback, this, std::placeholders::_1));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            default_pose_topic, rclcpp::SensorDataQoS(),
            std::bind(&SimpleLocalPlanner::poseCallback, this, std::placeholders::_1));
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            default_goal_topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
            std::bind(&SimpleLocalPlanner::goalCallback, this, std::placeholders::_1));

        path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, 10);
        if (debug_markers_) {
            marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
            RCLCPP_INFO(get_logger(),
                        "Debug markers enabled; publishing visualization markers on %s.",
                        marker_topic_.c_str());
        }

        RCLCPP_INFO(get_logger(),
                    "SimpleLocalPlanner ready. Map topic: %s, pose topic: %s, goal topic: %s, path "
                    "topic: %s",
                    default_map_topic.c_str(), default_pose_topic.c_str(),
                    default_goal_topic.c_str(), path_topic.c_str());
    }

   private:
    void mapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
        if (!abstract_tree) {
            RCLCPP_WARN(get_logger(), "Received octomap message that could not be converted.");
            return;
        }

        auto* octree = dynamic_cast<octomap::OcTree*>(abstract_tree);
        if (!octree) {
            RCLCPP_WARN(get_logger(),
                        "Received octomap that is not an OcTree. Only OcTree is supported for this "
                        "planner.");
            delete abstract_tree;
            return;
        }

        octree_.reset(octree);
        RCLCPP_INFO(get_logger(), "Updated octomap (resolution %.3f m, depth %u).",
                    octree_->getResolution(), octree_->getTreeDepth());

        if (line_sample_step_ <= 0.0) {
            line_sample_step_ = octree_->getResolution() * 0.5;
            RCLCPP_INFO(get_logger(), "line_sample_step automatically set to %.3f m",
                        line_sample_step_);
        }

        tryPlan();
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
        tryPlan();
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_pose_ = *msg;
        RCLCPP_INFO(get_logger(), "Received new goal (%.2f, %.2f, %.2f).", msg->pose.position.x,
                    msg->pose.position.y, msg->pose.position.z);
        tryPlan();
    }

    void tryPlan() {
        if (!octree_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "No octomap received yet; cannot plan path.");
            return;
        }
        if (!current_pose_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "No current pose received yet; cannot plan path.");
            return;
        }
        if (!goal_pose_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "No goal pose received yet; cannot plan path.");
            return;
        }

        nav_msgs::msg::Path path_msg;
        if (!planPath(path_msg)) {
            clearDebugMarkers();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Unable to compute a collision-free path with the current map.");
            return;
        }

        path_pub_->publish(path_msg);
        publishDebugMarkers(path_msg);
        RCLCPP_INFO(get_logger(), "Published path with %zu poses (length %.2f m).",
                    path_msg.poses.size(), estimatePathLength(path_msg));
    }

    bool planPath(nav_msgs::msg::Path& path_out) {
        RCLCPP_INFO(get_logger(), "Planning path from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f).",
                    current_pose_->pose.position.x, current_pose_->pose.position.y,
                    current_pose_->pose.position.z, goal_pose_->pose.position.x,
                    goal_pose_->pose.position.y, goal_pose_->pose.position.z);
        if (!octree_ || !current_pose_ || !goal_pose_) {
            return false;
        }

        octomap::OcTreeKey start_key;
        if (!octree_->coordToKeyChecked(current_pose_->pose.position.x,
                                        current_pose_->pose.position.y,
                                        current_pose_->pose.position.z, start_key)) {
            RCLCPP_WARN(get_logger(), "Current pose is outside of the map bounds.");
            return false;
        }

        octomap::OcTreeKey goal_key;
        if (!octree_->coordToKeyChecked(goal_pose_->pose.position.x, goal_pose_->pose.position.y,
                                        goal_pose_->pose.position.z, goal_key)) {
            RCLCPP_WARN(get_logger(), "Goal pose is outside of the map bounds.");
            return false;
        }

        const octomap::point3d start_coord = keyToCoord(*octree_, start_key);
        const octomap::point3d goal_coord = keyToCoord(*octree_, goal_key);

        if (!isCollisionFree(start_coord)) {
            RCLCPP_WARN(get_logger(), "Robot start position is in collision.");
            return false;
        }

        if (!isCollisionFree(goal_coord)) {
            RCLCPP_WARN(get_logger(), "Goal position is in collision.");
            return false;
        }

        if (euclidean(start_coord, goal_coord) < 1e-3) {
            path_out = nav_msgs::msg::Path();
            path_out.header.stamp = now();
            path_out.header.frame_id = map_frame_;
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_out.header;
            pose.pose.position.x = start_coord.x();
            pose.pose.position.y = start_coord.y();
            pose.pose.position.z = start_coord.z();
            pose.pose.orientation.w = 1.0;
            path_out.poses.push_back(pose);
            return true;
        }

        if (isLineFree(start_coord, goal_coord)) {
            path_out = createStraightPath(start_coord, goal_coord);
            return true;
        }

        std::vector<octomap::OcTreeKey> key_path;
        if (!performAStar(start_key, goal_key, start_coord, goal_coord, key_path)) {
            RCLCPP_WARN(get_logger(), "A* search failed to find a path.");
            return false;
        }

        path_out = keysToPath(key_path);
        return true;
    }

    bool performAStar(const octomap::OcTreeKey& start_key, const octomap::OcTreeKey& goal_key,
                      const octomap::point3d& start_coord, const octomap::point3d& goal_coord,
                      std::vector<octomap::OcTreeKey>& result_path) {
        const double resolution = octree_->getResolution();
        const double max_range_sq = max_search_range_ > 0.0
                                        ? max_search_range_ * max_search_range_
                                        : std::numeric_limits<double>::infinity();

        std::vector<octomap::point3d> neighbor_directions;
        neighbor_directions.reserve(26);
        const float step = static_cast<float>(resolution);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    neighbor_directions.emplace_back(dx * step, dy * step, dz * step);
                }
            }
        }

        std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open;
        std::unordered_set<KeyId> closed;
        std::unordered_map<KeyId, double> g_score;
        std::unordered_map<KeyId, KeyId> came_from;
        std::unordered_map<KeyId, octomap::OcTreeKey> key_lookup;

        const KeyId start_id = keyToId(start_key);
        const KeyId goal_id = keyToId(goal_key);

        g_score[start_id] = 0.0;
        key_lookup[start_id] = start_key;
        came_from[start_id] = start_id;
        open.push({start_id, euclidean(start_coord, goal_coord)});

        std::size_t expansions = 0;

        while (!open.empty()) {
            if (expansions++ > static_cast<std::size_t>(max_expansions_)) {
                RCLCPP_WARN(get_logger(), "A* expansion limit reached (%d).", max_expansions_);
                return false;
            }

            const auto current_entry = open.top();
            open.pop();

            if (closed.find(current_entry.id) != closed.end()) {
                continue;
            }

            const octomap::OcTreeKey current_key = key_lookup[current_entry.id];
            const octomap::point3d current_coord = keyToCoord(*octree_, current_key);

            if (current_entry.id == goal_id) {
                reconstructPath(goal_id, start_id, came_from, key_lookup, result_path);
                return true;
            }

            closed.insert(current_entry.id);

            for (const auto& direction : neighbor_directions) {
                const octomap::point3d neighbor_coord = current_coord + direction;

                if (squaredDistance(neighbor_coord, start_coord) > max_range_sq) {
                    continue;
                }

                octomap::OcTreeKey neighbor_key;
                if (!octree_->coordToKeyChecked(neighbor_coord.x(), neighbor_coord.y(),
                                                neighbor_coord.z(), neighbor_key)) {
                    continue;
                }

                const KeyId neighbor_id = keyToId(neighbor_key);
                if (closed.find(neighbor_id) != closed.end()) {
                    continue;
                }

                if (!isCollisionFree(neighbor_coord)) {
                    continue;
                }

                const double tentative_g =
                    g_score[current_entry.id] + euclidean(current_coord, neighbor_coord);

                const auto g_it = g_score.find(neighbor_id);
                if (g_it != g_score.end() && tentative_g >= g_it->second) {
                    continue;
                }

                g_score[neighbor_id] = tentative_g;
                came_from[neighbor_id] = current_entry.id;
                key_lookup[neighbor_id] = neighbor_key;

                const double h = euclidean(neighbor_coord, goal_coord);
                open.push({neighbor_id, tentative_g + h});
            }
        }

        return false;
    }

    void reconstructPath(KeyId goal_id, KeyId start_id,
                         const std::unordered_map<KeyId, KeyId>& came_from,
                         const std::unordered_map<KeyId, octomap::OcTreeKey>& key_lookup,
                         std::vector<octomap::OcTreeKey>& out_path) const {
        out_path.clear();
        KeyId current = goal_id;
        while (true) {
            const auto key_it = key_lookup.find(current);
            if (key_it == key_lookup.end()) {
                break;
            }
            out_path.push_back(key_it->second);
            if (current == start_id) {
                break;
            }
            const auto parent_it = came_from.find(current);
            if (parent_it == came_from.end()) {
                break;
            }
            current = parent_it->second;
        }
        std::reverse(out_path.begin(), out_path.end());
    }

    nav_msgs::msg::Path keysToPath(const std::vector<octomap::OcTreeKey>& keys) const {
        nav_msgs::msg::Path path;
        path.header.stamp = now();
        path.header.frame_id = map_frame_;

        path.poses.reserve(keys.size());
        for (const auto& key : keys) {
            const octomap::point3d coord = keyToCoord(*octree_, key);
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = coord.x();
            pose.pose.position.y = coord.y();
            pose.pose.position.z = coord.z();
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        return path;
    }

    nav_msgs::msg::Path createStraightPath(const octomap::point3d& start_coord,
                                           const octomap::point3d& goal_coord) const {
        nav_msgs::msg::Path path;
        path.header.stamp = now();
        path.header.frame_id = map_frame_;

        const double distance = euclidean(start_coord, goal_coord);

        const double sample_step = std::max(line_sample_step_, 1e-3);
        const int steps = std::max(2, static_cast<int>(std::ceil(distance / sample_step)));

        path.poses.reserve(static_cast<std::size_t>(steps) + 1);
        for (int i = 0; i <= steps; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(steps);
            const octomap::point3d point =
                start_coord + (goal_coord - start_coord) * static_cast<float>(t);
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = point.z();
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }

        return path;
    }

    bool isCollisionFree(const octomap::point3d& point) const {
        if (!octree_) {
            return false;
        }

        const double resolution = octree_->getResolution();
        const int radius_cells =
            std::max(1, static_cast<int>(std::ceil(robot_radius_ / resolution)));

        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
                    const octomap::point3d sample_point =
                        point + octomap::point3d(dx * static_cast<float>(resolution),
                                                 dy * static_cast<float>(resolution),
                                                 dz * static_cast<float>(resolution));
                    if (squaredDistance(sample_point, point) > robot_radius_ * robot_radius_) {
                        continue;
                    }

                    octomap::OcTreeNode* node = octree_->search(sample_point);
                    if (!node) {
                        continue;
                    }
                    const double occ = node->getOccupancy();
                    if (occ >= occupancy_threshold_) {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    bool isLineFree(const octomap::point3d& start, const octomap::point3d& goal) const {
        if (!octree_) {
            return false;
        }

        const double distance = euclidean(start, goal);
        const double step =
            std::max(line_sample_step_, static_cast<double>(octree_->getResolution()));
        const int samples = std::max(2, static_cast<int>(std::ceil(distance / step)));

        for (int i = 0; i <= samples; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(samples);
            const octomap::point3d point = start + (goal - start) * static_cast<float>(t);
            if (!isCollisionFree(point)) {
                return false;
            }
        }

        return true;
    }

    double estimatePathLength(const nav_msgs::msg::Path& path) const {
        if (path.poses.size() < 2) {
            return 0.0;
        }
        double length = 0.0;
        for (std::size_t i = 1; i < path.poses.size(); ++i) {
            const auto& a = path.poses[i - 1].pose.position;
            const auto& b = path.poses[i].pose.position;
            const double dx = b.x - a.x;
            const double dy = b.y - a.y;
            const double dz = b.z - a.z;
            length += std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        return length;
    }

    void publishDebugMarkers(const nav_msgs::msg::Path& path) {
        if (!marker_pub_ || path.poses.empty()) {
            return;
        }

        visualization_msgs::msg::MarkerArray markers;
        markers.markers.reserve(4);

        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header = path.header;
        clear_marker.ns = "simple_local_planner";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clear_marker);

        visualization_msgs::msg::Marker line_marker;
        line_marker.header = path.header;
        line_marker.ns = "simple_local_planner";
        line_marker.id = 1;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        line_marker.scale.x = 0.05;
        line_marker.color.a = 1.0F;
        line_marker.color.r = 0.0F;
        line_marker.color.g = 1.0F;
        line_marker.color.b = 0.3F;
        for (const auto& pose : path.poses) {
            line_marker.points.emplace_back(pose.pose.position);
        }
        markers.markers.push_back(line_marker);

        const auto& start_point = path.poses.front().pose.position;
        const auto& goal_point = path.poses.back().pose.position;

        visualization_msgs::msg::Marker start_marker;
        start_marker.header = path.header;
        start_marker.ns = "simple_local_planner";
        start_marker.id = 2;
        start_marker.type = visualization_msgs::msg::Marker::SPHERE;
        start_marker.action = visualization_msgs::msg::Marker::ADD;
        start_marker.pose.position = start_point;
        start_marker.pose.orientation.w = 1.0;
        start_marker.scale.x = 0.18;
        start_marker.scale.y = 0.18;
        start_marker.scale.z = 0.18;
        start_marker.color.a = 0.9F;
        start_marker.color.r = 0.0F;
        start_marker.color.g = 0.4F;
        start_marker.color.b = 1.0F;
        markers.markers.push_back(start_marker);

        visualization_msgs::msg::Marker goal_marker;
        goal_marker.header = path.header;
        goal_marker.ns = "simple_local_planner";
        goal_marker.id = 3;
        goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
        goal_marker.action = visualization_msgs::msg::Marker::ADD;
        goal_marker.pose.position = goal_point;
        goal_marker.pose.orientation.w = 1.0;
        goal_marker.scale.x = 0.2;
        goal_marker.scale.y = 0.2;
        goal_marker.scale.z = 0.2;
        goal_marker.color.a = 0.9F;
        goal_marker.color.r = 1.0F;
        goal_marker.color.g = 0.2F;
        goal_marker.color.b = 0.1F;
        markers.markers.push_back(goal_marker);

        marker_pub_->publish(markers);
    }

    void clearDebugMarkers() {
        if (!marker_pub_) {
            return;
        }

        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = map_frame_;
        clear_marker.header.stamp = now();
        clear_marker.ns = "simple_local_planner";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clear_marker);
        marker_pub_->publish(markers);
    }

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    std::shared_ptr<octomap::OcTree> octree_;
    std::optional<geometry_msgs::msg::PoseStamped> current_pose_;
    std::optional<geometry_msgs::msg::PoseStamped> goal_pose_;

    double robot_radius_{0.35};
    double occupancy_threshold_{0.5};
    double max_search_range_{15.0};
    int max_expansions_{60000};
    double line_sample_step_{0.25};
    std::string map_frame_{"map"};
    bool debug_markers_{false};
    std::string marker_topic_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleLocalPlanner>());
    rclcpp::shutdown();
    return 0;
}
