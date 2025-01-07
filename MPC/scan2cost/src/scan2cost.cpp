#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>

class Scan2CostNode : public rclcpp::Node
{
public:
    Scan2CostNode()
        : Node("scan2cost")
    {
        // 파라미터 선언
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/scan");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/costmap");
        frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
        resolution_ = this->declare_parameter<double>("resolution", 0.1);
        width_ = this->declare_parameter<int>("width", 100);
        height_ = this->declare_parameter<int>("height", 100);
        origin_x_ = this->declare_parameter<double>("origin_x", -5.0);
        origin_y_ = this->declare_parameter<double>("origin_y", -5.0);

        // 토픽 구독 및 퍼블리셔 설정
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            input_topic_, 10, std::bind(&Scan2CostNode::scanCallback, this, std::placeholders::_1));

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            output_topic_, 10);

        initializeCostmap();

        // 토픽 상태 확인 타이머 설정 (0.5초 주기)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&Scan2CostNode::checkScanTopic, this));

        RCLCPP_INFO(this->get_logger(), "Scan2CostNode initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid costmap_;
    int width_, height_;
    double resolution_, origin_x_, origin_y_;
    std::string input_topic_, output_topic_, frame_id_;
    bool scan_received_ = false;

    void initializeCostmap()
    {
        costmap_.header.frame_id = frame_id_;
        costmap_.info.resolution = resolution_;
        costmap_.info.width = width_;
        costmap_.info.height = height_;
        costmap_.info.origin.position.x = origin_x_;
        costmap_.info.origin.position.y = origin_y_;
        costmap_.info.origin.position.z = 0.0;
        costmap_.info.origin.orientation.w = 1.0;

        costmap_.data.resize(width_ * height_, -1);
    }

    // /scan 토픽 확인 (타이머 기반)
    void checkScanTopic()
    {
        if (!scan_received_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for input topic: %s", input_topic_.c_str());
        }
        scan_received_ = false; // 다음 체크를 위해 초기화
    }

    // LiDAR 데이터 수신 콜백
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        scan_received_ = true; // 토픽 수신 상태 확인

        auto map = costmap_;
        std::fill(map.data.begin(), map.data.end(), 0);

        double angle = scan->angle_min;
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            double range = scan->ranges[i];
            if (range >= scan->range_min && range <= scan->range_max)
            {
                int x = static_cast<int>((range * cos(angle) - origin_x_) / resolution_);
                int y = static_cast<int>((range * sin(angle) - origin_y_) / resolution_);

                if (x >= 0 && x < width_ && y >= 0 && y < height_)
                {
                    map.data[y * width_ + x] = 100; // 장애물 표시
                }
            }
            angle += scan->angle_increment;
        }

        map.header.stamp = this->get_clock()->now();
        map.header.frame_id = frame_id_;
        map_pub_->publish(map);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Scan2CostNode>());
    rclcpp::shutdown();
    return 0;
}

