// image_delay_node.cpp
#include <chrono>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class ImageDelayNode : public rclcpp::Node
{
public:
    ImageDelayNode() : Node("image_delay_node")
    {
        this->declare_parameter("input_topic", "rgb_camera");
        this->declare_parameter("output_topic", "rgb_camera_delayed");
        this->declare_parameter("delay_ms", 500);

        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        int delay_ms = this->get_parameter("delay_ms").as_int();

        delay_duration_ = std::chrono::milliseconds(delay_ms);

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&ImageDelayNode::imageCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);

        timer_ = this->create_wall_timer(10ms, std::bind(&ImageDelayNode::publishDelayed, this));

        RCLCPP_INFO(this->get_logger(), "Delaying images by %ld ms", delay_duration_.count());
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::deque<std::pair<rclcpp::Time, sensor_msgs::msg::Image::SharedPtr>> buffer_;
    std::chrono::milliseconds delay_duration_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        buffer_.emplace_back(this->now(), msg);
    }

    void publishDelayed()
    {
        if (buffer_.empty()) return;

        rclcpp::Time now = this->now();
        while (!buffer_.empty())
        {
            auto &entry = buffer_.front();
            if ((now - entry.first) >= rclcpp::Duration::from_seconds(delay_duration_.count() / 1000.0))
            {
                pub_->publish(*entry.second);
                buffer_.pop_front();
            }
            else
            {
                break;
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageDelayNode>());
    rclcpp::shutdown();
    return 0;
}

