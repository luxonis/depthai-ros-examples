#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    bool isStarted_ = false;
    unsigned long int counter_ = 0;
    rclcpp::Time startTime_;
    MinimalSubscriber()
    : Node("minimal_scan_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:

    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      if(!isStarted_){
        startTime_ = rclcpp::Clock().now();	
        isStarted_ = true;
      }
      rclcpp::Time endTime_ = rclcpp::Clock().now();

      auto elapsed_time = endTime_ - startTime_;
      counter_++;
      double fps = static_cast<double>(counter_) / elapsed_time.seconds();
      RCLCPP_INFO(this->get_logger(), "I heard fps is -> %f", fps);
      RCLCPP_INFO(this->get_logger(), "range size is -> %f", msg->ranges.size());
      // std::cout << msg->ranges << std::endl;
      int count = 0;
      for(auto val : msg->ranges){
        std::cout << val << std::endl;
        count++;
      }
      std::cout << "---------------------> Size is... " << count << std::endl;
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}