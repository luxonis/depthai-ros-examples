#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
using std::placeholders::_1;


class MinimalSubscriber : public rclcpp::Node
{
  public:
    bool isStarted_ = false;
    unsigned long int counter_ = 0;
    rclcpp::Time startTime_;
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "stereo/depth", 5, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
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
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}