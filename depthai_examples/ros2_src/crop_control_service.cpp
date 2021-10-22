#include "rclcpp/rclcpp.hpp"
#include <depthai_ros_msgs/srv/NormalizedImageCrop.h>
#include <stdio.h>
#include <ctype.h>

static constexpr float stepSize = 0.02;

void boundAdjuster(double& value){
    if(value < 0){
        std::cout << "values should always be greater than 0." << std::endl;
        std::cout << "Resetting to 0." << std::endl;
        value = 0;
    }
    else if(value > 1){
        std::cout << "values should always be less than 1." << std::endl;
        std::cout << "Resetting to 1." << std::endl;
        value = 1;
    }
    return;
}


int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("crop_control_service");
    std::string serviceName = "crop_control_srv";

    /*     
    int badParams = 0;
    badParams += !pnh.getParam("service_name", serviceName);

    if (badParams > 0){
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find %d of the parameters");
    }
    */
    
    rclcpp::Client<depthai_ros_msgs::srv::NormalizedImageCrop>::SharedPtr client =
    node->create_client<depthai_ros_msgs::srv::NormalizedImageCrop>(serviceName);

    depthai_ros_msgs::srv::NormalizedImageCrop srvMsg;
    srvMsg.request.top_left.x = 0.2; 
    srvMsg.request.top_left.y = 0.2; 
    srvMsg.request.bottom_right.x = 0.2; 
    srvMsg.request.bottom_right.y = 0.2; 
    
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
            }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    std::cout << "Use the following keys to control the cropping region" << std::endl; 
    std::cout << "  Q/W -> Increment/Decrement the topleft X position" << std::endl;
    std::cout << "  A/S -> Increment/Decrement the topleft Y position" << std::endl;
    std::cout << "  E/R -> Increment/Decrement the bottomright X position" << std::endl;
    std::cout << "  D/F -> Increment/Decrement the bottomright Y position" << std::endl;
    std::cout << "  Preess ctrl+D to exit." << std::endl;
    char c;
    bool sendSignal = false;

    while (ros::ok()){
        c = std::tolower(getchar());
        switch(c){
            case 'w':
                srvMsg.request.top_left.x -= stepSize;
                boundAdjuster(srvMsg.request.top_left.x);
                sendSignal = true;
                break;
            case 'q':
                srvMsg.request.top_left.x += stepSize;
                boundAdjuster(srvMsg.request.top_left.x);
                sendSignal = true;
                break;
            case 'a':
                srvMsg.request.top_left.y += stepSize;
                boundAdjuster(srvMsg.request.top_left.y);
                sendSignal = true;
                break;
            case 's':
                srvMsg.request.top_left.y -= stepSize;
                boundAdjuster(srvMsg.request.top_left.y);
                sendSignal = true;
                break;
            case 'e':
                srvMsg.request.bottom_right.x += stepSize;
                boundAdjuster(srvMsg.request.bottom_right.x);
                sendSignal = true;
                break;
            case 'r':
                srvMsg.request.bottom_right.x -= stepSize;
                boundAdjuster(srvMsg.request.bottom_right.x);
                sendSignal = true;
                break;
            case 'd':
                srvMsg.request.bottom_right.y += stepSize;
                boundAdjuster(srvMsg.request.bottom_right.y);
                sendSignal = true;
                break;
            case 'f':
                srvMsg.request.bottom_right.y -= stepSize;
                boundAdjuster(srvMsg.request.bottom_right.y);
                sendSignal = true;
                break;
            default:
                // TODO(sachin): Use RCLCPP_INFO instead of cout.
                std::cout << " Entered Invalid Key..!!!" << std::endl;
                std::cout << "Use the following keys to control the cropping region" << std::endl; 
                std::cout << "  Q/W -> Increment/Decrement the topleft X position" << std::endl;
                std::cout << "  A/S -> Increment/Decrement the topleft Y position" << std::endl;
                std::cout << "  E/R -> Increment/Decrement the bottomright X position" << std::endl;
                std::cout << "  D/F -> Increment/Decrement the bottomright Y position" << std::endl;
                std::cout << "  Preess ctrl+D to exit." << std::endl;
        }

        if (sendSignal){
            std::cout << "Top left Position -> (" << srvMsg.request.top_left.x << ", " << srvMsg.request.top_left.y << ")" << std::endl; 
            std::cout << "Bottion right Position -> (" << srvMsg.request.bottom_right.x << ", " << srvMsg.request.bottom_right.y << ")" << std::endl; 
            auto result = client->async_send_request(srvMsg.request);
            if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service crop_control_srv");
            }
            sendSignal = false;                   
        }
    }

    return 0;
}