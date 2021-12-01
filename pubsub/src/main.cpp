#include <iostream>
#include <getopt.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <fstream>
#include <numeric>

#include <qcsnpe.hpp>
#include  <utils.hpp>

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/b_box.hpp"     // CHANGE

using namespace std::chrono_literals;

void print_help() {
    std::cout << "\nDESCRIPTION:\n"
                    << "Application for Detecting Person using SNPE C++ API on RB5\n\n"
                    << "Required Argument:\n"
                    << "-m <Model-Path>: Give path of DLC File. You can avoid this while running to get a reference frame.\n\n"
                    << "Optional Arguments:\n"
                    << "-r <Runtime>: Specify the Runtime to run network Eg, CPU(0), GPU(1), DSP(2), AIP(3)\n";
    return;
}


class MinimalPublisher : public rclcpp::Node
{
public:
	int runtime;  //CPU(0), GPU(1), DSP(2)
	float original_width;
	float original_height;
	float iou;
	std::string model_path;
	cv::VideoCapture cap;
	cv::VideoWriter video;
	int fourcc;
	cv::Mat frame;
	cv::Mat resized_img;
	std::vector<cv::Rect> found;

	Qcsnpe *qc;//(model_path, output_layers, runtime);  //Initialize Qcsnpe Class
	std::vector<std::string> output_layers;
	//Qcsnpe qc;//(model_path, output_layers, runtime);  //Initialize Qcsnpe Class



  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)//, qc("assets/mobilenet_ssd.dlc", {OUTPUT_LAYER_1, OUTPUT_LAYER_2}, 3)
  {
	runtime = 3;
	model_path = "src/pubsub/assets/mobilenet_ssd.dlc";
	fourcc = cv::VideoWriter::fourcc('X','V','I','D');
	output_layers = {OUTPUT_LAYER_1, OUTPUT_LAYER_2};
	//*qc = std::make_shared<Qcsnpe>(model_path, output_layers, runtime);
	qc = new Qcsnpe(model_path, output_layers, runtime);
	original_width = cap.get(cv::CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	original_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
	std::cout << "Resolution of the video : " << original_width << " x " << original_height << std::endl;
	video.open("Vid.mp4", fourcc, 30/1, cv::Size(original_width, original_height), true);
	cap.open(0);
	if (cap.isOpened() == false)  
	{
	    std::cout << "Cannot open the video camera" << std::endl;
	    std::cin.get();
	    return;
	}

	publisher_ = this->create_publisher<custom_msg::msg::BBox>("topic", 10);    // CHANGE
	timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }


private:
  void timer_callback()
  {

    auto message = custom_msg::msg::BBox();                               // CHANGE
    unsigned int person = 0;

    if (cap.read(frame) == false) 
    {
	std::cout << "Video camera is disconnected" << std::endl;
	std::cin.get();
    }

    cv::resize(frame, resized_img, cv::Size(MOBILENET_IMG_WIDTH, MOBILENET_IMG_HEIGHT)); 
    std::map<std::string, std::vector<float>> out = qc->predict(resized_img);
    found = postprocess(out, original_height, original_width);

    for(auto r:found) {
	    cv::rectangle(frame, r, cv::Scalar(0, 0, 255), 2);
	cv::putText(frame, "Person " + std::to_string(person), r.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1);
	++person;
    }

    //Time calculation
    auto c_time = std::chrono::system_clock::now();
    std::time_t cur_time = std::chrono::system_clock::to_time_t(c_time);
    char * time_str= std::ctime(&cur_time);
    std::string time_string(time_str);
    time_string.pop_back();

    //Writing on Data on Frame
    cv::putText(frame, time_string, cv::Point(40,560), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(255,255,0), 1);
    cv::putText(frame, "Total Count :" +std::to_string(person - 1), cv::Point(40,30), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(255,255,0), 1);
    
    std::cout<<"Total Count :"<<person << std::endl;
    std::cout<<"\n\n";

    video.write(frame); //writing output video
    
    message.timestamp = time_string;    // CHANGE
    message.probability = 0.0;
    message.xmin = 0;
    message.ymin = 0;
    message.xmax = 0;
    message.ymax = 0;
    message.id = person;
    message.cls = "Person";    // CHANGE
    message.collision_warning_status = 0;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.id);    // CHANGE
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_msg::msg::BBox>::SharedPtr publisher_;         // CHANGE
  size_t count_;
};




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
