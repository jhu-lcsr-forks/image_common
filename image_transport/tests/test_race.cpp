#include <gtest/gtest.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/advertise_options.h>

boost::mutex mut;

void connect() {
  boost::mutex::scoped_lock lock(mut);
}

void disconnect() {
  boost::mutex::scoped_lock lock(mut);
}


void load_thread(bool* &stop, const std::string suffix) {
  std::string camera_name = "camera" + suffix;
  ros::NodeHandle nh("~" + camera_name);

  ros::CallbackQueue camera_queue;

  cv::Mat image = cv::Mat(960, 1280, CV_8UC3);
  cv::randu(image, cv::Scalar::all(0), cv::Scalar::all(255));
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise(
      "image_raw", 2,
      boost::bind(connect),
      boost::bind(disconnect),
      ros::VoidPtr(), &camera_queue);

  ros::AdvertiseOptions cio =
    ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
    "camera_info", 2,
      boost::bind(connect),
      boost::bind(disconnect),
    ros::VoidPtr(), &camera_queue);
  ros::Publisher camera_info_pub = nh.advertise(cio);

  ros::WallRate loop_rate(5);
  while(not *stop) {
    camera_queue.callAvailable(ros::WallDuration(0.001));
    loop_rate.sleep();
  }
}

TEST(race, fourway) {
  bool stop = false;

  ros::NodeHandle nh;

  boost::thread it1(boost::bind(load_thread, &stop, "/1"));
  boost::thread it2(boost::bind(load_thread, &stop, "/2"));

  /**
  At this point, the GazeboRosCameraUtils class would have been instantiated
  twice, but each ROS camera would be missing some of the transport plugin types.

  Expected output of `rostopic list | grep image_publisher | grep parameter`:

    /image_publisher/camera/1/image_raw/compressed/parameter_updates
    /image_publisher/camera/1/image_raw/compressedDepth/parameter_updates
    /image_publisher/camera/1/image_raw/theora/parameter_updates
    /image_publisher/camera/2/image_raw/compressed/parameter_updates
    /image_publisher/camera/2/image_raw/compressedDepth/parameter_updates
    /image_publisher/camera/2/image_raw/theora/parameter_updates

  Output with GazeboRosCameraUtils:

    /image_publisher/camera/1/image_raw/compressedDepth/parameter_updates
    /image_publisher/camera/2/image_raw/compressed/parameter_updates
    /image_publisher/camera/2/image_raw/theora/parameter_updates

  Note that neither cameras share the same transport types.

  **/

  ros::WallDuration(30).sleep();


  stop = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
