#ifndef H264_IMAGE_TRANSPORT_SUBSCRIBER_H
#define H264_IMAGE_TRANSPORT_SUBSCRIBER_H

#include <string>

#include <image_transport/simple_subscriber_plugin.h>
#include <image_transport/transport_hints.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CompressedImage.h>

#include <boost/shared_ptr.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
}

namespace h264_image_transport {

class H264Subscriber
    : public image_transport::SimpleSubscriberPlugin< sensor_msgs::CompressedImage > {
public:
  H264Subscriber();
  virtual ~H264Subscriber();

  virtual std::string getTransportName() const { return "h264"; }

protected:
  virtual void internalCallback(const sensor_msgs::CompressedImage::ConstPtr &message,
                                const Callback &user_cb);
  virtual void subscribeImpl(ros::NodeHandle &nh, const std::string &base_topic,
                             uint32_t queue_size, const Callback &callback,
                             const ros::VoidPtr &tracked_object,
                             const image_transport::TransportHints &transport_hints);

private:
  boost::shared_ptr< AVCodecContext > decoder_ctx_;
};

}; // namespace h264_image_transport

#endif
