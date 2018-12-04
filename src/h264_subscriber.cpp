#include <string>

#include "h264_image_transport/h264_subscriber.h"

#include <ros/exception.h>
#include <ros/node_handle.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/shared_ptr.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/hwcontext.h>
#include <libavutil/opt.h>
}

namespace h264_image_transport {

H264Subscriber::H264Subscriber() : decoder_ctx_(NULL), convert_ctx_(NULL) {}

H264Subscriber::~H264Subscriber() {
  if (decoder_ctx_) {
    avcodec_free_context(&decoder_ctx_);
  }
}

void H264Subscriber::init(ros::NodeHandle param_nh) {
  if (decoder_ctx_) {
    return;
  }

  // init libavformat
  av_register_all();
  av_log_set_level(AV_LOG_FATAL);

  // find h264 decoder
  AVCodec *const decoder(avcodec_find_decoder_by_name("h264"));
  if (!decoder) {
    throw ros::Exception("Cannot find h264 decoder");
  }

  // allocate h264 decoder context
  decoder_ctx_ = avcodec_alloc_context3(decoder);
  if (!decoder_ctx_) {
    throw ros::Exception("Cannot allocate h264 decoder context");
  }

  // open decoder
  if (avcodec_open2(decoder_ctx_, decoder, NULL) < 0) {
    throw ros::Exception("Failed to open h264 codec");
  }
}

void H264Subscriber::subscribeImpl(ros::NodeHandle &nh, const std::string &base_topic,
                                   uint32_t queue_size, const Callback &callback,
                                   const ros::VoidPtr &tracked_object,
                                   const image_transport::TransportHints &transport_hints) {
  if (!decoder_ctx_) {
    init(transport_hints.getParameterNH());
  }

  SimpleSubscriberPlugin::subscribeImpl(nh, base_topic, queue_size, callback, tracked_object,
                                        transport_hints);
}

struct AVFrameDeleter {
  void operator()(AVFrame *frame) {
    if (frame) {
      av_frame_free(&frame);
    }
  }
};

void H264Subscriber::internalCallback(const sensor_msgs::CompressedImage::ConstPtr &message,
                                      const Callback &user_cb) {
  //
  AVPacket packet;
  av_init_packet(&packet);
  packet.size = message->data.size();
  packet.data = (uint8_t *)&message->data[0];

  //
  if (avcodec_send_packet(decoder_ctx_, &packet) < 0) {
    ROS_ERROR("Cannot send h264 packet to decoder");
    return;
  }

  while (true) {
    // allocate frame
    boost::shared_ptr< AVFrame > frame(av_frame_alloc(), AVFrameDeleter());
    if (!frame) {
      ROS_ERROR("Cannot allocate frame");
      return;
    }

    // receive frame from the decoder
    const int res(avcodec_receive_frame(decoder_ctx_, frame.get()));
    if (res == AVERROR(EAGAIN) || res == AVERROR_EOF) {
      // no more frames in the packet
      return;
    } else if (res < 0) {
      ROS_ERROR("Cannot decode h264 packet");
      return;
    }

    // allocate output message
    const sensor_msgs::ImagePtr out(new sensor_msgs::Image());
    out->header = message->header;
    out->height = frame->height;
    out->width = frame->width;
    out->encoding = sensor_msgs::image_encodings::BGR8;
    out->step = 3 * frame->width;
    out->data.resize(3 * frame->width * frame->height);

    // layout data by converting color spaces (YUV -> RGB)
    convert_ctx_ = sws_getCachedContext(convert_ctx_, frame->width, frame->height,
                                        AV_PIX_FMT_YUV420P, frame->width, frame->height,
                                        AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL, NULL);
    int stride = 3 * frame->width;
    uint8_t *dst = &out->data[0];
    sws_scale(convert_ctx_,
              // src data
              frame->data, frame->linesize, 0, frame->height,
              // dst data
              &dst, &stride);

    // exec user callback
    user_cb(out);
  }
}

}; // namespace h264_image_transport
