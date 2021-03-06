#ifndef EXTERNALOUTPUT_H_
#define EXTERNALOUTPUT_H_

#include <string> 
#include <map>
#include <queue>
#include "../MediaDefinitions.h"
#include "codecs/VideoCodec.h"
#include "codecs/AudioCodec.h"
#include "MediaProcessor.h"
#include "boost/thread.hpp"
#include "logger.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}

namespace erizo{
#define UNPACKAGE_BUFFER_SIZE 200000
  class WebRtcConnection;

  class ExternalOutput : public MediaSink, public RawDataReceiver, public FeedbackSource {
      DECLARE_LOGGER();
    public:
      ExternalOutput ();
      virtual ~ExternalOutput();
      bool init(const std::string path, const std::string name, const std::string room, const std::string width, const std::string height);
      int deliverAudioData(char* buf, int len);
	  int deliverVideoData(char* buf, int len);
      void receiveRawData(RawDataPacket& packet);

    private:
      OutputProcessor* op_;
      unsigned char* decodedBuffer_;
      char* sendVideoBuffer_;
      bool initContext();
      int sendFirPacket();
      void encodeLoop();

      bool running;

      struct timeval tv;
      boost::mutex queueMutex_;
      boost::thread thread_, encodeThread_;
      std::queue<RawDataPacket> packetQueue_;
      AVStream        *video_st, *audio_st;
      
      AudioEncoder* audioCoder_;
      int gotUnpackagedFrame_;
      int unpackagedSize_;
      int prevEstimatedFps_;
      int warmupfpsCount_;
      int sequenceNumberFIR_;
      int videoW, videoH;
//      unsigned long long lastTime_;
      bool KFrame;
      double lastKeyFrame, lastTs;
      int video_stream_index, bufflen, aviores_, writeheadres_;


      AVFormatContext *context_;
      AVOutputFormat *oformat_;
      AVCodec *videoCodec_, *audioCodec_; 
      AVCodecContext *videoCodecCtx_, *audioCodecCtx_;
      InputProcessor *in;


      AVPacket avpacket;
      unsigned char* unpackagedBufferpart_;
      unsigned char deliverMediaBuffer_[3000];
      unsigned char unpackagedBuffer_[UNPACKAGE_BUFFER_SIZE];
      unsigned char unpackagedAudioBuffer_[UNPACKAGE_BUFFER_SIZE/10];
      unsigned long long initTime_;
      double videoTs_;
      std::string globalpath;
  };
}
#endif /* EXTERNALOUTPUT_H_ */
