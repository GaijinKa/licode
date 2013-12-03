#include "ExternalOutput.h"
#include "../WebRtcConnection.h"
#include "../rtputils.h"
#include <cstdio>

#include <boost/cstdint.hpp>
#include <sys/time.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/timeb.h>

namespace erizo {
#define FIR_INTERVAL_MS 4000

  DEFINE_LOGGER(ExternalOutput, "media.ExternalOutput");

  ExternalOutput::ExternalOutput(){
    //ELOG_DEBUG("Created ExternalOutput to %s", outputUrl.c_str());
    //url = outputUrl;
    sinkfbSource_=NULL;
    audioSinkSSRC_ = 0;
    videoSinkSSRC_ = 0;
    videoCodec_ = NULL;
    audioCodec_ = NULL;
    video_st = NULL;
    audio_st = NULL;
    audioCoder_ = NULL;
    prevEstimatedFps_ = 0;
    warmupfpsCount_ = 0;
    sequenceNumberFIR_ = 0;
    aviores_ = -1;
    writeheadres_=-1;
    unpackagedBufferpart_ = unpackagedBuffer_;
    initTime_ = 0;
    lastTime_ = 0;

    sinkfbSource_ = this;
    fbSink_ = NULL;
  }

  bool ExternalOutput::init(std::string path, std::string name, std::string room){

	std::cout << "Init Recorder" << std::endl;

    av_register_all();
    avcodec_register_all();
    context_ = avformat_alloc_context();
    if (context_==NULL){
      ELOG_ERROR("Error allocating memory for IO context");
      return false;
    }

    path += room+"/";
    struct stat st = {0};

    if (stat(path.c_str(), &st) == -1) {
    	std::cout << "Recording directory " << path << " not exist, creating... " << std::endl;
    	mkdir(path.c_str(), 0755);
    }
    globalpath = path+name;

//  oformat_ = av_guess_format(NULL,  url.c_str(), NULL);
    oformat_ = av_guess_format("webm", NULL, NULL);
    if (!oformat_){
      ELOG_ERROR("Error opening output file");
      return false;
    }
    context_->oformat = oformat_;
    context_->oformat->video_codec = AV_CODEC_ID_VP8;
    context_->oformat->audio_codec = AV_CODEC_ID_PCM_MULAW;

    /*start meetecho code*/
    context_->oformat->flags |= AVFMT_TS_NONSTRICT;

	struct timeb tmb;
	ftime(&tmb);
	unsigned long int timestmp_Sec =  tmb.time;
	unsigned short int timestmp_Mill = tmb.millitm;

	if (timestmp_Mill<100) timestmp_Mill += 99;
        const int n1 = snprintf(NULL, 0, "%lu", timestmp_Sec);
        char timestmp_string1[n1+1];
        int c = snprintf(timestmp_string1, n1+1, "%lu", timestmp_Sec);

	const int n2 = snprintf(NULL, 0, "%lu", timestmp_Mill);
	char timestmp_string2[n2+1];
	c = snprintf(timestmp_string2, n2+1, "%lu", timestmp_Mill);

 	globalpath = globalpath +"_"+timestmp_string1+timestmp_string2+".webm";
  	snprintf(context_->filename, sizeof(context_->filename), globalpath.c_str());
  	/* meetecho code */

//  url.copy(context_->filename, sizeof(context_->filename),0);
    video_st = NULL;
    audio_st = NULL;
    in = new InputProcessor();
    MediaInfo m;
    //    m.processorType = RTP_ONLY;
    m.hasVideo = false;
    m.hasAudio = false;
    if (m.hasAudio) {
      m.audioCodec.sampleRate = 8000;
      m.audioCodec.bitRate = 64000;
      m.audioCodec.codec = AUDIO_CODEC_VORBIS;
      audioCoder_ = new AudioEncoder();
      if (!audioCoder_->initEncoder(m.audioCodec))
        exit(0);
    }
    gotUnpackagedFrame_ = 0;
    unpackagedSize_ = 0;
    in->init(m, this);
    ELOG_DEBUG("Initialized successfully");
    return true;
  }


  ExternalOutput::~ExternalOutput(){
    ELOG_DEBUG("Destructor");
    ELOG_DEBUG("Closing Sink");
    delete in;
    in = NULL;
    
    if (context_!=NULL){
      if (writeheadres_>=0)
        av_write_trailer(context_);
      if (avio_close>=0)
        avio_close(context_->pb);
      avformat_free_context(context_);
      context_=NULL;
    }
    if (videoCodec_!=NULL){
      avcodec_close(videoCodecCtx_);
      videoCodec_=NULL;
    }
    if (audioCodec_!=NULL){
      avcodec_close(audioCodecCtx_);
      audioCodec_ = NULL;
    }
    ELOG_DEBUG("ExternalOutput closed Successfully");
    return;
  }

  void ExternalOutput::receiveRawData(RawDataPacket& packet){
    return;
  }


  int ExternalOutput::deliverAudioData(char* buf, int len){
    if (in!=NULL){
      if (videoCodec_ == NULL) {
        return 0;
      }
      rtpheader *head = (rtpheader*)buf;
      //We dont need any other payload at this time
      if(head->payloadtype != PCMU_8000_PT){
        return 0;
      }

      int ret = in->unpackageAudio(reinterpret_cast<unsigned char*>(buf), len,
          unpackagedAudioBuffer_);
      if (ret <= 0)
        return ret;
      timeval time;
      gettimeofday(&time, NULL);
      unsigned long long millis = (time.tv_sec * 1000) + (time.tv_usec / 1000);
      if (millis -lastTime_ >FIR_INTERVAL_MS){
        this->sendFirPacket();
        lastTime_ = millis;
      }
      if (initTime_ == 0) {
        initTime_ = millis;
      }
      if (millis < initTime_){
        ELOG_WARN("initTime is smaller than currentTime, possible problems when recording ");
      }
      if (ret > UNPACKAGE_BUFFER_SIZE){
        ELOG_ERROR("Unpackaged Audio size too big %d", ret);
      }
      AVPacket avpkt;
      av_init_packet(&avpkt);
      avpkt.data = unpackagedAudioBuffer_;
      avpkt.size = ret;
      avpkt.pts = millis - initTime_;
      avpkt.stream_index = 1;
      av_write_frame(context_, &avpkt);
      av_free_packet(&avpkt);
      return ret;

    }
    return 0;
  }

  int ExternalOutput::deliverVideoData(char* buf, int len){
    if (in!=NULL){
      rtpheader *head = (rtpheader*) buf;
      if (head->payloadtype == RED_90000_PT) {
        int totalLength = 12;

        if (head->extension) {
          totalLength += ntohs(head->extensionlength)*4 + 4; // RTP Extension header
        }
        int rtpHeaderLength = totalLength;
        redheader *redhead = (redheader*) (buf + totalLength);

        //redhead->payloadtype = remoteSdp_.inOutPTMap[redhead->payloadtype];
        if (redhead->payloadtype == VP8_90000_PT) {
          while (redhead->follow) {
            totalLength += redhead->getLength() + 4; // RED header
            redhead = (redheader*) (buf + totalLength);
          }
          // Parse RED packet to VP8 packet.
          // Copy RTP header
          memcpy(deliverMediaBuffer_, buf, rtpHeaderLength);
          // Copy payload data
          memcpy(deliverMediaBuffer_ + totalLength, buf + totalLength + 1, len - totalLength - 1);
          // Copy payload type
          rtpheader *mediahead = (rtpheader*) deliverMediaBuffer_;
          mediahead->payloadtype = redhead->payloadtype;
          buf = reinterpret_cast<char*>(deliverMediaBuffer_);
          len = len - 1 - totalLength + rtpHeaderLength;
        }
      }
      int estimatedFps=0;
      int ret = in->unpackageVideo(reinterpret_cast<unsigned char*>(buf), len,
          unpackagedBufferpart_, &gotUnpackagedFrame_, &estimatedFps);

      if (ret < 0)
        return 0;
      
      if (videoCodec_ == NULL) {
        if ((estimatedFps!=0)&&((estimatedFps < prevEstimatedFps_*(1-0.2))||(estimatedFps > prevEstimatedFps_*(1+0.2)))){
          prevEstimatedFps_ = estimatedFps;
        }
        if (warmupfpsCount_++ == 20){
          if (prevEstimatedFps_==0){
            warmupfpsCount_ = 0;
            return 0;
          }
          if (!this->initContext()){
            ELOG_ERROR("Context cannot be initialized properly, closing...");
            return -1;
          }
        }
        return 0;
      }

      unpackagedSize_ += ret;
      unpackagedBufferpart_ += ret;
      if (unpackagedSize_ > UNPACKAGE_BUFFER_SIZE){
        ELOG_ERROR("Unpackaged size bigget than buffer %d", unpackagedSize_);
      }
      if (gotUnpackagedFrame_ && videoCodec_!=NULL) {
        if (initTime_ == 0) {
          initTime_ = head->timestamp;
        }
        if (head->timestamp < initTime_)
        {
          ELOG_WARN("initTime is smaller than currentTime, possible problems when recording ");
        }
        unpackagedBufferpart_ -= unpackagedSize_;

        AVPacket avpkt;
        av_init_packet(&avpkt);
        avpkt.data = unpackagedBufferpart_;
        avpkt.size = unpackagedSize_;
        avpkt.pts = head->timestamp;
        avpkt.stream_index = 0;
        av_write_frame(context_, &avpkt);
        av_free_packet(&avpkt);
        gotUnpackagedFrame_ = 0;
        unpackagedSize_ = 0;
        unpackagedBufferpart_ = unpackagedBuffer_;

      }
    }
    return 0;
  }
  bool ExternalOutput::initContext() {
    ELOG_DEBUG("Init Context");
    if (oformat_->video_codec != AV_CODEC_ID_NONE && videoCodec_ == NULL) {
      videoCodec_ = avcodec_find_encoder(oformat_->video_codec);
      ELOG_DEBUG("Found Codec %s", videoCodec_->name);
      ELOG_DEBUG("Initing context with fps: %d", (int)prevEstimatedFps_);
      if (videoCodec_==NULL){
        ELOG_ERROR("Could not find codec");
        return false;
      }
      video_st = avformat_new_stream (context_, videoCodec_);
      video_st->id = 0;
      videoCodecCtx_ = video_st->codec;
      videoCodecCtx_->codec_id = oformat_->video_codec;
      videoCodecCtx_->width = 320;
      videoCodecCtx_->height = 240;
      videoCodecCtx_->time_base = (AVRational){1,(int)prevEstimatedFps_};
      videoCodecCtx_->pix_fmt = PIX_FMT_YUV420P;
      if (oformat_->flags & AVFMT_GLOBALHEADER){
        videoCodecCtx_->flags|=CODEC_FLAG_GLOBAL_HEADER;
      }
      oformat_->flags |= AVFMT_VARIABLE_FPS;
      ELOG_DEBUG("Init audio context");

      audioCodec_ = avcodec_find_encoder(oformat_->audio_codec);
      ELOG_DEBUG("Found Audio Codec %s", audioCodec_->name);
      if (audioCodec_==NULL){
        ELOG_ERROR("Could not find audio codec");
        return false;
      }
      audio_st = avformat_new_stream (context_, audioCodec_);
      audio_st->id = 1;
      audioCodecCtx_ = audio_st->codec;
      audioCodecCtx_->codec_id = oformat_->audio_codec;
      audioCodecCtx_->sample_rate = 8000;
      audioCodecCtx_->channels = 1;
      //      audioCodecCtx_->sample_fmt = AV_SAMPLE_FMT_S8;
      if (oformat_->flags & AVFMT_GLOBALHEADER){
        audioCodecCtx_->flags|=CODEC_FLAG_GLOBAL_HEADER;
      }

      context_->streams[0] = video_st;
      context_->streams[1] = audio_st;
      aviores_ = avio_open(&context_->pb, context_->filename, AVIO_FLAG_WRITE);
      if (aviores_<0){
        ELOG_ERROR("Error opening output file");
        return false;
      }
      writeheadres_ = avformat_write_header(context_, NULL);
      if (writeheadres_<0){
        ELOG_ERROR("Error writing header");
        return false;
      }
      ELOG_DEBUG("AVFORMAT CONFIGURED");
    }
    return true;
  }

  int ExternalOutput::sendFirPacket() {
    if (fbSink_ != NULL) {
      //ELOG_DEBUG("Sending FIR");
      sequenceNumberFIR_++; // do not increase if repetition
      int pos = 0;
      uint8_t rtcpPacket[50];
      // add full intra request indicator
      uint8_t FMT = 4;
      rtcpPacket[pos++] = (uint8_t) 0x80 + FMT;
      rtcpPacket[pos++] = (uint8_t) 206;
      /*
      //Length of 4
      rtcpPacket[pos++] = (uint8_t) 0;
      rtcpPacket[pos++] = (uint8_t) (4);

      // Add our own SSRC
      uint32_t* ptr = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
      ptr[0] = htonl(this->getVideoSinkSSRC());
      pos += 4;

      rtcpPacket[pos++] = (uint8_t) 0;
      rtcpPacket[pos++] = (uint8_t) 0;
      rtcpPacket[pos++] = (uint8_t) 0;
      rtcpPacket[pos++] = (uint8_t) 0;
      // Additional Feedback Control Information (FCI)
      uint32_t* ptr2 = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
      ptr2[0] = htonl(0);
      pos += 4;

      rtcpPacket[pos++] = (uint8_t) (sequenceNumberFIR_);
      rtcpPacket[pos++] = (uint8_t) 0;
      rtcpPacket[pos++] = (uint8_t) 0;
      rtcpPacket[pos++] = (uint8_t) 0;
      */
      pos = 12;
      fbSink_->deliverFeedback((char*)rtcpPacket, pos);
      return pos;
    }

    return -1;
  }

  void ExternalOutput::encodeLoop() {
    while (running == true) {
      queueMutex_.lock();
      if (packetQueue_.size() > 0) {
        op_->receiveRawData(packetQueue_.front());
        packetQueue_.pop();
        queueMutex_.unlock();
      } else {
        queueMutex_.unlock();
        usleep(1000);
      }
    }
  }

}

