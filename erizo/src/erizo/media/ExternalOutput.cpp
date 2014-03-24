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
    myheader = "MEETECHO";
    sinkfbSource_=NULL;
    audioSinkSSRC_ = 0;
    videoSinkSSRC_ = 0;
    videoCodec_ = NULL;
    audioCodec_ = NULL;
    video_st = NULL;
    audio_st = NULL;
    KFrame = false;
    audioCoder_ = NULL;
    dumpRTP = NULL;
    prevEstimatedFps_ = 0;
    warmupfpsCount_ = 0;
    sequenceNumberFIR_ = 0;
    aviores_ = -1;
    writeheadres_=-1;
    unpackagedBufferpart_ = unpackagedBuffer_;
    initTime_ = 0;
    failCount = 0;
    lastTs = 0;
    lastKeyFrame = 0;
    sinkfbSource_ = this;
    fbSink_ = NULL;
    avgBitrate = 0;
    lastAvgBitrate = 0;
    lastWarmupTime_ = 0;
    successRate = 0;
  }

  bool ExternalOutput::init(std::string path, std::string name, std::string room, std::string width, std::string height){


    std::cout << "Init Recorder " <<  width << " x " << height << std::endl;

    videoW = atoi(width.c_str());
    videoH = atoi(height.c_str());

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
    context_->oformat->audio_codec = AV_CODEC_ID_VORBIS;

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
	c = snprintf(timestmp_string2, n2+1, "%d", timestmp_Mill);

        dumppath = globalpath+"_"+timestmp_string1+timestmp_string2+".rtp";
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
    thread_ = boost::thread(&ExternalOutput::sendLoop, this);
    sending_ = true;
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
//    if (audioCodec_!=NULL){
//      avcodec_close(audioCodecCtx_);
//      audioCodec_ = NULL;
//    }

   if (dumpRTP!=NULL) {
        fseek(dumpRTP, 0L, SEEK_END);
        size_t fsize = ftell(dumpRTP);
        fseek(dumpRTP, 0L, SEEK_SET);
        ELOG_DEBUG("File is %zu bytes\n", fsize);

        fclose(dumpRTP);
   }

    ELOG_DEBUG("ExternalOutput closed Successfully");
    return;
  }

  void ExternalOutput::receiveRawData(RawDataPacket& packet){
    return;
  }


  int ExternalOutput::deliverAudioData(char* buf, int len){
/*    if (in!=NULL){
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

    }*/
    return 0;
  }

  int ExternalOutput::writeVideoData(char* buf, int len){

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
          unpackagedBufferpart_, &gotUnpackagedFrame_, &estimatedFps, &videoTs_, &KFrame);


     if (ret < 0)
        return 0;


     //DUMP RTP PACKET
     if (dumpRTP!=NULL) {

 	int myh = fwrite(myheader, sizeof(char), 8, dumpRTP);
        ELOG_DEBUG("  >> Printing header pt.1: %zu bytes\n", myh);

        uint16_t header_bytes = htons(len);
        int hby = fwrite(&header_bytes, sizeof(uint16_t), 1, dumpRTP);
        ELOG_DEBUG("  >> Printing header pt.2: %zu bytes\n", hby);

        /* Save packet on file */
//        int hbuf = fwrite(reinterpret_cast<unsigned char*>(buf), sizeof(char), len, dumpRTP);
	int hbuf = 0, tot = len;
 	while(tot > 0) {
         	hbuf = fwrite(reinterpret_cast<unsigned char*>(buf+len-tot), sizeof(char), tot, dumpRTP);
         	ELOG_DEBUG("  >> Printing buffer: %zu bytes\n", hbuf);
  		if(hbuf <= 0) {
          		ELOG_DEBUG("  >> >> Error!\n");
   			break;
  		}
  		tot -= hbuf;
 	}
//       ELOG_DEBUG("  >> Printing buffer: %zu bytes\n", hbuf);
     }



//      ELOG_DEBUG("SeqNum %zu", head->seqnum);

//      avgBitrate += (uint32_t) len;
//      uint32_t elapTBit = (videoTs_ - lastWarmupTime_)/90000;
//      if (elapTBit>=10) {
//          ELOG_DEBUG("LastWarmup %zu millisec, videoTs %zu millisec", lastWarmupTime_, videoTs_);
//          ELOG_DEBUG("Bitrate Warmup Time Elapsed %zu sec, received %zu bytes", elapTBit, avgBitrate);
//          avgBitrate = (avgBitrate*8)/elapTBit;
//          if ((avgBitrate!=0)&&((avgBitrate < lastAvgBitrate*(1-0.3))||(avgBitrate > lastAvgBitrate*(1+0.3)))&&(lastAvgBitrate!=0)){
//                   ELOG_DEBUG("Bitrate Variation, send FIR and REMB at value %zu",avgBitrate);
//                   this->sendFirPacket();
//                   this->sendRembPacket(avgBitrate);
//          } else {
//                   successRate++;
//                   if (successRate>3) {
//                           uint32_t avgBitrateReUP = avgBitrate*(1+0.5);
//                           if (avgBitrateReUP<128000) avgBitrateReUP = 128000;
//                           ELOG_DEBUG("30 sec ok, increase BW at %zu",avgBitrateReUP);
//                           this->sendRembPacket(avgBitrateReUP);
//                           successRate = 0;
//                    }
//          }
//          lastAvgBitrate = avgBitrate;
//          avgBitrate = 0;
//         lastWarmupTime_ = videoTs_;
//     }


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
          initTime_ = videoTs_;
          this->sendFirPacket();
        }

        if (videoTs_ < lastTs)
        {
          ELOG_WARN("videoTs is smaller than lastTs, possible problems when recording");
//	  failCount++;
//	  this->sendFirPacket();
        }

        if (((videoTs_ - lastKeyFrame)/90000 > FIR_INTERVAL_MS) || failCount>=10) {
        	this->sendFirPacket();
        	ELOG_WARN("Too messy packet or Too much time from last FIR: %f, resend FIR PACKET",(videoTs_ - lastKeyFrame)/90000);
		failCount=0;
        }

        unpackagedBufferpart_ -= unpackagedSize_;

        AVPacket avpkt;
        av_init_packet(&avpkt);
        avpkt.data = unpackagedBufferpart_;
        avpkt.size = unpackagedSize_;
        avpkt.pts = (videoTs_ - initTime_)/90;
//	avpkt.dts = AV_NOPTS_VALUE;
        avpkt.stream_index = 0;
        if(KFrame) {
        	avpkt.flags |= AV_PKT_FLAG_KEY;
        	lastKeyFrame = videoTs_;
//		ELOG_WARN("KEYFRAME, setting lastKeyframe to %f", lastKeyFrame);
        }
        av_write_frame(context_, &avpkt);
        av_free_packet(&avpkt);
        gotUnpackagedFrame_ = 0;
        unpackagedSize_ = 0;
        unpackagedBufferpart_ = unpackagedBuffer_;
        lastTs = videoTs_;
      }
    }
    return 0;
  }


//   int ExternalOutput::deliverAudioData(char* buf, int len) {
//     this->queueData(buf,len,AUDIO_PACKET);
//     return 0;
//   }

   int ExternalOutput::deliverVideoData(char* buf, int len) {
     rtcpheader *head = reinterpret_cast<rtcpheader*>(buf);
     if (head->isRtcp()){
       return 0;
     }
     this->queueData(buf,len,VIDEO_PACKET);
     return 0;
   }

void ExternalOutput::queueData(char* buffer, int length, packetType type){
     if (in==NULL) {
       return;
     }
     boost::mutex::scoped_lock lock(queueMutex_);

     if (type == VIDEO_PACKET){
       videoQueue_.pushPacket(buffer, length);
     }else{
       audioQueue_.pushPacket(buffer, length);
     }
     cond_.notify_one();
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
      videoCodecCtx_->width = videoW;
      videoCodecCtx_->height = videoH;
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

      //audio_st = avformat_new_stream (context_, audioCodec_);
      //audio_st->id = 1;
      //audioCodecCtx_ = audio_st->codec;
      //audioCodecCtx_->codec_id = oformat_->audio_codec;
      //audioCodecCtx_->sample_rate = 8000;
      //audioCodecCtx_->channels = 1;
      //      audioCodecCtx_->sample_fmt = AV_SAMPLE_FMT_S8;
      //if (oformat_->flags & AVFMT_GLOBALHEADER){
      //  audioCodecCtx_->flags|=CODEC_FLAG_GLOBAL_HEADER;
      //}

      context_->streams[0] = video_st;
//      context_->streams[1] = audio_st;
      aviores_ = avio_open(&context_->pb, context_->filename, AVIO_FLAG_WRITE);
      if (aviores_<0){
        ELOG_ERROR("Error opening output file");
        return false;
      }
      writeheadres_ = avformat_write_header(context_, NULL);
      if (writeheadres_<0){
        ELOG_ERROR("Error writing header %d",writeheadres_);
        return false;
      }

      /*creating dumpRTP */
      dumpRTP = fopen(dumppath.c_str(), "wb");
      if(dumpRTP == NULL) {
              ELOG_ERROR("Error creating dump.rtp\n");
              return false;
      }


      ELOG_DEBUG("AVFORMAT CONFIGURED");
    }
    return true;
  }

  int ExternalOutput::sendRembPacket(uint32_t bitrate) {

   if (fbSink_) {
	   ELOG_INFO("Generating EXTERNAL REMB Packet (bitrate=%zu)",bitrate);
	   int pos = 0;
	   uint8_t rtcpPacket[48];
	   uint8_t FMT = 15;
	   rtcpPacket[pos++] = (uint8_t) 0x80 + FMT;
	   rtcpPacket[pos++] = (uint8_t) 206;

	   //Length of 5
	   rtcpPacket[pos++] = (uint8_t) 0;
	   rtcpPacket[pos++] = (uint8_t) (5);

//	   ELOG_INFO("packet sender media SSRC is %lu", 55543);

	   // Add our own SSRC
	   uint32_t* ptr = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
	   ptr[0] = htonl(55543);
	   pos += 4;

	   //pezzotto per comunicazione bitrate!
           uint32_t* ptr3 = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
           ptr3[0] = bitrate;
           pos += 4;

//	   rtcpPacket[pos++] = (uint8_t) 0;
//	   rtcpPacket[pos++] = (uint8_t) 0;
//	   rtcpPacket[pos++] = (uint8_t) 0;
//	   rtcpPacket[pos++] = (uint8_t) 0;

	   rtcpPacket[pos++] = 'R';
	   rtcpPacket[pos++] = 'E';
	   rtcpPacket[pos++] = 'M';
	   rtcpPacket[pos++] = 'B';

	   rtcpPacket[pos++] = (uint8_t) 1;
	   uint8_t brExp = 0;
	   for(uint32_t i=0; i<64; i++) {
	          if(bitrate <= ((uint32_t)262143 << i))  {
        	      brExp = i;
	              break;
          	  }
      	   }

	   const uint32_t brMantissa = (bitrate >> brExp);
	   rtcpPacket[pos++]=(uint8_t)((brExp << 2) + ((brMantissa >> 16) & 0x03));
	   rtcpPacket[pos++]=(uint8_t)(brMantissa >> 8);
	   rtcpPacket[pos++]=(uint8_t)(brMantissa);

	   ELOG_DEBUG("the ssrc is %u", this->getVideoSinkSSRC());

	   uint32_t* ptr2 = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
	   ptr2[0] = htonl(this->getVideoSinkSSRC());
	   pos += 4;

	   fbSink_->deliverFeedback((char*)rtcpPacket, pos);
	   return pos;
	}
   return -1;
 }



  int ExternalOutput::sendFirPacket() {
    if (fbSink_ != NULL) {
      ELOG_DEBUG("Sending FIR");
      sequenceNumberFIR_++; // do not increase if repetition
      int pos = 0;
      uint8_t rtcpPacket[50];
      // add full intra request indicator
      uint8_t FMT = 4;
      rtcpPacket[pos++] = (uint8_t) 0x80 + FMT;
      rtcpPacket[pos++] = (uint8_t) 206;
      pos = 12;
      fbSink_->deliverFeedback((char*)rtcpPacket, pos);
      return pos;
    }
    return -1;
  }

  //RR ROUTINES

 void init_seq(sourcestat *s, uint16_t seq)   {
       s->base_seq = seq;
       s->max_seq = seq;
       s->bad_seq = RTP_SEQ_MOD + 1;   /* so seq == bad_seq is false */
       s->cycles = 0;
       s->received = 0;
       s->received_prior = 0;
       s->expected_prior = 0;
  }

 int update_seq(sourcestat *s, uint16_t seq)   {
       uint16_t udelta = seq - s->max_seq;
       const int MAX_DROPOUT = 3000;
       const int MAX_MISORDER = 100;
       const int MIN_SEQUENTIAL = 2;

       /*
        * Source is not valid until MIN_SEQUENTIAL packets with
        * sequential sequence numbers have been received.
        */
       if (s->probation) {
           /* packet is in sequence */
           if (seq == s->max_seq + 1) {
               s->probation--;
               s->max_seq = seq;
               if (s->probation == 0) {
                   init_seq(s, seq);
                   s->received++;
                   return 1;
               }
           } else {
               s->probation = MIN_SEQUENTIAL - 1;
               s->max_seq = seq;
           }
           return 0;
       } else if (udelta < MAX_DROPOUT) {
           /* in order, with permissible gap */
           if (seq < s->max_seq) {
               /*
                * Sequence number wrapped - count another 64K cycle.
                */
               s->cycles += RTP_SEQ_MOD;
           }
           s->max_seq = seq;
       } else if (udelta <= RTP_SEQ_MOD - MAX_MISORDER) {
           /* the sequence number made a very large jump */
           if (seq == s->bad_seq) {
               /*
                * Two sequential packets -- assume that the other side
                * restarted without telling us so just re-sync
                * (i.e., pretend this was the first packet).
                */
               init_seq(s, seq);
           }
           else {
               s->bad_seq = (seq + 1) & (RTP_SEQ_MOD-1);
               return 0;
           }
       } else {
           /* duplicate or reordered packet */
       }
       s->received++;
       return 1;
   }

  //END RR ROUTINES

void ExternalOutput::sendLoop() {
     /* while (sending_ == true) { */
     /*   queueMutex_.lock(); */
     /*   if (packetQueue_.size() > 0) { */
     /*     op_->receiveRawData(packetQueue_.front()); */
     /*     packetQueue_.pop(); */
     /*     queueMutex_.unlock(); */
     /*   } else { */
     /*     queueMutex_.unlock(); */
     /*     usleep(1000); */
     /*   } */
     /* } */
 
     while (sending_ == true) {
       boost::unique_lock<boost::mutex> lock(queueMutex_);
       while ((!audioQueue_.getSize())&&(!videoQueue_.getSize())) {
         cond_.wait(lock);
         if (sending_ == false) {
           lock.unlock();
           return;
         }
       }
     if (audioQueue_.getSize()){
        boost::shared_ptr<dataPacket> audioP = audioQueue_.popPacket();
        this->writeAudioData(audioP->data, audioP->length);
      }
      if (videoQueue_.getSize()) {
        boost::shared_ptr<dataPacket> videoP = videoQueue_.popPacket();
        this->writeVideoData(videoP->data, videoP->length);
      }
/*
       if (packetQueue_.front().type == VIDEO_PACKET) {
         this->writeVideoData(packetQueue_.front().data, packetQueue_.front().length);
      } else {
	  this->writeAudioData(packetQueue_.front().data, packetQueue_.front().length);
      }
*/
       lock.unlock();
    }
  }

}

