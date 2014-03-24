/*
 * WebRTCConnection.cpp
 */

#include <cstdio>
#include <inttypes.h>
#include "WebRtcConnection.h"
#include "DtlsTransport.h"
#include "SdesTransport.h"

#include "SdpInfo.h"
#include "rtputils.h"

//#define BYTETOBINARYPATTERN "%d%d%d%d%d%d%d%d"
//#define BYTETOBINARY(byte)  \
//  (byte & 0x80 ? 1 : 0), \
//  (byte & 0x40 ? 1 : 0), \
//  (byte & 0x20 ? 1 : 0), \
//  (byte & 0x10 ? 1 : 0), \
//  (byte & 0x08 ? 1 : 0), \
//  (byte & 0x04 ? 1 : 0), \
//  (byte & 0x02 ? 1 : 0), \
//  (byte & 0x01 ? 1 : 0)
namespace erizo {
  DEFINE_LOGGER(WebRtcConnection, "WebRtcConnection");

  WebRtcConnection::WebRtcConnection(bool audioEnabled, bool videoEnabled, const std::string &stunServer, int stunPort, int minPort, int maxPort) {

    ELOG_WARN("WebRtcConnection constructor stunserver %s stunPort %d minPort %d maxPort %d\n", stunServer.c_str(), stunPort, minPort, maxPort);
    video_ = 0;
    audio_ = 0;
    sequenceNumberFIR_ = 0;
    bundle_ = false;
    this->setVideoSinkSSRC(55543);
    this->setAudioSinkSSRC(44444);
    videoSink_ = NULL;
    audioSink_ = NULL;
    fbSink_ = NULL;
    sourcefbSink_ = this;
    sinkfbSource_ = this;

    globalState_ = INITIAL;
    connStateListener_ = NULL;

    sending_ = true;
    send_Thread_ = boost::thread(&WebRtcConnection::sendLoop, this);

    videoTransport_ = NULL;
    audioTransport_ = NULL;

    audioEnabled_ = audioEnabled;
    videoEnabled_ = videoEnabled;

    stunServer_ = stunServer;
    stunPort_ = stunPort;
    minPort_ = minPort;
    maxPort_ = maxPort;
//    maxVideoBitRate_ = 128000;

//    avgBitrate = 0;
//    lastAvgBitrate = 0;
//    lastWarmupTime_ = 0;
//    successRate = 0;
  }

  WebRtcConnection::~WebRtcConnection() {
    ELOG_DEBUG("WebRtcConnection Destructor");
    videoSink_ = NULL;
    audioSink_ = NULL;
    fbSink_ = NULL;
    delete videoTransport_;
    videoTransport_=NULL;
    delete audioTransport_;
    audioTransport_= NULL;
    sending_ = false;
    cond_.notify_one();
    send_Thread_.join();
    boost::mutex::scoped_lock lock(receiveVideoMutex_);
  }

  bool WebRtcConnection::init() {
    return true;
  }
  
  bool WebRtcConnection::setRemoteSdp(const std::string &sdp) {
    ELOG_DEBUG("Set Remote SDP %s", sdp.c_str());
    remoteSdp_.initWithSdp(sdp);
    //std::vector<CryptoInfo> crypto_remote = remoteSdp_.getCryptoInfos();
    video_ = (remoteSdp_.videoSsrc==0?false:true);
    audio_ = (remoteSdp_.audioSsrc==0?false:true);

    CryptoInfo cryptLocal_video;
    CryptoInfo cryptLocal_audio;
    CryptoInfo cryptRemote_video;
    CryptoInfo cryptRemote_audio;

    bundle_ = remoteSdp_.isBundle;
    ELOG_DEBUG("Is bundle? %d %d ", bundle_, true);
    std::vector<RtpMap> payloadRemote = remoteSdp_.getPayloadInfos();
    localSdp_.getPayloadInfos() = remoteSdp_.getPayloadInfos();
    localSdp_.isBundle = bundle_;
    localSdp_.isRtcpMux = remoteSdp_.isRtcpMux;

    ELOG_DEBUG("Video %d videossrc %u Audio %d audio ssrc %u Bundle %d", video_, remoteSdp_.videoSsrc, audio_, remoteSdp_.audioSsrc,  bundle_);

    ELOG_INFO("Setting SSRC to localSdp %u", this->getVideoSinkSSRC());
    localSdp_.videoSsrc = this->getVideoSinkSSRC();
    localSdp_.audioSsrc = this->getAudioSinkSSRC();

    this->setVideoSourceSSRC(remoteSdp_.videoSsrc);
    this->setAudioSourceSSRC(remoteSdp_.audioSsrc);

    if (remoteSdp_.profile == SAVPF) {
      if (remoteSdp_.isFingerprint) {
        // DTLS-SRTP
        if (remoteSdp_.hasVideo) {
          videoTransport_ = new DtlsTransport(VIDEO_TYPE, "video", bundle_, remoteSdp_.isRtcpMux, this, stunServer_, stunPort_, minPort_, maxPort_);
        }
        if (remoteSdp_.hasAudio) {
          audioTransport_ = new DtlsTransport(AUDIO_TYPE, "audio", bundle_, remoteSdp_.isRtcpMux, this, stunServer_, stunPort_, minPort_, maxPort_);
        }
      } else {
        // SDES
        std::vector<CryptoInfo> crypto_remote = remoteSdp_.getCryptoInfos();
        for (unsigned int it = 0; it < crypto_remote.size(); it++) {
          CryptoInfo cryptemp = crypto_remote[it];
          if (cryptemp.mediaType == VIDEO_TYPE
              && !cryptemp.cipherSuite.compare("AES_CM_128_HMAC_SHA1_80")) {
            videoTransport_ = new SdesTransport(VIDEO_TYPE, "video", bundle_, remoteSdp_.isRtcpMux, &cryptemp, this, stunServer_, stunPort_, minPort_, maxPort_);
          } else if (!bundle_ && cryptemp.mediaType == AUDIO_TYPE
              && !cryptemp.cipherSuite.compare("AES_CM_128_HMAC_SHA1_80")) {
            audioTransport_ = new SdesTransport(AUDIO_TYPE, "audio", bundle_, remoteSdp_.isRtcpMux, &cryptemp, this, stunServer_, stunPort_, minPort_, maxPort_);
          }
        }
      }
    }

    return true;
  }

  std::string WebRtcConnection::getLocalSdp() {
    ELOG_DEBUG("Getting SDP");
    if (videoTransport_ != NULL) {
      videoTransport_->processLocalSdp(&localSdp_);
    }
    ELOG_DEBUG("Video SDP done.");
    if (!bundle_ && audioTransport_ != NULL) {
      audioTransport_->processLocalSdp(&localSdp_);
    }
    ELOG_DEBUG("Audio SDP done.");
    localSdp_.profile = remoteSdp_.profile;
    return localSdp_.getSdp();
  }

  int WebRtcConnection::deliverAudioData(char* buf, int len) {
    writeSsrc(buf, len, this->getAudioSinkSSRC());
    if (bundle_){
      if (videoTransport_ != NULL) {
        if (audioEnabled_ == true) {
          this->queueData(0, buf, len, videoTransport_);
        }
      }
    } else if (audioTransport_ != NULL) {
      if (audioEnabled_ == true) {
        this->queueData(0, buf, len, audioTransport_);
      }
    }
    return len;
  }

  int WebRtcConnection::deliverVideoData(char* buf, int len) {
    rtpheader *head = (rtpheader*) buf;
    writeSsrc(buf, len, this->getVideoSinkSSRC());
    if (videoTransport_ != NULL) {
      if (videoEnabled_ == true) {

        if (head->payloadtype == RED_90000_PT) {
          int totalLength = 12;

          if (head->extension) {
            totalLength += ntohs(head->extensionlength)*4 + 4; // RTP Extension header
          }
          int rtpHeaderLength = totalLength;
          redheader *redhead = (redheader*) (buf + totalLength);

          //redhead->payloadtype = remoteSdp_.inOutPTMap[redhead->payloadtype];
          if (!remoteSdp_.supportPayloadType(head->payloadtype)) {
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
            buf = deliverMediaBuffer_;
            len = len - 1 - totalLength + rtpHeaderLength;
          }
        }

        this->queueData(0, buf, len, videoTransport_);
      }
    }
    return len;
  }

  int WebRtcConnection::deliverFeedback(char* buf, int len){
    // Check where to send the feedback
    rtcpheader *chead = reinterpret_cast<rtcpheader*> (buf);
    ELOG_DEBUG("received Feedback type %u ssrc %u, sourcessrc %u", chead->packettype, ntohl(chead->ssrc), ntohl(chead->ssrcsource));
    if (ntohl(chead->ssrcsource) == this->getAudioSourceSSRC()) {
        writeSsrc(buf,len,this->getAudioSinkSSRC());
    } else {
        writeSsrc(buf,len,this->getVideoSinkSSRC());
    }

    if (bundle_){
      if (videoTransport_ != NULL) {
        printRtcpHeaders(buf,len);
    	char* movingBuf = buf;
    	int rtcpLength = 0;
    	int totalLength = 0;
    	do{
    		movingBuf+=rtcpLength;
      		rtcpheader *chead= reinterpret_cast<rtcpheader*>(movingBuf);
      		rtcpLength= (ntohs(chead->length)+1)*4;
      		totalLength+= rtcpLength;

		if (chead->packettype != RTCP_Receiver_PT && chead->packettype != RTCP_PS_Feedback_PT && chead->packettype != RTCP_RTP_Feedback_PT)
	        	this->queueData(0, movingBuf, rtcpLength, videoTransport_);

	} while(totalLength<len);
      }
    } else {
      // TODO: Check where to send the feedback
      if (videoTransport_ != NULL) {
        this->queueData(0, buf, len, videoTransport_);
      }
    }
    return len;
  }

  void WebRtcConnection::writeSsrc(char* buf, int len, unsigned int ssrc) {
    rtpheader *head = reinterpret_cast<rtpheader*> (buf);
    rtcpheader *chead = reinterpret_cast<rtcpheader*> (buf);
    //if it is RTCP we check it it is a compound packet
    if (chead->isRtcp()) {
        processRtcpHeaders(buf,len,ssrc);
    } else {
      head->ssrc=htonl(ssrc);
    }
  }

  void WebRtcConnection::onTransportData(char* buf, int len, Transport *transport) {
    if (audioSink_ == NULL && videoSink_ == NULL && fbSink_==NULL)
      return;
    boost::mutex::scoped_lock lock(writeMutex_);
    int length = len;
    rtcpheader *chead = reinterpret_cast<rtcpheader*> (buf);
    if (chead->packettype == RTCP_Receiver_PT || chead->packettype == RTCP_PS_Feedback_PT || chead->packettype == RTCP_RTP_Feedback_PT){
      if (fbSink_ != NULL) {
        fbSink_->deliverFeedback(buf,length);
      }
    } else {
      // RTP or RTCP Sender Report
      if (bundle_) {

        // Check incoming SSRC
        rtpheader *head = reinterpret_cast<rtpheader*> (buf);
        rtcpheader *chead = reinterpret_cast<rtcpheader*> (buf);
        unsigned int recvSSRC = ntohl(head->ssrc);

        if (chead->packettype == RTCP_Sender_PT) { //Sender Report
          ELOG_DEBUG ("RTP Sender Report %d length %d ", chead->packettype, ntohs(chead->length));
          recvSSRC = ntohl(chead->ssrc);
//          this->resendRR(buf, len);
        }

        // Deliver data
        if (recvSSRC==this->getVideoSourceSSRC() || recvSSRC==this->getVideoSinkSSRC()) {
          videoSink_->deliverVideoData(buf, length);
        } else if (recvSSRC==this->getAudioSourceSSRC() || recvSSRC==this->getAudioSinkSSRC()) {
          audioSink_->deliverAudioData(buf, length);
        } else {
          ELOG_DEBUG("Unknown SSRC %u, localVideo %u, remoteVideo %u, ignoring", recvSSRC, this->getVideoSourceSSRC(), this->getVideoSinkSSRC());
        }
      } else if (transport->mediaType == AUDIO_TYPE) {
        if (audioSink_ != NULL) {
          rtpheader *head = (rtpheader*) buf;
          // Firefox does not send SSRC in SDP
          if (this->getAudioSourceSSRC() == 0) {
            ELOG_DEBUG("Audio Source SSRC is %u", ntohl(head->ssrc));
            this->setAudioSourceSSRC(ntohl(head->ssrc));
            //this->updateState(TRANSPORT_READY, transport);
          }
          head->ssrc = htonl(this->getAudioSinkSSRC());
          audioSink_->deliverAudioData(buf, length);
        }
      } else if (transport->mediaType == VIDEO_TYPE) {
        if (videoSink_ != NULL) {
          rtpheader *head = (rtpheader*) buf;
          // Firefox does not send SSRC in SDP
          if (this->getVideoSourceSSRC() == 0) {
            ELOG_DEBUG("Video Source SSRC is %u", ntohl(head->ssrc));
            this->setVideoSourceSSRC(ntohl(head->ssrc));
            //this->updateState(TRANSPORT_READY, transport);
          }

          head->ssrc = htonl(this->getVideoSinkSSRC());
          videoSink_->deliverVideoData(buf, length);
        }
      }
    }
  }

  int WebRtcConnection::resendRR(char *buf, int len) {
	ELOG_DEBUG("Mirroring SR PACKET");
    	int pos = 0;
    	uint8_t rtcpPacket[len];
	uint8_t RC = 1;

        rtcpPacket[pos++] = (uint8_t) 0x80 + RC;
    	rtcpPacket[pos++] = (uint8_t) 201;

    	//Length of 6 (single Block)
    	rtcpPacket[pos++] = (uint8_t) 0;
    	rtcpPacket[pos++] = (uint8_t) (7);

    	// Add our own SSRC
    	uint32_t* ptr = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
    	ptr[0] = htonl(this->getVideoSinkSSRC());
    	pos += 4;

	//SKIP 2+5 LINES OF BUF
        char *skipBuf = buf + 28;
        SRBlock *RRpkt = reinterpret_cast<SRBlock*> (skipBuf);
        SRheader *RRhead = reinterpret_cast<SRheader*> (buf);

	ELOG_DEBUG("PRINT SR RECEIVED");
        ELOG_DEBUG("packet RC %u", RRhead->blockcount);
        ELOG_DEBUG("packet type %u", RRhead->packettype);
        ELOG_DEBUG("packet length %u", RRhead->length);
        ELOG_DEBUG("packet ssrc %u", RRhead->ssrc);
        ELOG_DEBUG("packet ssrc_1 %u", RRpkt->ssrc);
        ELOG_DEBUG("packet fraction %u", RRpkt->fraction);
        ELOG_DEBUG("packet lost %u", RRpkt->lost);
        ELOG_DEBUG("packet lsr %u", RRpkt->lsr);
        ELOG_DEBUG("packet dlsr %u", RRpkt->dlsr);
        ELOG_DEBUG("packet jitter %u", RRpkt->jitter);

	ptr = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
    	ptr[0] = RRhead->ssrc;
    	pos += 4;

        rtcpPacket[pos++] = (uint8_t) 0;
        rtcpPacket[pos++] = (uint8_t) 0;
        rtcpPacket[pos++] = (uint8_t) 0;
        rtcpPacket[pos++] = (uint8_t) 0;

//        ptr = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
//        ptr[0] = (uint8_t) 0;
//        pos += 3;

        ptr = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
        ptr[0] = RRpkt->last_seq;
        pos += 4;

        ptr = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
        ptr[0] = RRpkt->jitter;
        pos += 4;

        rtcpPacket[pos++] = (uint8_t) 0;
        rtcpPacket[pos++] = (uint8_t) 0;
        rtcpPacket[pos++] = (uint8_t) 0;
        rtcpPacket[pos++] = (uint8_t) 0;

        rtcpPacket[pos++] = (uint8_t) 0;
        rtcpPacket[pos++] = (uint8_t) 0;
        rtcpPacket[pos++] = (uint8_t) 0;
        rtcpPacket[pos++] = (uint8_t) 0;

//        ptr = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
//        ptr[0] = RRpkt->lsr;
//        pos += 4;

//        ptr = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
//        ptr[0] = RRpkt->dlsr;
//        pos += 4;

        SRBlock *RRpkt2 = reinterpret_cast<SRBlock*> (rtcpPacket+8);
        SRheader *RRhead2 = reinterpret_cast<SRheader*> (rtcpPacket);


        ELOG_DEBUG("PRINT RR TRANSMITTED");
        ELOG_DEBUG("packet RC %u", RRhead2->blockcount);
        ELOG_DEBUG("packet type %u", RRhead2->packettype);
        ELOG_DEBUG("packet length %u", RRhead2->length);
        ELOG_DEBUG("packet ssrc %u", RRhead2->ssrc);
        ELOG_DEBUG("packet ssrc_1 %u", RRpkt2->ssrc);
        ELOG_DEBUG("packet fraction %u", RRpkt2->fraction);
        ELOG_DEBUG("packet lost %u", RRpkt2->lost);
        ELOG_DEBUG("packet lsr %u", RRpkt2->lsr);
        ELOG_DEBUG("packet dlsr %u", RRpkt2->dlsr);
        ELOG_DEBUG("packet jitter %u", RRpkt2->jitter);


    	if (videoTransport_ != NULL) {
      		videoTransport_->write((char*)rtcpPacket, pos);
    	}

    	return pos;
  }

  int WebRtcConnection::sendPliPacket() {
    ELOG_DEBUG("Generating PLI Packet");
    int pos = 0;
    uint8_t rtcpPacket[50];
    // add full intra request indicator
    uint8_t FMT = 1;
    rtcpPacket[pos++] = (uint8_t) 0x80 + FMT;
    rtcpPacket[pos++] = (uint8_t) 206; //192

    //Length of 4
    rtcpPacket[pos++] = (uint8_t) 0;
    rtcpPacket[pos++] = (uint8_t) (4);

    // Add our own SSRC
    uint32_t* ptr = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
    ptr[0] = htonl(this->getVideoSinkSSRC());
    pos += 4;

    if (videoTransport_ != NULL) {
      videoTransport_->write((char*)rtcpPacket, pos);
    }

    return pos;
  }

  int WebRtcConnection::sendFirPacket() {
    ELOG_DEBUG("Generating FIR Packet");
    sequenceNumberFIR_++; // do not increase if repetition
    int pos = 0;
    uint8_t rtcpPacket[50];
    // add full intra request indicator
    uint8_t FMT = 4;
    rtcpPacket[pos++] = (uint8_t) 0x80 + FMT;
    rtcpPacket[pos++] = (uint8_t) 206; //192

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

    ELOG_DEBUG("ssrc %zu, ssrcsource %zu",this->getVideoSinkSSRC(), this->getVideoSourceSSRC());

    // Additional Feedback Control Information (FCI)
    uint32_t* ptr2 = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
    ptr2[0] = htonl(this->getVideoSourceSSRC());
    pos += 4;

    rtcpPacket[pos++] = (uint8_t) (sequenceNumberFIR_);
    rtcpPacket[pos++] = (uint8_t) 0;
    rtcpPacket[pos++] = (uint8_t) 0;
    rtcpPacket[pos++] = (uint8_t) 0;

    if (videoTransport_ != NULL) {
      ELOG_DEBUG("trasmetto fir!!");
      videoTransport_->write((char*)rtcpPacket, pos);
    }

    return pos;
  }

  int WebRtcConnection::sendRembPacket(uint32_t bitrate) {
   ELOG_INFO("Generating WEBRTCCONN REMB Packet (bitrate=%zu)",bitrate);
   int pos = 0;
   uint8_t rtcpPacket[48];
   // add REMB (pt=206, fmt=15)
   uint8_t FMT = 15;
   rtcpPacket[pos++] = (uint8_t) 0x80 + FMT;
   rtcpPacket[pos++] = (uint8_t) 206;

   rtcpPacket[pos++] = (uint8_t) 0;
   rtcpPacket[pos++] = (uint8_t) (5);

   // Add our own SSRC
   uint32_t* ptr = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
   ptr[0] = htonl(this->getVideoSinkSSRC());
   pos += 4;

   rtcpPacket[pos++] = (uint8_t) 0;
   rtcpPacket[pos++] = (uint8_t) 0;
   rtcpPacket[pos++] = (uint8_t) 0;
   rtcpPacket[pos++] = (uint8_t) 0;

//   ELOG_INFO("REMB PACKET : 'R''E''M''B'");
   rtcpPacket[pos++] = 'R';
   rtcpPacket[pos++] = 'E';
   rtcpPacket[pos++] = 'M';
   rtcpPacket[pos++] = 'B';

//   ELOG_INFO("REMB PACKET : MANTISSA");
      rtcpPacket[pos++] = (uint8_t) 1;
      // 6 bit Exp
      // 18 bit mantissa
      uint8_t brExp = 0;
      for(uint32_t i=0; i<64; i++)
      {
          if(bitrate <= ((uint32_t)262143 << i))
          {
              brExp = i;
              break;
          }
      }
      const uint32_t brMantissa = (bitrate >> brExp);
      rtcpPacket[pos++]=(uint8_t)((brExp << 2) + ((brMantissa >> 16) & 0x03));
      rtcpPacket[pos++]=(uint8_t)(brMantissa >> 8);
      rtcpPacket[pos++]=(uint8_t)(brMantissa);

   uint32_t* ptr2 = reinterpret_cast<uint32_t*>(rtcpPacket + pos);
   ptr2[0] = htonl(this->getVideoSourceSSRC());
   pos += 4;

//   ELOG_INFO("REMB PACKET : VIDEOSSRC");

   if (videoTransport_ != NULL) {
    videoTransport_->write((char*)rtcpPacket, pos);
   }

   return pos;
  }

  void WebRtcConnection::setWebRTCConnectionStateListener(
      WebRtcConnectionStateListener* listener) {
    this->connStateListener_ = listener;
  }

  void WebRtcConnection::updateState(TransportState state, Transport * transport) {
    boost::mutex::scoped_lock lock(updateStateMutex_);
    WebRTCState temp = globalState_;
    ELOG_INFO("Update Transport State %s to %d", transport->transport_name.c_str(), state);
    if (audioTransport_ == NULL && videoTransport_ == NULL) {
      return;
    }

    if (state == TRANSPORT_FAILED) {
      temp = FAILED;
      ELOG_INFO("WebRtcConnection failed.");
    }

    
    if (globalState_ == FAILED) {
      // if current state is failed we don't use
      return;
    }

    if (state == TRANSPORT_STARTED &&
        (!remoteSdp_.hasAudio || (audioTransport_ != NULL && audioTransport_->getTransportState() == TRANSPORT_STARTED)) &&
        (!remoteSdp_.hasVideo || (videoTransport_ != NULL && videoTransport_->getTransportState() == TRANSPORT_STARTED))) {
      if (remoteSdp_.hasVideo) {
        videoTransport_->setRemoteCandidates(remoteSdp_.getCandidateInfos());
      }
      if (!bundle_ && remoteSdp_.hasAudio) {
        audioTransport_->setRemoteCandidates(remoteSdp_.getCandidateInfos());
      }
      temp = STARTED;
    }

    if (state == TRANSPORT_READY &&
        (!remoteSdp_.hasAudio || (audioTransport_ != NULL && audioTransport_->getTransportState() == TRANSPORT_READY)) &&
        (!remoteSdp_.hasVideo || (videoTransport_ != NULL && videoTransport_->getTransportState() == TRANSPORT_READY))) {
        // WebRTCConnection will be ready only when all channels are ready.
        temp = READY;
    }

    if (transport != NULL && transport == videoTransport_ && bundle_) {
      if (state == TRANSPORT_STARTED) {
        videoTransport_->setRemoteCandidates(remoteSdp_.getCandidateInfos());
        temp = STARTED;
      }
      if (state == TRANSPORT_READY) {
        temp = READY;
      }
    }

    if (temp == READY && globalState_ != temp) {
      ELOG_INFO("Ready to send and receive media");
//      this->sendRembPacket(maxVideoBitRate_);
    }

    if (audioTransport_ != NULL && videoTransport_ != NULL) {
      ELOG_INFO("%s - Update Transport State end, %d - %d, %d - %d, %d - %d", 
        transport->transport_name.c_str(),
        (int)audioTransport_->getTransportState(), 
        (int)videoTransport_->getTransportState(), 
        this->getAudioSourceSSRC(),
        this->getVideoSourceSSRC(),
        (int)temp, 
        (int)globalState_);
    }

    if (temp < 0) {
      return;
    }

    if (temp == globalState_ || (temp == STARTED && globalState_ == READY))
      return;

    globalState_ = temp;
    if (connStateListener_ != NULL)
      connStateListener_->connectionStateChanged(globalState_);
  }

  void WebRtcConnection::queueData(int comp, const char* buf, int length, Transport *transport) {
    if (audioSink_ == NULL && videoSink_ == NULL && fbSink_==NULL) //we don't enqueue data if there is nothing to receive it
      return;
    boost::mutex::scoped_lock lock(receiveVideoMutex_);
    if (sendQueue_.size() < 1000) {
      dataPacket p_;
      memset(p_.data, 0, length);
      memcpy(p_.data, buf, length);
      p_.comp = comp;
      if (transport->mediaType == VIDEO_TYPE) {
        p_.type = VIDEO_PACKET;
      } else {
        p_.type = AUDIO_PACKET;
      }

      p_.length = length;
      sendQueue_.push(p_);
    }
    cond_.notify_one();
  }

  WebRTCState WebRtcConnection::getCurrentState() {
    return globalState_;
  }

  void WebRtcConnection::printRtcpHeaders(char* buf, int len){
    char* movingBuf = buf;
    int rtcpLength = 0;
    int totalLength = 0;
    do{
      movingBuf+=rtcpLength;
      rtcpheader *chead= reinterpret_cast<rtcpheader*>(movingBuf);
      rtcpLength= (ntohs(chead->length)+1)*4;
      totalLength+= rtcpLength;
      firheader *thefir = reinterpret_cast<firheader*>(movingBuf);
      ELOG_DEBUG("Feedback PT %u - [fmt %d]", thefir->packettype, thefir->fmt);
//      if (thefir->packettype == 201) {
//        char *skipBuf = movingBuf + 8;
//        SRBlock *RRpkt = reinterpret_cast<SRBlock*> (skipBuf);
//        SRheader *RRhead = reinterpret_cast<SRheader*> (movingBuf);

//        ELOG_DEBUG("PRINT RR RECEIVED");
//        ELOG_DEBUG("packet RC %u", RRhead->blockcount);
//        ELOG_DEBUG("packet type %u", RRhead->packettype);
//        ELOG_DEBUG("packet length %u", RRhead->length);
//        ELOG_DEBUG("packet ssrc %u", RRhead->ssrc);
//        ELOG_DEBUG("packet ssrc_1 %u", RRpkt->ssrc);
//        ELOG_DEBUG("packet fraction %u", RRpkt->fraction);
//        ELOG_DEBUG("packet lost %u", RRpkt->lost);
//        ELOG_DEBUG("packet lsr %u", RRpkt->lsr);
//        ELOG_DEBUG("packet dlsr %u", RRpkt->dlsr);
//        ELOG_DEBUG("packet jitter %u", RRpkt->jitter);


//      }


    } while(totalLength<len);
 }

  void WebRtcConnection::processRtcpHeaders(char* buf, int len, unsigned int ssrc){
    char* movingBuf = buf;
    int rtcpLength = 0;
    int totalLength = 0;
    do{
      movingBuf+=rtcpLength;
      rtcpheader *chead= reinterpret_cast<rtcpheader*>(movingBuf);
      rtcpLength= (ntohs(chead->length)+1)*4;
      totalLength+= rtcpLength;
      chead->ssrc=htonl(ssrc);
      if (chead->packettype == RTCP_PS_Feedback_PT){
        firheader *thefir = reinterpret_cast<firheader*>(movingBuf);
        if (thefir->fmt == 4) {// It is a FIR Packet, we generate it
          ELOG_DEBUG("Feedback FIR packet, changed source %u - fmt %d", ssrc, thefir->fmt);
          this->sendFirPacket();
	} else if (thefir->fmt == 1) {
          ELOG_DEBUG("Feedback PLI packet, changed source %u - fmt %d", ssrc, thefir->fmt);
          this->sendFirPacket();
        } else if (thefir->fmt == 15) {//It is a REMB packet, we generate it
          ELOG_DEBUG("Feedback REMB packet, changed source %u - fmt %d", ssrc, thefir->fmt);
          this->sendRembPacket(300000);

//        } else if (thefir->fmt == 15) {
//          rembheader *rembpkt = reinterpret_cast<rembheader*>(movingBuf);
//          if (rembpkt->ssrcofmediasource!=0) {  //separa i Remb del recorder dagli RR di altri eventuali subscribers
//	          ELOG_DEBUG("Feedback REMB packet, changed source %u, previous %u - fmt %d", ssrc, rembpkt->ssrc, rembpkt->fmt);
//	          ELOG_DEBUG("bitrate %u", rembpkt->ssrcofmediasource);
//		  this->sendRembPacket(rembpkt->ssrcofmediasource);
//	  }
	}
      }
    } while(totalLength<len);
  }

  void WebRtcConnection::sendLoop() {

    while (sending_ == true) {

      boost::unique_lock<boost::mutex> lock(receiveVideoMutex_);
      while (sendQueue_.size() == 0) {
        cond_.wait(lock);
        if (sending_ == false) {
          lock.unlock();
          return;
        }
      }

      if (sendQueue_.front().type == VIDEO_PACKET || bundle_) {
        videoTransport_->write(sendQueue_.front().data, sendQueue_.front().length);
      } else {
        audioTransport_->write(sendQueue_.front().data, sendQueue_.front().length);
      }
      sendQueue_.pop();
      lock.unlock();
    }

  }
}
/* namespace erizo */
