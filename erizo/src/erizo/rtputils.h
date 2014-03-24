/*
 * rtputils.h
 */

#ifndef RTPUTILS_H_
#define RTPUTILS_H_

#include <netinet/in.h>

namespace erizo{
  // Payload types
#define RTCP_Sender_PT      200 // RTCP Sender Report
#define RTCP_Receiver_PT    201 // RTCP Receiver Report
#define RTCP_RTP_Feedback_PT 205 // RTCP Transport Layer Feedback Packet
#define RTCP_PS_Feedback_PT    206 // RTCP Payload Specific Feedback Packet

#define VP8_90000_PT        100 // VP8 Video Codec
#define RED_90000_PT        116 // REDundancy (RFC 2198)
#define ULP_90000_PT        117 // ULP/FEC
#define ISAC_16000_PT       103 // ISAC Audio Codec
#define ISAC_32000_PT       104 // ISAC Audio Codec
#define PCMU_8000_PT        0   // PCMU Audio Codec
#define OPUS_48000_PT       111 // Opus Audio Codec
#define PCMA_8000_PT        8   // PCMA Audio Codec
#define CN_8000_PT          13  // CN Audio Codec
#define CN_16000_PT         105 // CN Audio Codec
#define CN_32000_PT         106 // CN Audio Codec
#define CN_48000_PT         107 // CN Audio Codec
#define TEL_8000_PT         126 // Tel Audio Events


//HELPERS
#define RTP_SEQ_MOD (1<<16)

  //    0                   1                   2                   3
  //    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //   |V=2|P|X|  CC   |M|     PT      |       sequence number         |
  //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //   |                           timestamp                           |
  //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //   |           synchronization source (SSRC) identifier            |
  //   +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
  //   |            contributing source (CSRC) identifiers             |
  //   |                             ....                              |
  //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

  //   The first twelve octets are present in every RTP packet, while the
  //   list of CSRC identifiers is present only when inserted by a mixer.
  //   The fields have the following meaning:

  //   version (V): 2 bits
  //        This field identifies the version of RTP. The version defined by
  //        this specification is two (2). (The value 1 is used by the first
  //        draft version of RTP and the value 0 is used by the protocol
  //        initially implemented in the "vat" audio tool.)

  //   padding (P): 1 bit
  //        If the padding bit is set, the packet contains one or more
  //        additional padding octets at the end which are not part of the
  //      payload. The last octet of the padding contains a count of how
  //      many padding octets should be ignored. Padding may be needed by
  //      some encryption algorithms with fixed block sizes or for
  //      carrying several RTP packets in a lower-layer protocol data
  //      unit.

  // extension (X): 1 bit
  //      If the extension bit is set, the fixed header is followed by
  //      exactly one header extension, with a format defined in Section
  //      5.3.1.

  // CSRC count (CC): 4 bits
  //      The CSRC count contains the number of CSRC identifiers that
  //      follow the fixed header.

  // marker (M): 1 bit
  //      The interpretation of the marker is defined by a profile. It is
  //      intended to allow significant events such as frame boundaries to
  //      be marked in the packet stream. A profile may define additional
  //      marker bits or specify that there is no marker bit by changing
  //      the number of bits in the payload type field (see Section 5.3).

  // payload type (PT): 7 bits
  //      This field identifies the format of the RTP payload and
  //      determines its interpretation by the application. A profile
  //      specifies a default static mapping of payload type codes to
  //      payload formats. Additional payload type codes may be defined
  //      dynamically through non-RTP means (see Section 3). An initial
  //      set of default mappings for audio and video is specified in the
  //      companion profile Internet-Draft draft-ietf-avt-profile, and
  //      may be extended in future editions of the Assigned Numbers RFC
  //      [6].  An RTP sender emits a single RTP payload type at any given
  //      time; this field is not intended for multiplexing separate media
  //      streams (see Section 5.2).

  // sequence number: 16 bits
  //      The sequence number increments by one for each RTP data packet
  //      sent, and may be used by the receiver to detect packet loss and
  //      to restore packet sequence. The initial value of the sequence
  //      number is random (unpredictable) to make known-plaintext attacks
  //      on encryption more difficult, even if the source itself does not
  //      encrypt, because the packets may flow through a translator that
  //      does. Techniques for choosing unpredictable numbers are
  //      discussed in [7].

  // timestamp: 32 bits
  //      The timestamp reflects the sampling instant of the first octet
  //      in the RTP data packet. The sampling instant must be derived
  //      from a clock that increments monotonically and linearly in time
  //      to allow synchronization and jitter calculations (see Section
  //      6.3.1).  The resolution of the clock must be sufficient for the
  //      desired synchronization accuracy and for measuring packet
  //      arrival jitter (one tick per video frame is typically not
  //      sufficient).  The clock frequency is dependent on the format of
  //      data carried as payload and is specified statically in the
  //      profile or payload format specification that defines the format,
  //      or may be specified dynamically for payload formats defined
  //      through non-RTP means. If RTP packets are generated
  //      periodically, the nominal sampling instant as determined from
  //      the sampling clock is to be used, not a reading of the system
  //      clock. As an example, for fixed-rate audio the timestamp clock
  //      would likely increment by one for each sampling period.  If an
  //      audio application reads blocks covering 160 sampling periods
  //      from the input device, the timestamp would be increased by 160
  //      for each such block, regardless of whether the block is
  //      transmitted in a packet or dropped as silent.

  // The initial value of the timestamp is random, as for the sequence
  // number. Several consecutive RTP packets may have equal timestamps if
  // they are (logically) generated at once, e.g., belong to the same
  // video frame. Consecutive RTP packets may contain timestamps that are
  // not monotonic if the data is not transmitted in the order it was
  // sampled, as in the case of MPEG interpolated video frames. (The
  // sequence numbers of the packets as transmitted will still be
  // monotonic.)

  // SSRC: 32 bits
  //      The SSRC field identifies the synchronization source. This
  //      identifier is chosen randomly, with the intent that no two
  //      synchronization sources within the same RTP session will have
  //      the same SSRC identifier. An example algorithm for generating a
  //      random identifier is presented in Appendix A.6. Although the
  //      probability of multiple sources choosing the same identifier is
  //      low, all RTP implementations must be prepared to detect and
  //      resolve collisions.  Section 8 describes the probability of
  //      collision along with a mechanism for resolving collisions and
  //      detecting RTP-level forwarding loops based on the uniqueness of
  //      the SSRC identifier. If a source changes its source transport
  //      address, it must also choose a new SSRC identifier to avoid
  //      being interpreted as a looped source.

  // 0                   1                   2                   3
  //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |      defined by profile       |           length              |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |                        header extension                       |
  // |                             ....                              |

  typedef struct {
    uint32_t cc :4;
    uint32_t extension :1;
    uint32_t padding :1;
    uint32_t version :2;
    uint32_t payloadtype :7;
    uint32_t marker :1;
    uint32_t seqnum :16;
    uint32_t timestamp;
    uint32_t ssrc;
    uint32_t extensionpayload:16;
    uint32_t extensionlength:16;
  } rtpheader;

  //  0                   1                   2                   3
  //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |V=2|P|    RC   |   PT=RR=201   |             length            | header
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |                     SSRC of packet sender                     |
  // +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
  // |                 SSRC_1 (SSRC of first source)                 | report
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ block
  // | fraction lost |       cumulative number of packets lost       |   1
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |           extended highest sequence number received           |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |                      interarrival jitter                      |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |                         last SR (LSR)                         |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |                   delay since last SR (DLSR)                  |
  // +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
  // |                 SSRC_2 (SSRC of second source)                | report
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ block
  // :                               ...                             :   2
  // +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
  // |                  profile-specific extensions                  |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

  typedef struct {
    uint32_t blockcount :5;
    uint32_t padding :1;
    uint32_t version :2;
    uint32_t packettype :8;
    uint32_t length :16;
    uint32_t ssrc;
    uint32_t ssrcsource;
    uint32_t fractionlost:8;
    bool isRtcp(){        
      if (packettype == RTCP_Sender_PT || 
          packettype == RTCP_Receiver_PT || 
          packettype == RTCP_PS_Feedback_PT||
          packettype == RTCP_RTP_Feedback_PT){
        return true;
      }
      return false;
    }
  } rtcpheader;


  //    0                   1                   2                   3
  //    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //   |V=2|P|   FMT   |       PT      |          length               |
  //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //   |                  SSRC of packet sender                        |
  //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //   |                  SSRC of media source                         |
  //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //   :            Feedback Control Information (FCI)                 :
  //   :                                                               :

  //   The Feedback Control Information (FCI) for the Full Intra Request
  //   consists of one or more FCI entries, the content of which is depicted
  //   in Figure 4.  The length of the FIR feedback message MUST be set to
  //   2+2*N, where N is the number of FCI entries.
  //
  //    0                   1                   2                   3
  //    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //   |                              SSRC                             |
  //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //   | Seq nr.       |    Reserved                                   |
  //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+


  typedef struct {
    uint32_t fmt :5;
    uint32_t padding :1;
    uint32_t version :2;
    uint32_t packettype :8;
    uint32_t length :16;
    uint32_t ssrc;
    uint32_t ssrcofmediasource;
    uint32_t ssrc_fir;
  } firheader;


  //     0                   1                    2                   3
  //     0 1 2 3 4 5 6 7 8 9 0 1 2 3  4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //    |F|   block PT  |  timestamp offset         |   block length    |
  //    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //
  //
  // RFC 2198          RTP Payload for Redundant Audio Data    September 1997
  //
  //    The bits in the header are specified as follows:
  //
  //    F: 1 bit First bit in header indicates whether another header block
  //        follows.  If 1 further header blocks follow, if 0 this is the
  //        last header block.
  //        If 0 there is only 1 byte RED header
  //
  //    block PT: 7 bits RTP payload type for this block.
  //
  //    timestamp offset:  14 bits Unsigned offset of timestamp of this block
  //        relative to timestamp given in RTP header.  The use of an unsigned
  //        offset implies that redundant data must be sent after the primary
  //        data, and is hence a time to be subtracted from the current
  //        timestamp to determine the timestamp of the data for which this
  //        block is the redundancy.
  //
  //    block length:  10 bits Length in bytes of the corresponding data
  //        block excluding header.
  struct redheader {
    uint32_t payloadtype :7;
    uint32_t follow :1;
    uint32_t tsLength :24;
    uint32_t getTS() {
      return (ntohl(tsLength) & 0xfffc00) >> 10;
    }
    uint32_t getLength() {
      return (ntohl(tsLength) & 0x3ff);
    }
  };

//    0                   1                   2                   3
//    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//   |V=2|P| FMT=15  |   PT=206      |             length            |
//   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//   |                  SSRC of packet sender                        |
//   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//   |                  SSRC of media source                         |
//   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//   |  Unique identifier 'R' 'E' 'M' 'B'                            |
//   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//   |  Num SSRC     | BR Exp    |  BR Mantissa                      |
//   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//   |   SSRC feedback                                               |
//   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//   |  ...                                                          |
//
//   The fields V, P, SSRC, and length are defined in the RTP
//   specification [2], the respective meaning being summarized below:
//
//   version (V): (2 bits):   This field identifies the RTP version.  The
//               current version is 2.

//   padding (P) (1 bit):   If set, the padding bit indicates that the
//               packet contains additional padding octets at the end that
//               are not part of the control information but are included
//               in the length field.  Always 0.

//   Feedback message type (FMT) (5 bits):  This field identifies the type
//               of the FB message and is interpreted relative to the type
//               (transport layer, payload- specific, or application layer
//               feedback).  Always 15, application layer feedback
//               message.  RFC 4585 section 6.4.

//   Payload type (PT) (8 bits):   This is the RTCP packet type that
//               identifies the packet as being an RTCP FB message.
//               Always PSFB (206), Payload-specific FB message.  RFC 4585
//               section 6.4.

//   Length (16 bits):  The length of this packet in 32-bit words minus
//               one, including the header and any padding.  This is in
//               line with the definition of the length field used in RTCP
//               sender and receiver reports [3].  RFC 4585 section 6.4.

//   SSRC of packet sender (32 bits):  The synchronization source
//               identifier for the originator of this packet.  RFC 4585
//               section 6.4.

//   SSRC of media source (32 bits):  Always 0; this is the same
//               convention as in [RFC5104] section 4.2.2.2 (TMMBN).

//   Unique identifier (32 bits):  Always 'R' 'E' 'M' 'B' (4 ASCII
//               characters).

//   Num SSRC (8 bits):  Number of SSRCs in this message.

//   BR Exp (6 bits):   The exponential scaling of the mantissa for the
//               maximum total media bit rate value, ignoring all packet
//               overhead.  The value is an unsigned integer [0..63], as
//               in RFC 5104 section 4.2.2.1.

//   BR Mantissa (18 bits):   The mantissa of the maximum total media bit
//               rate (ignoring all packet overhead) that the sender of
//               the REMB estimates.  The BR is the estimate of the
//               traveled path for the SSRCs reported in this message.
//               The value is an unsigned integer in number of bits per
//               second.

//   SSRC feedback (32 bits)  Consists of one or more SSRC entries which
//               this feedback message applies to.
//
//
//
typedef struct {
    uint32_t fmt :5;
    uint32_t padding :1;
    uint32_t version :2;
    uint32_t packettype :8;
    uint32_t length :16;
    uint32_t ssrc;
    uint32_t ssrcofmediasource;
    uint32_t rembidentifier;
    uint32_t numssrc:8;
    uint32_t brexp:6;
    uint32_t brmantissa:18;
    uint32_t ssrcfeedback;
} rembheader;


//	 0                   1                   2                   3
//	 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//	|V=2|P|    RC   |   PT=SR=200   |             length            | header
//	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//	|                         SSRC of sender                        |
//	+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
//	|              NTP timestamp, most significant word             | sender
//	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ info
//	|             NTP timestamp, least significant word             |
//	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//	|                         RTP timestamp                         |
//	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//	|                     sender's packet count                     |
//	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//	|                      sender's octet count                     |
//	+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
//	|      	          SSRC_1 (SSRC of first source)                 | report
//	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ block
//	| fraction lost |       cumulative number of packets lost       |   1
//	-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//	|           extended highest sequence number received           |
//	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//	|                      interarrival jitter                      |
//	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//	|                         last SR (LSR)                         |
//	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//	|                   delay since last SR (DLSR)                  |
//	+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
//	|                 SSRC_2 (SSRC of second source)                | report
//	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ block
//	:                               ...                             :   2
//	+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
//	|                  profile-specific extensions                  |
//	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+



typedef struct {
    uint32_t blockcount :5;
    uint32_t padding :1;
    uint32_t version :2;
    uint32_t packettype :8;
    uint32_t length :16;
    uint32_t ssrc;
} SRheader;

typedef struct {
	uint32_t ntptsmoresig;
	uint32_t ntptslesssig;
	uint32_t rtptimestamp;
	uint32_t senderpktcount;
	uint32_t senderoctet;
} SRInfo;

typedef struct {
       uint32_t ssrc;             /* data source being reported */
       unsigned int fraction:8;  /* fraction lost since last SR/RR */
       int lost:24;              /* cumul. no. pkts lost (signed!) */
       uint32_t last_seq;         /* extended last seq. no. received */
       uint32_t jitter;           /* interarrival jitter */
       uint32_t lsr;              /* last SR packet from this source */
       uint32_t dlsr;             /* delay since last SR packet */
} SRBlock;



// PACKET RR
struct rtcp_rr_t {
       unsigned int version:2;   /* protocol version */
       unsigned int p:1;         /* padding flag */
       unsigned int count:5;     /* varies by packet type */
       unsigned int pt:8;        /* RTCP packet type */
       uint16_t length;           /* pkt len in words, w/o this word */
       uint32_t ssrcsender;     		 /* receiver generating this report */
       uint32_t ssrc;             /* data source being reported */
       unsigned int fraction:8;  /* fraction lost since last SR/RR */
       int lost:24;              /* cumul. no. pkts lost (signed!) */
       uint32_t last_seq;         /* extended last seq. no. received */
       uint32_t jitter;           /* interarrival jitter */
       uint32_t lsr;              /* last SR packet from this source */
       uint32_t dlsr;             /* delay since last SR packet */
};


struct sourcestat{
       uint16_t max_seq;        /* highest seq. number seen */
       uint32_t cycles;         /* shifted count of seq. number cycles */
       uint32_t base_seq;       /* base seq number */
       uint32_t bad_seq;        /* last 'bad' seq number + 1 */
       uint32_t probation;      /* sequ. packets till source is valid */
       uint32_t received;       /* packets received */
       uint32_t expected_prior; /* packet expected at last interval */
       uint32_t received_prior; /* packet received at last interval */
       uint32_t transit;        /* relative trans time for prev pkt */
       uint32_t jitter;         /* estimated jitter */
};


} /*namespace erizo*/

#endif /* RTPUTILS_H */
