#ifndef _ROS_cbbot_msgs_CBService_h
#define _ROS_cbbot_msgs_CBService_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cbbot_msgs
{

  class CBService : public ros::Msg
  {
    public:
      typedef uint8_t _cmd_type;
      _cmd_type cmd;
      typedef int32_t _ival_type;
      _ival_type ival;
      typedef float _fval_type;
      _fval_type fval;
      typedef const char* _sval_type;
      _sval_type sval;

    CBService():
      cmd(0),
      ival(0),
      fval(0),
      sval("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->cmd >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cmd);
      union {
        int32_t real;
        uint32_t base;
      } u_ival;
      u_ival.real = this->ival;
      *(outbuffer + offset + 0) = (u_ival.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ival.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ival.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ival.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ival);
      union {
        float real;
        uint32_t base;
      } u_fval;
      u_fval.real = this->fval;
      *(outbuffer + offset + 0) = (u_fval.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fval.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fval.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fval.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fval);
      uint32_t length_sval = strlen(this->sval);
      varToArr(outbuffer + offset, length_sval);
      offset += 4;
      memcpy(outbuffer + offset, this->sval, length_sval);
      offset += length_sval;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->cmd =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cmd);
      union {
        int32_t real;
        uint32_t base;
      } u_ival;
      u_ival.base = 0;
      u_ival.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ival.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ival.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ival.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ival = u_ival.real;
      offset += sizeof(this->ival);
      union {
        float real;
        uint32_t base;
      } u_fval;
      u_fval.base = 0;
      u_fval.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fval.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fval.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fval.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fval = u_fval.real;
      offset += sizeof(this->fval);
      uint32_t length_sval;
      arrToVar(length_sval, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sval; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sval-1]=0;
      this->sval = (char *)(inbuffer + offset-1);
      offset += length_sval;
     return offset;
    }

    const char * getType(){ return "cbbot_msgs/CBService"; };
    const char * getMD5(){ return "2419214c08aec8ea68ff98eb11b1cab2"; };

  };

}
#endif