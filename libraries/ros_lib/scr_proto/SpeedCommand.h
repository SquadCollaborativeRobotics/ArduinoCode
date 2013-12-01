#ifndef _ROS_scr_proto_SpeedCommand_h
#define _ROS_scr_proto_SpeedCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace scr_proto
{

  class SpeedCommand : public ros::Msg
  {
    public:
      float left_motor_w;
      float right_motor_w;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      int32_t * val_left_motor_w = (int32_t *) &(this->left_motor_w);
      int32_t exp_left_motor_w = (((*val_left_motor_w)>>23)&255);
      if(exp_left_motor_w != 0)
        exp_left_motor_w += 1023-127;
      int32_t sig_left_motor_w = *val_left_motor_w;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_left_motor_w<<5) & 0xff;
      *(outbuffer + offset++) = (sig_left_motor_w>>3) & 0xff;
      *(outbuffer + offset++) = (sig_left_motor_w>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_left_motor_w<<4) & 0xF0) | ((sig_left_motor_w>>19)&0x0F);
      *(outbuffer + offset++) = (exp_left_motor_w>>4) & 0x7F;
      if(this->left_motor_w < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_right_motor_w = (int32_t *) &(this->right_motor_w);
      int32_t exp_right_motor_w = (((*val_right_motor_w)>>23)&255);
      if(exp_right_motor_w != 0)
        exp_right_motor_w += 1023-127;
      int32_t sig_right_motor_w = *val_right_motor_w;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_right_motor_w<<5) & 0xff;
      *(outbuffer + offset++) = (sig_right_motor_w>>3) & 0xff;
      *(outbuffer + offset++) = (sig_right_motor_w>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_right_motor_w<<4) & 0xF0) | ((sig_right_motor_w>>19)&0x0F);
      *(outbuffer + offset++) = (exp_right_motor_w>>4) & 0x7F;
      if(this->right_motor_w < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t * val_left_motor_w = (uint32_t*) &(this->left_motor_w);
      offset += 3;
      *val_left_motor_w = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_left_motor_w |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_left_motor_w |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_left_motor_w |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_left_motor_w = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_left_motor_w |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_left_motor_w !=0)
        *val_left_motor_w |= ((exp_left_motor_w)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->left_motor_w = -this->left_motor_w;
      uint32_t * val_right_motor_w = (uint32_t*) &(this->right_motor_w);
      offset += 3;
      *val_right_motor_w = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_right_motor_w |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_right_motor_w |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_right_motor_w |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_right_motor_w = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_right_motor_w |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_right_motor_w !=0)
        *val_right_motor_w |= ((exp_right_motor_w)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->right_motor_w = -this->right_motor_w;
     return offset;
    }

    const char * getType(){ return "scr_proto/SpeedCommand"; };
    const char * getMD5(){ return "e23f0995ca625a7a50820ad5a3547921"; };

  };

}
#endif