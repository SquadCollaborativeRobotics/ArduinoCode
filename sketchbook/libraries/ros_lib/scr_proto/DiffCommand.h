#ifndef _ROS_scr_proto_DiffCommand_h
#define _ROS_scr_proto_DiffCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace scr_proto
{

  class DiffCommand : public ros::Msg
  {
    public:
      int64_t left_motor;
      int64_t right_motor;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_left_motor;
      u_left_motor.real = this->left_motor;
      *(outbuffer + offset + 0) = (u_left_motor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_motor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_motor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_motor.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_left_motor.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_left_motor.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_left_motor.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_left_motor.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->left_motor);
      union {
        int64_t real;
        uint64_t base;
      } u_right_motor;
      u_right_motor.real = this->right_motor;
      *(outbuffer + offset + 0) = (u_right_motor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_motor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_motor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_motor.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_right_motor.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_right_motor.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_right_motor.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_right_motor.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->right_motor);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_left_motor;
      u_left_motor.base = 0;
      u_left_motor.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_motor.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_motor.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_motor.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_left_motor.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_left_motor.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_left_motor.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_left_motor.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->left_motor = u_left_motor.real;
      offset += sizeof(this->left_motor);
      union {
        int64_t real;
        uint64_t base;
      } u_right_motor;
      u_right_motor.base = 0;
      u_right_motor.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_motor.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_motor.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_motor.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_right_motor.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_right_motor.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_right_motor.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_right_motor.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->right_motor = u_right_motor.real;
      offset += sizeof(this->right_motor);
     return offset;
    }

    const char * getType(){ return "scr_proto/DiffCommand"; };
    const char * getMD5(){ return "c6bda84704b740f39f2489eca53cdc9a"; };

  };

}
#endif