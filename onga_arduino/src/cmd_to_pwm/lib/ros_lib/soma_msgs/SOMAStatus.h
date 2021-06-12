#ifndef _ROS_soma_msgs_SOMAStatus_h
#define _ROS_soma_msgs_SOMAStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace soma_msgs
{

  class SOMAStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint16_t _status_type;
      _status_type status;
      typedef const char* _status_str_type;
      _status_str_type status_str;
      typedef float _steering_pos_type;
      _steering_pos_type steering_pos;
      typedef float _steering_target_pos_type;
      _steering_target_pos_type steering_target_pos;
      typedef float _rear_pos_type;
      _rear_pos_type rear_pos;
      typedef float _rear_target_pos_type;
      _rear_target_pos_type rear_target_pos;
      typedef float _front_pos_type;
      _front_pos_type front_pos;
      typedef float _front_target_pos_type;
      _front_target_pos_type front_target_pos;
      typedef float _throttle_pos_type;
      _throttle_pos_type throttle_pos;
      typedef float _throttle_target_pos_type;
      _throttle_target_pos_type throttle_target_pos;
      typedef uint16_t _clutch_status_type;
      _clutch_status_type clutch_status;
      typedef const char* _clutch_status_str_type;
      _clutch_status_str_type clutch_status_str;
      typedef float _wheel_vel_type;
      _wheel_vel_type wheel_vel;
      typedef float _target_vel_type;
      _target_vel_type target_vel;
      uint32_t vel_errors_length;
      typedef float _vel_errors_type;
      _vel_errors_type st_vel_errors;
      _vel_errors_type * vel_errors;
      typedef float _PGain_type;
      _PGain_type PGain;
      typedef float _DGain_type;
      _DGain_type DGain;

    SOMAStatus():
      header(),
      status(0),
      status_str(""),
      steering_pos(0),
      steering_target_pos(0),
      rear_pos(0),
      rear_target_pos(0),
      front_pos(0),
      front_target_pos(0),
      throttle_pos(0),
      throttle_target_pos(0),
      clutch_status(0),
      clutch_status_str(""),
      wheel_vel(0),
      target_vel(0),
      vel_errors_length(0), vel_errors(NULL),
      PGain(0),
      DGain(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->status >> (8 * 1)) & 0xFF;
      offset += sizeof(this->status);
      uint32_t length_status_str = strlen(this->status_str);
      varToArr(outbuffer + offset, length_status_str);
      offset += 4;
      memcpy(outbuffer + offset, this->status_str, length_status_str);
      offset += length_status_str;
      union {
        float real;
        uint32_t base;
      } u_steering_pos;
      u_steering_pos.real = this->steering_pos;
      *(outbuffer + offset + 0) = (u_steering_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steering_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steering_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steering_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->steering_pos);
      union {
        float real;
        uint32_t base;
      } u_steering_target_pos;
      u_steering_target_pos.real = this->steering_target_pos;
      *(outbuffer + offset + 0) = (u_steering_target_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steering_target_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steering_target_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steering_target_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->steering_target_pos);
      union {
        float real;
        uint32_t base;
      } u_rear_pos;
      u_rear_pos.real = this->rear_pos;
      *(outbuffer + offset + 0) = (u_rear_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rear_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rear_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rear_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rear_pos);
      union {
        float real;
        uint32_t base;
      } u_rear_target_pos;
      u_rear_target_pos.real = this->rear_target_pos;
      *(outbuffer + offset + 0) = (u_rear_target_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rear_target_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rear_target_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rear_target_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rear_target_pos);
      union {
        float real;
        uint32_t base;
      } u_front_pos;
      u_front_pos.real = this->front_pos;
      *(outbuffer + offset + 0) = (u_front_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_front_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_front_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_front_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->front_pos);
      union {
        float real;
        uint32_t base;
      } u_front_target_pos;
      u_front_target_pos.real = this->front_target_pos;
      *(outbuffer + offset + 0) = (u_front_target_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_front_target_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_front_target_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_front_target_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->front_target_pos);
      union {
        float real;
        uint32_t base;
      } u_throttle_pos;
      u_throttle_pos.real = this->throttle_pos;
      *(outbuffer + offset + 0) = (u_throttle_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_throttle_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_throttle_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_throttle_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->throttle_pos);
      union {
        float real;
        uint32_t base;
      } u_throttle_target_pos;
      u_throttle_target_pos.real = this->throttle_target_pos;
      *(outbuffer + offset + 0) = (u_throttle_target_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_throttle_target_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_throttle_target_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_throttle_target_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->throttle_target_pos);
      *(outbuffer + offset + 0) = (this->clutch_status >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->clutch_status >> (8 * 1)) & 0xFF;
      offset += sizeof(this->clutch_status);
      uint32_t length_clutch_status_str = strlen(this->clutch_status_str);
      varToArr(outbuffer + offset, length_clutch_status_str);
      offset += 4;
      memcpy(outbuffer + offset, this->clutch_status_str, length_clutch_status_str);
      offset += length_clutch_status_str;
      union {
        float real;
        uint32_t base;
      } u_wheel_vel;
      u_wheel_vel.real = this->wheel_vel;
      *(outbuffer + offset + 0) = (u_wheel_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_vel);
      union {
        float real;
        uint32_t base;
      } u_target_vel;
      u_target_vel.real = this->target_vel;
      *(outbuffer + offset + 0) = (u_target_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_vel);
      *(outbuffer + offset + 0) = (this->vel_errors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vel_errors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vel_errors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vel_errors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel_errors_length);
      for( uint32_t i = 0; i < vel_errors_length; i++){
      union {
        float real;
        uint32_t base;
      } u_vel_errorsi;
      u_vel_errorsi.real = this->vel_errors[i];
      *(outbuffer + offset + 0) = (u_vel_errorsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vel_errorsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vel_errorsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vel_errorsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel_errors[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_PGain;
      u_PGain.real = this->PGain;
      *(outbuffer + offset + 0) = (u_PGain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_PGain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_PGain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_PGain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->PGain);
      union {
        float real;
        uint32_t base;
      } u_DGain;
      u_DGain.real = this->DGain;
      *(outbuffer + offset + 0) = (u_DGain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_DGain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_DGain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_DGain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->DGain);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->status =  ((uint16_t) (*(inbuffer + offset)));
      this->status |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->status);
      uint32_t length_status_str;
      arrToVar(length_status_str, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status_str; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status_str-1]=0;
      this->status_str = (char *)(inbuffer + offset-1);
      offset += length_status_str;
      union {
        float real;
        uint32_t base;
      } u_steering_pos;
      u_steering_pos.base = 0;
      u_steering_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steering_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steering_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steering_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->steering_pos = u_steering_pos.real;
      offset += sizeof(this->steering_pos);
      union {
        float real;
        uint32_t base;
      } u_steering_target_pos;
      u_steering_target_pos.base = 0;
      u_steering_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steering_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steering_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steering_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->steering_target_pos = u_steering_target_pos.real;
      offset += sizeof(this->steering_target_pos);
      union {
        float real;
        uint32_t base;
      } u_rear_pos;
      u_rear_pos.base = 0;
      u_rear_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rear_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rear_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rear_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rear_pos = u_rear_pos.real;
      offset += sizeof(this->rear_pos);
      union {
        float real;
        uint32_t base;
      } u_rear_target_pos;
      u_rear_target_pos.base = 0;
      u_rear_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rear_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rear_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rear_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rear_target_pos = u_rear_target_pos.real;
      offset += sizeof(this->rear_target_pos);
      union {
        float real;
        uint32_t base;
      } u_front_pos;
      u_front_pos.base = 0;
      u_front_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_front_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_front_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_front_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->front_pos = u_front_pos.real;
      offset += sizeof(this->front_pos);
      union {
        float real;
        uint32_t base;
      } u_front_target_pos;
      u_front_target_pos.base = 0;
      u_front_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_front_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_front_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_front_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->front_target_pos = u_front_target_pos.real;
      offset += sizeof(this->front_target_pos);
      union {
        float real;
        uint32_t base;
      } u_throttle_pos;
      u_throttle_pos.base = 0;
      u_throttle_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_throttle_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_throttle_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_throttle_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->throttle_pos = u_throttle_pos.real;
      offset += sizeof(this->throttle_pos);
      union {
        float real;
        uint32_t base;
      } u_throttle_target_pos;
      u_throttle_target_pos.base = 0;
      u_throttle_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_throttle_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_throttle_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_throttle_target_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->throttle_target_pos = u_throttle_target_pos.real;
      offset += sizeof(this->throttle_target_pos);
      this->clutch_status =  ((uint16_t) (*(inbuffer + offset)));
      this->clutch_status |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->clutch_status);
      uint32_t length_clutch_status_str;
      arrToVar(length_clutch_status_str, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_clutch_status_str; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_clutch_status_str-1]=0;
      this->clutch_status_str = (char *)(inbuffer + offset-1);
      offset += length_clutch_status_str;
      union {
        float real;
        uint32_t base;
      } u_wheel_vel;
      u_wheel_vel.base = 0;
      u_wheel_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_vel = u_wheel_vel.real;
      offset += sizeof(this->wheel_vel);
      union {
        float real;
        uint32_t base;
      } u_target_vel;
      u_target_vel.base = 0;
      u_target_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_vel = u_target_vel.real;
      offset += sizeof(this->target_vel);
      uint32_t vel_errors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      vel_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      vel_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      vel_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->vel_errors_length);
      if(vel_errors_lengthT > vel_errors_length)
        this->vel_errors = (float*)realloc(this->vel_errors, vel_errors_lengthT * sizeof(float));
      vel_errors_length = vel_errors_lengthT;
      for( uint32_t i = 0; i < vel_errors_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_vel_errors;
      u_st_vel_errors.base = 0;
      u_st_vel_errors.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_vel_errors.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_vel_errors.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_vel_errors.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_vel_errors = u_st_vel_errors.real;
      offset += sizeof(this->st_vel_errors);
        memcpy( &(this->vel_errors[i]), &(this->st_vel_errors), sizeof(float));
      }
      union {
        float real;
        uint32_t base;
      } u_PGain;
      u_PGain.base = 0;
      u_PGain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_PGain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_PGain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_PGain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->PGain = u_PGain.real;
      offset += sizeof(this->PGain);
      union {
        float real;
        uint32_t base;
      } u_DGain;
      u_DGain.base = 0;
      u_DGain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_DGain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_DGain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_DGain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->DGain = u_DGain.real;
      offset += sizeof(this->DGain);
     return offset;
    }

    const char * getType(){ return "soma_msgs/SOMAStatus"; };
    const char * getMD5(){ return "fbf823439300032b876c633999689246"; };

  };

}
#endif
