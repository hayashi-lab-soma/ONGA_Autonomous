#ifndef _ROS_SERVICE_GetGraphBasedPlan_h
#define _ROS_SERVICE_GetGraphBasedPlan_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_msgs/Path.h"

namespace soma_msgs
{

static const char GETGRAPHBASEDPLAN[] = "soma_msgs/GetGraphBasedPlan";

  class GetGraphBasedPlanRequest : public ros::Msg
  {
    public:

    GetGraphBasedPlanRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETGRAPHBASEDPLAN; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetGraphBasedPlanResponse : public ros::Msg
  {
    public:
      typedef nav_msgs::Path _path_type;
      _path_type path;

    GetGraphBasedPlanResponse():
      path()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->path.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->path.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETGRAPHBASEDPLAN; };
    const char * getMD5(){ return "58d6f138c7de7ef47c75d4b7e5df5472"; };

  };

  class GetGraphBasedPlan {
    public:
    typedef GetGraphBasedPlanRequest Request;
    typedef GetGraphBasedPlanResponse Response;
  };

}
#endif
