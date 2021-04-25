#ifndef __DATA_PROTOCOL_H
#define __DATA_PROTOCOL_H


#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <cstdlib>

#define kang_num 50u

typedef struct
{
    uint8_t send_version;
    uint8_t send_speed;
}dt_flag_t;

class lykang
{
public:
    lykang()
    {

        aa = 0;    
    }
    ~lykang()
    {


    }

private:
    uint8_t aa;
    dt_flag_t bb;
};




#endif