
#include "mvp_drivers/pwm_driver.hpp"
#include "iostream"
#include "cstdio"
#include <chrono>
#include <functional>
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

PwmDriver::PwmDriver(std::string name) : Node(name)
{
    this->declare_parameter("config_file_name", "");
    this->get_parameter("config_file_name", m_config_file);

    f_load_configuration();
   
}


bool PwmDriver::f_load_configuration()
{
    YAML::Node map = YAML::LoadFile(m_config_file);
    printf("FIle_name=%s\r\n", m_config_file.c_str());
    //parse thruster settings
    if(map["thrusters"])
    {
        thruster_t tt;
        for(YAML::const_iterator it=map["thrusters"].begin();it != map["thrusters"].end(); ++it)
        {
            std::string thruster_name = it->first.as<std::string>();    
            
            printf("thruster=%s\r\n", thruster_name.c_str() );

            tt.channel = map["thrusters"][thruster_name]["channel"].as<int>();
            tt.max_pwm = map["thrusters"][thruster_name]["max_pwm"].as<int>();
            tt.min_pwm = map["thrusters"][thruster_name]["min_pwm"].as<int>();
            tt.topic_name = map["thrusters"][thruster_name]["topic_name"].as<std::string>();

            thrusters.push_back(tt);
        }
    }

    //parse servo settings
    if(map["servos"])
    {
        servo_t tt;

        for(YAML::const_iterator it=map["servos"].begin();it != map["servos"].end(); ++it)
        {
            std::string thruster_name = it->first.as<std::string>();    
            
            printf("servos=%s\r\n", thruster_name.c_str() );

            tt.channel = map["servos"][thruster_name]["channel"].as<int>();
            tt.max_pwm = map["servos"][thruster_name]["max_pwm"].as<int>();
            tt.min_pwm = map["servos"][thruster_name]["min_pwm"].as<int>();
            tt.topic_name = map["servos"][thruster_name]["topic_name"].as<std::string>();
            tt.joint_name = map["servos"][thruster_name]["joint_name"].as<std::string>();
            servos.push_back(tt);
        }
    }



}