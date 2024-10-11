/*
This node toggles the GPIO on the PI
GPIO 9, 12, 13
To control the Power source on the power distribution board.
*/


#include "gpio_manager.hpp"
#include <iostream> 
#include <string>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


GPIOManager::GPIOManager(std::string name) : Node(name)
{
    
    this->declare_parameter("device_name", m_device_name);
    this->get_parameter("device_name", m_device_name);

    this->declare_parameter("wiringpi_gpio", m_gpio_id);
    this->get_parameter("wiringpi_gpio", m_gpio_id);

    m_gpio_count = m_gpio_id.size();
    // printf("gpio size =%d\r\n", m_gpio_count);

    for (int i=0; i<m_gpio_count; i++)
    {
        gpio_t g;
        g.id = i;
        g.device_name = m_device_name[i];
        g.gpio_num = std::to_string(m_gpio_id[i]);
        g.service_name = "gpio_manager/set_power_" + g.device_name;
        printf("%s\r\n", g.service_name.c_str());

        

        g.m_set_gpio = this->create_service<std_srvs::srv::SetBool>(g.service_name,
                                    [this, i]
                                    (const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                    const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                                    this->f_cb_srv_set_power(request, response, i);
                                    });
        gpio_vector.push_back(g);
    }


    m_get_p_state = this->create_service<std_srvs::srv::Trigger>
    (
        "gpio_manager/get_power_status",
        std::bind(
            &GPIOManager::f_cb_srv_get_state,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );


    f_initialize_gpio();

}

bool GPIOManager::f_initialize_gpio()
{
    wiringPiSetup();
//     // for each gpio we do the following
    for (int i=0; i<m_gpio_count; i++)
    {
       pinMode(m_gpio_id[i],OUTPUT);
       digitalWrite(m_gpio_id[i],LOW);
       gpio_vector[i].state = 0;
    }

    return true;

}

bool GPIOManager::f_cb_srv_set_power(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            const std::shared_ptr<std_srvs::srv::SetBool::Response> response, int id)
{
    if(request->data)
    {
        digitalWrite(m_gpio_id[id], HIGH);
        // printf("%d\r\n",m_gpio_id[id]);
        gpio_vector[id].state = 1;
        response->success = 1;
        response->message = "GPIO-" + gpio_vector[id].gpio_num + " enabled, Device name:" + gpio_vector[id].device_name;
        return true;
    }
    else
    {
        digitalWrite(m_gpio_id[id], LOW);
        gpio_vector[id].state = 0;
        response->success = 1;
        response->message = "GPIO-" + gpio_vector[id].gpio_num + " disabled, Device name:" + gpio_vector[id].device_name;
        return true;
    }
    return true;
}
            
bool GPIOManager::f_cb_srv_get_state(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    //std::string msg = "#GPIO Manager#";
    std::string msg= "";

    for (int i=0; i<m_gpio_count; i++)
    {
        msg = msg + "gpio_manager/set_power_"+ gpio_vector[i].device_name + "=" + std::to_string(gpio_vector[i].state) + "\r\n" ;
    }
    response->success = 1;
    response->message = msg;
    return true;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<GPIOManager> node = std::make_shared<GPIOManager>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;

}

