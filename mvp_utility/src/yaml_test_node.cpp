#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"

class YamlTest : public rclcpp::Node
{
    public:
        YamlTest(std::string name = "yaml_test");
        void f_read_control_modes();
    
};

YamlTest::YamlTest(std::string name) : Node(name)
{

}


void YamlTest::f_read_control_modes()
{

    std::string filename  = "/home/mingxi/ros2_ws/src/mvp_core/mvp_utility/config/control.yaml";

    YAML::Node map = YAML::LoadFile(filename);

    if(map["control_modes"])
    {
        //parse control modes
        // printf("size = %d\r\n", map["control_modes"].size() );
        std::vector<std::string> modes;
        //iterate through control modes
        for(YAML::const_iterator it=map["control_modes"].begin();it != map["control_modes"].end(); ++it) 
        {
            std::string key = it->first.as<std::string>();       // <- key
            modes.push_back(key);
            
        }
        //loop through modes
        for (const auto &mode: modes) 
        {
            // std::string k = mode;
            // printf("name = %s\r\n", k.c_str());
            printf("mode_name = %s\r\n", mode.c_str() );
            //loop through DOF to create ros params
            for(YAML::const_iterator it=map["control_modes"][mode].begin();it != map["control_modes"][mode].end(); ++it) 
            {
                std::string param_name;
                std::string dof_name = it->first.as<std::string>();
                printf("    dof_name = %s\r\n", dof_name.c_str());
                //get PID values
                for(const auto& key : {"p", "i", "d", "i_max", "i_min"})
                {
                    param_name = "control_modes/" + mode + "/" + dof_name + "/" + key;
                    printf("         param = %s\r\n", param_name.c_str());
                    this->declare_parameter( param_name, map["control_modes"][mode][dof_name][key].as<float>() );
                }
            
            }
        }

    }

}

void f_generate_thrusters()
{


}


int main (int argc, char ** argv)
{
    rclcpp::init(argc, argv);
   
    std::shared_ptr<YamlTest> node = std::make_shared<YamlTest>();

    node->f_read_control_modes();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}