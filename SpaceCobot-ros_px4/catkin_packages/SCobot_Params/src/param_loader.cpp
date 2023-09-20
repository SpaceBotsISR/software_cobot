//
// Created by filiperosa on 3/8/18.
//
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include "SCobotPipeCommunication.h"
#include <string>

int paramPipe_fd;
const char *paramPipe_path="/home/erle/param_fifo";

void loadROSParameters(ros::NodeHandle* , const char*);
void loadParamVector(ros::NodeHandle*, char*);
std::string GetStdoutFromCommand(std::string);
std::string parseParamName(std::string);


std::map<std::string,float> last_param_values;

int main(int argc, char **argv)
{
    paramPipe_fd = 0;
    setupPipeProducer(paramPipe_path,&paramPipe_fd,false);

    ros::init(argc, argv, "param_manager");

    ros::NodeHandle nh;

    ros::Rate loop_rate(1);

    while(ros::ok()){
        loadROSParameters(&nh, "scobot");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

/**
 * Iterate through parameters and write them to the pipe
 *
 * @param nh - node handler
 * @param param_list - string with all ROS Params
 * @param keyword - filter or namespace to select only desired parameters
 */
void loadROSParameters(ros::NodeHandle* nh, const char* keyword){

    std::vector<std::string> param_vector;
    nh->getParamNames(param_vector);
    ///iterate all parameters
    for (std::vector<std::string>::iterator it = param_vector.begin() ; it != param_vector.end(); ++it){
        std::string param_path = *it;
        if(param_path.find(keyword) != std::string::npos){
            //get ROS Parameter
            float param_value = 0.0;
            nh->getParam(param_path, param_value);

            //Check if value changed to decide wether to send it or not
            std::map<std::string,float>::iterator found = last_param_values.find(param_path);
            if(found == last_param_values.end() ) //not found -> insert it
                last_param_values.insert( std::pair<std::string,float>(param_path,param_value) );
            else if( found->second == param_value ) continue; //value did not change
            else last_param_values[param_path] = param_value; //update value

            //Create a new pipe message
            ParamMsg msg;
            strcpy(msg.name, parseParamName(param_path).c_str());
            //msg.type=FLOAT;
            msg.f_value=param_value;

            if( write( paramPipe_fd, &msg , sizeof(ParamMsg) ) < 0 ) {
                ROS_INFO("FIFO ERROR sending parameter");
                //return;
            }
            else ROS_INFO("param name: %s   value: %f ", msg.name, msg.f_value);
            //usleep(10000);
        }
    }
}


void loadParamVector(ros::NodeHandle* nh, char* param_name){
    XmlRpc::XmlRpcValue my_list;
    nh->getParam(param_name, my_list);
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < my_list.size(); ++i)
    {
        ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        //sum += static_cast<double>(my_list[i]);
        ROS_INFO("value: %f",static_cast<double>(my_list[i]));

        ParamMsg msg;
        //msg.type = FLOAT;
        msg.f_value = static_cast<double>(my_list[i]);

        if( write( paramPipe_fd, &msg , sizeof(ParamMsg) ) < 0 ) {
            ROS_INFO("FIFO ERROR sending parameter");
            //return;
        }
    }
}

std::string GetStdoutFromCommand(std::string cmd) {

    std::string data;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");

    stream = popen(cmd.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
        pclose(stream);
    }
    return data;
}

std::string parseParamName(std::string str){
    size_t found;
    found=str.find_last_of("/");
    return str.substr(found+1);
}