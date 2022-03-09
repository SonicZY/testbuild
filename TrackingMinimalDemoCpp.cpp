#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

#include <Antilatency.InterfaceContract.LibraryLoader.h>
#include <Antilatency.DeviceNetwork.h>
#include "CommandRecv.h"

#if defined(__linux__)
#include <dlfcn.h>
#include <filesystem>

#include <configor/json.hpp>

#include <sys/socket.h>		// For socket()
#include <netinet/in.h> 	// For socket() and sockaddr_in struct */
#include <netinet/udp.h>	// For socket()
#include <unistd.h>			// For close()
#include <arpa/inet.h>		// For inet_aton()
#include <fcntl.h>

#include <wiringPi.h>


using namespace configor;
using namespace std;

#endif

#define _USE_MATH_DEFINES
#include <cmath>

#include <thread>
#include <chrono>
// to do list : NFC function( auto ip address )\ command ack \ LED lightting \  keys 


#if defined(__linux__)
int pin_Trigger = 7;
int pin_Switch = 8;
int pin_Reload = 9;
int pin_F1 = 10;
int pin_F2 = 11;
#endif


double setPrecise5(float value) {

	stringstream sstreamx;
	double P5value = 0;
	char str[10];

	sprintf(str, "%0.5f", value);

	sstreamx << atof(str) << endl;
	sstreamx >> P5value;

	return P5value;


}



Antilatency::DeviceNetwork::NodeHandle getIdleTrackingNode(Antilatency::DeviceNetwork::INetwork network, Antilatency::Alt::Tracking::ITrackingCotaskConstructor altTrackingCotaskConstructor,std::string Tagname) {
    // Get all currently connected nodes, that supports alt tracking task.
    std::vector<Antilatency::DeviceNetwork::NodeHandle> altNodes = altTrackingCotaskConstructor.findSupportedNodes(network);
    if (altNodes.size() == 0) {
        std::cout << "No nodes with Alt Tracking Task support found" << std::endl;
        return Antilatency::DeviceNetwork::NodeHandle::Null;
    }

    // Return first idle node.
    for (auto node : altNodes) {
        if (network.nodeGetStatus(node) == Antilatency::DeviceNetwork::NodeStatus::Idle) {

            std::cout << "nodes with Alt Tracking Task support found" << std::endl;
           
            auto soket = network.nodeGetParent(node);
            if (soket != Antilatency::DeviceNetwork::NodeHandle::Null)
            {
                auto tagname = network.nodeGetStringProperty(soket, "Tag");
                if (Tagname == tagname)
                {
                    std::cout << tagname << ": nodes with alt tracking task support found" << std::endl;
                    return node;
                }
            }
        }
    }

    std::cout << "No taget nodes with Alt Tracking Task support found" << std::endl;
    return Antilatency::DeviceNetwork::NodeHandle::Null;
}

float round(float src, int bits) {
    std::stringstream ss;
    float fixed;
    ss << std::setiosflags(std::ios::fixed) << std::setprecision(bits) << src;
    ss >> fixed;
    return fixed;
}


static void setnonblocking(int sockfd) {
    int flag = fcntl(sockfd, F_GETFL, 0);
    if (flag < 0) {
        perror("fcntl F_GETFL fail");
        return;
    }
    if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) {
        perror("fcntl F_SETFL fail");
    }
}

int main_task(int n){



#if defined(__linux__)
    //json sendMessage;
    json sendMessage_gun;

    

    //init the socket
    int client_sockfd, ret, on;
    int len;
    struct sockaddr_in remote_addr; 
    struct sockaddr_in remote_addr_gun; 
    int sin_size;
    char buf[BUFSIZ];  

    memset(&remote_addr, 0, sizeof(remote_addr)); 
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr("192.168.1.223");
    remote_addr.sin_port = htons(1999); 

    memset(&remote_addr_gun, 0, sizeof(remote_addr_gun)); 
    remote_addr_gun.sin_family = AF_INET; 
    remote_addr_gun.sin_addr.s_addr = inet_addr("192.168.1.169");
    remote_addr_gun.sin_port = htons(1998); 


    struct sockaddr_in local_addr; 
    memset(&local_addr, 0, sizeof(local_addr)); 
    local_addr.sin_family = AF_INET; 
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);//LocalIP
    local_addr.sin_port = htons(36362); //Local


    if ((client_sockfd = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creat error");
        return 1;
    }
    setnonblocking(client_sockfd);
    
    
    on = 1;
    ret = setsockopt(client_sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    if (bind(client_sockfd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        perror("socket bind error");
    }


    strcpy(buf, "Socket initalnaized,hello there\n"); 
    //sin_size=sizeof(struct sockaddr_in);
    
    if ((len = sendto(client_sockfd, buf, strlen(buf), 0, (struct sockaddr*)&remote_addr, sizeof(struct sockaddr))) < 0)
    {
        perror("socket error when send message");
        return 1;
    }


    int setupGpio = wiringPiSetup();

    if (0 == setupGpio) {
        pinMode(pin_Trigger, INPUT);
        pullUpDnControl(pin_Trigger, PUD_UP);

        pinMode(pin_Switch, INPUT);
        pullUpDnControl(pin_Switch, PUD_UP);

        pinMode(pin_Reload, INPUT);
        pullUpDnControl(pin_Reload, PUD_UP);

        pinMode(pin_F1, INPUT);
        pullUpDnControl(pin_F1, PUD_UP);

        pinMode(pin_F2, INPUT);
        pullUpDnControl(pin_F2, PUD_UP);
    }
#endif



    // Load the Antilatency Device Network library
    Antilatency::DeviceNetwork::ILibrary deviceNetworkLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::DeviceNetwork::ILibrary>(libNameADN.c_str());
    if (deviceNetworkLibrary == nullptr) {
        std::cout << "Failed to get Antilatency Device Network Library" << std::endl;
        return 1;
    }

    // Load the Antilatency Alt Tracking library
    Antilatency::Alt::Tracking::ILibrary altTrackingLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::Alt::Tracking::ILibrary>(libNameTracking.c_str());
    if (altTrackingLibrary == nullptr) {
        std::cout << "Failed to get Antilatency Alt Tracking Library" << std::endl;
        return 1;
    }

    // Load the Antilatency Alt Environment Selector library
    Antilatency::Alt::Environment::Selector::ILibrary environmentSelectorLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::Alt::Environment::Selector::ILibrary>(libNameEnvironmentSelector.c_str());
    if (environmentSelectorLibrary == nullptr) {
        std::cout << "Failed to get Antilatency Alt Environment Selector Library" << std::endl;
        return 1;
    }

    // Create a device network filter and then create a network using that filter.
    Antilatency::DeviceNetwork::IDeviceFilter filter = deviceNetworkLibrary.createFilter();
    filter.addUsbDevice(Antilatency::DeviceNetwork::Constants::AllUsbDevices);
    Antilatency::DeviceNetwork::INetwork network = deviceNetworkLibrary.createNetwork(filter);
    if (network == nullptr) {
        std::cout << "Failed to create Antilatency Device Network" << std::endl;
        return 1;
    }
    std::cout << "Antilatency Device Network created" << std::endl;

    // Get environment serialized data.
   // const std::string environmentData = argv[1];
    // Get placement serialized data.
   // const std::string placementData = argv[2];

     const std::string environmentData = "AAVSaWdpZMoDAQZ5ZWxsb3ckFbhTiT_cRqA-r45jvZqZmT4AAAAAAAAAAACamRk_iwEeDQMGFAAKFAAOFAASFAADFAIZFAAdFAAhFAAjEQEbBgIjCQEjBQEdAAIDAQICAAIJAAAHBAINAAIRAAIZAwIZAAIgAAAhBwMABgEeCgIADgEAEgEWFAIHEwAEEwIREwAXEwAbEwAfEwAMEwIJEgALEAENEQEQEgAFEgIZEQEWEgIfEgAcEgIiEgMCEAEIEAEEDwEEAgIAAwMCBAEaCwMCBwEGDQIACwMeAQIBDAMDDQIBCAMKBwMEBAEBBAMGAAIHAQAKAQAHAgIJAwMKBAAFCwIOAQEBEQMeAgIIDAMgAwAfBQAGBgIjAgMbAwMPBQIXAgIYBQAVAAIVAQASAQAiCAMWBQEUAwMUBwEVCgARCwMTDgIODQMQDQEGEAMHCAIECAMIBwMHCgIDCgIRBgMcEAIJDgEMBgAMCAAKCgMMAwMUBQIPBwASCAENCgIKDQMMDQEODwEQAgEgEAAeEAEcDgAaDgMgDAAhCwAgCgAiDgMcCwEcCAAfBwEdBQMZCAATDAMYCwMVDQMXCAIXDQEYDwEVEAAREAITEAEPCgMjDgOfQA";
    
      const std::string placementData = "AAAAAAAK1yO8BoGVPcK4sj0AAAAAAAAAAGY0";
      const std::string placementData_gun = "AAAAAADNzMw9m15wPsFX4D8AAAAA2w9JQA";
    // Create environment object from the serialized data.
    const Antilatency::Alt::Environment::IEnvironment environment = environmentSelectorLibrary.createEnvironment(environmentData);
    if (environment == nullptr) {
        std::cout << "Failed to create environment" << std::endl;
        return 1;
    }

    // Create placement from the serialized data.
    const Antilatency::Math::floatP3Q placement = altTrackingLibrary.createPlacement(placementData);
    const Antilatency::Math::floatP3Q placement_gun = altTrackingLibrary.createPlacement(placementData_gun);

    // Create alt tracking cotask constructor to find tracking-supported nodes and start tracking task on node.
    Antilatency::Alt::Tracking::ITrackingCotaskConstructor altTrackingCotaskConstructor = altTrackingLibrary.createTrackingCotaskConstructor();
    if (altTrackingCotaskConstructor == nullptr) {
        std::cout << "Failed to create Antilatency Alt Tracking Cotask Constructor" << std::endl;
        return 1;
    }

    // Each time the device network is changed due to connection or disconnection of a device that matches the device filter of the network,
    // or start or stop of a task on any network device, the network update id is incremented by 1. 
    uint32_t prevUpdateId = 0;
   
    bool blink = true;
    while (network != nullptr) {
        // Check if the network has been changed.
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(100));
        const uint32_t currentUpdateId = network.getUpdateId();

        if (prevUpdateId != currentUpdateId) {
            prevUpdateId = currentUpdateId;
            std::cout << "--- Device network changed, update id: " << currentUpdateId << " ---" << std::endl;

            // Get first idle node that supports tracking task.
            const Antilatency::DeviceNetwork::NodeHandle trackingNode_head = getIdleTrackingNode(network, altTrackingCotaskConstructor, "Head");
            const Antilatency::DeviceNetwork::NodeHandle trackingNode_gun = getIdleTrackingNode(network, altTrackingCotaskConstructor, "Gun");
            Antilatency::Alt::Tracking::ITrackingCotask altTrackingCotask_head;
            Antilatency::Alt::Tracking::ITrackingCotask altTrackingCotask_gun;
            if (trackingNode_head != Antilatency::DeviceNetwork::NodeHandle::Null) { 
                altTrackingCotask_head = altTrackingCotaskConstructor.startTask(network, trackingNode_head, environment);
            }
            if (trackingNode_gun != Antilatency::DeviceNetwork::NodeHandle::Null) {
                altTrackingCotask_gun = altTrackingCotaskConstructor.startTask(network, trackingNode_gun, environment);
            }
            int havetry = 1;
            uint8_t buffer_recv[256] = {'\0'};
            int count = 0;
            bool up = true;
            //struct sockaddr_in remote_addr; 
            struct sockaddr_in rev_addr; 
            
                if (altTrackingCotask_head != nullptr || altTrackingCotask_gun != nullptr) {
                    while (altTrackingCotask_head != nullptr || altTrackingCotask_gun != nullptr) {
                        // Print the extrapolated state of node to the console every 500 ms (2FPS).
                        
                        uint32_t currentUpdateId = 0;
                       
                        if (altTrackingCotask_head != nullptr) {
                            if (altTrackingCotask_head.isTaskFinished()) {
                                std::cout << "Head Tracking task finished" << std::endl;
                                break;
                            }
                        }
                        else
                        {
                            if (havetry == 1)
                            {
                                currentUpdateId = network.getUpdateId();
                                std::cout << "--- Device network update id: " << currentUpdateId << " ---" << std::endl;
                                prevUpdateId = currentUpdateId;
                                havetry = 0;
                            }
                            currentUpdateId = network.getUpdateId();
                            if (prevUpdateId != currentUpdateId) {
                                //prevUpdateId = currentUpdateId;
                                std::cout << "--- Device network changed, update id: " << currentUpdateId << " ---" << std::endl;
                                std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(500));
                                break;
                            }

                        }
                        if (altTrackingCotask_gun != nullptr) {
                            if (altTrackingCotask_gun.isTaskFinished()) {
                                std::cout << "Gun Tracking task finished" << std::endl;
                                break;
                            }
                        }

                        json sendMessage;
                        if (altTrackingCotask_head != nullptr) {

                                //wait for tick
                                //socklen_t addr_len = sizeof(rev_addr);
                                //std::cout << "tick!!" <<std::endl;
                                //len = recvfrom(client_sockfd, buffer_recv, sizeof(buffer_recv), 0, (struct sockaddr*)&rev_addr, &addr_len);

                            if(blink){
                                //std::cout << "recived udp " << len <<std::endl;
                            


                                Antilatency::Alt::Tracking::State state = altTrackingCotask_head.getExtrapolatedState(placement, 0.016f);

                                std::cout << "Head State:" << std::endl;

                                std::cout << "\tPose:" << std::endl;
                                std::cout << "\t\tPosition: x: " << state.pose.position.x << ", y: " << state.pose.position.y << ", z: " << state.pose.position.z << std::endl;
                                std::cout << "\t\tRotation: x: " << state.pose.rotation.x << ", y: " << state.pose.rotation.y << ", z: " << state.pose.rotation.z << ", w: " << state.pose.rotation.w << std::endl;

                                std::cout << "\tStability:" << std::endl;
                                std::cout << "\t\tStage: " << static_cast<int32_t>(state.stability.stage) << std::endl;
                                std::cout << "\t\tValue: " << state.stability.value << std::endl;

                                //std::cout << "\tVelocity:" << state.velocity.x << ", y: " << state.velocity.y << ", z: " << state.velocity.z << std::endl;

                                //std::cout << "\tLocalAngularVelocity: x:" << state.localAngularVelocity.x << ", y: " << state.localAngularVelocity.y << ", z: " << state.localAngularVelocity.z << std::endl << std::endl;

                                sendMessage["Head"]["pos"]["X"] = setPrecise5(state.pose.position.x);
                                sendMessage["Head"]["pos"]["Y"] = setPrecise5(state.pose.position.y);
                                sendMessage["Head"]["pos"]["Z"] = setPrecise5(state.pose.position.z);

                                sendMessage["Head"]["rot"]["X"] = setPrecise5(state.pose.rotation.x);
                                sendMessage["Head"]["rot"]["Y"] = setPrecise5(state.pose.rotation.y);
                                sendMessage["Head"]["rot"]["Z"] = setPrecise5(state.pose.rotation.z);
                                sendMessage["Head"]["rot"]["W"] = setPrecise5(state.pose.rotation.w);

                                sendMessage["Head"]["sta"]["stage"] = static_cast<int32_t>(state.stability.stage);
                                sendMessage["Head"]["sta"]["Val"] = state.stability.value;

                                blink = false;
                            }
                            else{

                                if(count<20){
                                    count++;
                                }else{
                                    count = 0;
                                    blink = true;
                                }
                                sendMessage["Head"]["sta"]["stage"] = 1;
                                sendMessage["Head"]["sta"]["Val"] = 0;
                            }



                        }
                        else{
                                sendMessage["Head"]["sta"]["stage"] = 1;
                                sendMessage["Head"]["sta"]["Val"] = 0;
                            
                            
                            }
                        

                        if (altTrackingCotask_gun != nullptr) {
                            Antilatency::Alt::Tracking::State state = altTrackingCotask_gun.getExtrapolatedState(placement_gun, 0.016f);

                            std::cout << "Gun State:" << std::endl;

                            std::cout << "\tPose:" << std::endl;
                            std::cout << "\t\tPosition: x: " << state.pose.position.x << ", y: " << state.pose.position.y << ", z: " << state.pose.position.z << std::endl;
                            std::cout << "\t\tRotation: x: " << state.pose.rotation.x << ", y: " << state.pose.rotation.y << ", z: " << state.pose.rotation.z << ", w: " << state.pose.rotation.w << std::endl;

                            std::cout << "\tStability:" << std::endl;
                            std::cout << "\t\tStage: " << static_cast<int32_t>(state.stability.stage) << std::endl;
                            std::cout << "\t\tValue: " << state.stability.value << std::endl;

                            //std::cout << "\tVelocity:" << state.velocity.x << ", y: " << state.velocity.y << ", z: " << state.velocity.z << std::endl;

                            //std::cout << "\tLocalAngularVelocity: x:" << state.localAngularVelocity.x << ", y: " << state.localAngularVelocity.y << ", z: " << state.localAngularVelocity.z << std::endl << std::endl;

                        
#if defined(__linux__)
                            auto pinState_Trigger = digitalRead(pin_Trigger);
                            auto pinState_Switch = digitalRead(pin_Switch);
                            auto pinState_Reload = digitalRead(pin_Reload);
                            auto pinState_F1 = digitalRead(pin_F1);
                            auto pinState_F2 = digitalRead(pin_F2);
                            sendMessage["Gun"]["pos"]["X"] = setPrecise5(state.pose.position.x);
                            sendMessage["Gun"]["pos"]["Y"] = setPrecise5(state.pose.position.y);
                            sendMessage["Gun"]["pos"]["Z"] = setPrecise5(state.pose.position.z);

                            sendMessage["Gun"]["rot"]["X"] = setPrecise5(state.pose.rotation.x);
                            sendMessage["Gun"]["rot"]["Y"] = setPrecise5(state.pose.rotation.y);
                            sendMessage["Gun"]["rot"]["Z"] = setPrecise5(state.pose.rotation.z);
                            sendMessage["Gun"]["rot"]["W"] = setPrecise5(state.pose.rotation.w);

                            sendMessage["Gun"]["sta"]["stage"] = static_cast<int32_t>(state.stability.stage);
                            sendMessage["Gun"]["sta"]["Val"] = state.stability.value;

                            sendMessage["Gun"]["Pin"]["T"] = (pinState_Trigger == 0)?"ON":"OFF";
                            sendMessage["Gun"]["Pin"]["S"] = (pinState_Switch == 0)? "ON":"OFF";
                            sendMessage["Gun"]["Pin"]["R"] = (pinState_Reload == 0)? "ON":"OFF";
                            sendMessage["Gun"]["Pin"]["F1"] = (pinState_F1 == 0)? "ON":"OFF";
                            sendMessage["Gun"]["Pin"]["F2"] = (pinState_F2 == 0)? "ON":"OFF";

#endif
                        }
                        
                        std::string message_str = sendMessage.dump();
                            strcpy(buf, message_str.c_str()); 

                            std::cout << message_str << std::endl;
                          
                            if ((len = sendto(client_sockfd, buf, strlen(buf), 0, (struct sockaddr*)&remote_addr_gun, sizeof(struct sockaddr))) < 0)
                            {
                                perror("socket error when send message");
                                
                                return 1;
                            }
                        

                        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(10));
                    }
                }
                else {
                    std::cout << "Failed to start tracking task on node" << std::endl;
                }
        }
    }






}


int main(int argc, char* argv[]) {
   // if (argc != 3) {
   //     std::cout << "Wrong arguments. Pass environment data string as first argument and placement data as second.";
   //     return 1;
   // }
#if defined(__linux__)
    Dl_info dlinfo;
    dladdr(reinterpret_cast<void*>(&main), &dlinfo);
    std::string path = std::filesystem::path(dlinfo.dli_fname).parent_path();
    std::string libNameADN = path + "/libAntilatencyDeviceNetwork.so";
    std::string libNameTracking = path + "/libAntilatencyAltTracking.so";
    std::string libNameEnvironmentSelector = path + "/libAntilatencyAltEnvironmentSelector.so";
#else
    std::string libNameADN = "AntilatencyDeviceNetwork";
    std::string libNameTracking = "AntilatencyAltTracking";
    std::string libNameEnvironmentSelector = "AntilatencyAltEnvironmentSelector";
#endif


//================creat a default config.json file=====================
json my_config;

 my_config["gun_SN"] = "001";

 my_config["bind"] = "false";
 my_config["bind"]["hostIP"] = "192.168.1.223";
 my_config["bind"]["hostPort"] = 36369;
 my_config["bind"]["device_name"] = "oculus1";
 my_config["bind"]["player_name"] = "unknow";

 my_config["parameter"]["Enviroment"] = "0";
 my_config["parameter"]["Placement"] = "001";
 my_config["parameter"]["GunRotationOffset"] = "001";
 my_config["parameter"]["GunPositionOffset"] = "001";
 my_config["parameter"]["SetSPS"] = "001";

 my_config["Vib"] = "001";
 my_config["VibMode"] = "001";
 my_config["Color"] = "001";
 my_config["BlinkMode"] = "001";


std::ofstream ofs("config.json");
if(!ofs.is_open()){
     std::cout<<"open file error"<<std::endl;
}
else
{
    ofs<<std::setw(4)<<my_config<<std::endl;
}

//================================================
//开一个udp lisenter,is config changed,save to file and reboot

thread thread_command(Command_task,1);
thread thread_main(main_task,1);

thread_command.join();
thread_main.join();

return 0;
}
