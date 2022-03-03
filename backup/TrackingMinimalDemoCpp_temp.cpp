#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

#include <Antilatency.InterfaceContract.LibraryLoader.h>
#include <Antilatency.DeviceNetwork.h>


#include <configor/json.hpp>

#define EDIT

#if defined (__linux__) || defined (EDIT)

#include <CommandRecv.h>
#include <dlfcn.h>
#include <filesystem>


#include <sys/socket.h>		// For socket()
#include <netinet/in.h> 	// For socket() and sockaddr_in struct */
#include <netinet/udp.h>	// For socket()
#include <unistd.h>			// For close()
#include <arpa/inet.h>		// For inet_aton()

#include <wiringPi.h>

using namespace configor;

#endif

#define _USE_MATH_DEFINES
#include <cmath>

#include <thread>
#include <chrono>


#if defined(__linux__) || defined (EDIT)
int pin_Trigger = 21;
int pin_Switch = 22;
int pin_Reload = 23;
int pin_F1 = 24;
int pin_F2 = 25;
#endif


struct Quaternion {
    double w, x, y, z;
};

Quaternion qRot;


struct EulerAngles {
    double roll, pitch, yaw;
};

Quaternion Qmulti (const Quaternion rp,const Quaternion rq)
{
	// the constructor takes its arguments as (x, y, z, w)
    Quaternion returnQ;
	returnQ.x = rp.w * rq.x + rp.x * rq.w + rp.y * rq.z - rp.z * rq.y;
	returnQ.y = rp.w * rq.y + rp.y * rq.w + rp.z * rq.x - rp.x * rq.z;
	returnQ.z = rp.w * rq.z + rp.z * rq.w + rp.x * rq.y - rp.y * rq.x;
	returnQ.w = rp.w * rq.w - rp.x * rq.x - rp.y * rq.y - rp.z * rq.z;

    return returnQ;
}


EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    angles.yaw = angles.yaw * 180 / 3.1415926;
    angles.pitch = angles.pitch * 180 / 3.1415926;
    angles.roll = angles.roll * 180 / 3.1415926;

    std::cout << "Roll: " << angles.yaw << std::endl;
    std::cout << "Pitch: " << angles.roll << std::endl;
    std::cout << "Yaw: " << angles.pitch << std::endl;

    return angles;
}

EulerAngles ToEulerAngles2UE(Quaternion q) {
    EulerAngles angles;

    // roll (z-axis rotation)
    double sinr_cosp = 2 * (q.w * q.z + q.y * q.x);
    double cosr_cosp = 1 - 2 * (q.z * q.z + q.x * q.x);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (x-axis rotation)
    double sinp = 2 * (q.w * q.x - q.z * q.y);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (y-axis rotation)
    double siny_cosp = 2 * (q.w * q.y + q.x * q.z);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.x * q.x);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    //angles.yaw = angles.yaw * 180 / 3.1415926;
    if(angles.yaw< -1.570796326795){
        angles.yaw += 4.71238898038469;
    }else{
        angles.yaw -= 1.570796326794;
    }
    angles.yaw = angles.yaw * 180 / 3.1415926;
    angles.pitch = angles.pitch * 180 / 3.1415926;
    angles.roll = angles.roll * 180 / 3.1415926;

    std::cout << "Yaw: " << angles.yaw << std::endl;
    std::cout << "Roll: " << angles.roll << std::endl;
    std::cout << "Pitch: " << angles.pitch << std::endl;

    return angles;
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
/*
     jconfig["TargetIP"] = jMessage["TargetIP"];
     jconfig["TargetPort"] = jMessage["TargetPort"];
     jconfig["Enviroment"] = jMessage["Enviroment"];
     jconfig["Placement"] = jMessage["Placement"];
     jconfig["GunPositionOffset"] = jMessage["GunPositionOffset"];
     jconfig["GunRotationOffset"] = jMessage["GunRotationOffset"];
     jconfig["TargetIP"] = jMessage["TargetIP"];
 }
 */


int main(int argc, char* argv[]) {
   // if (argc != 3) {
   //     std::cout << "Wrong arguments. Pass environment data string as first argument and placement data as second.";
   //     return 1;
   // }
    qRot.x = 0;
    qRot.y = -0.7071068;
    qRot.z = 0.7071068;
    qRot.w = 0;

    json jconfig;
    std::cout << "step 1" << std::endl;
    readConfig(jconfig);
    
    //char buf2[BUFSIZ];  
    
    std::string config_str = jconfig.dump();
    //strcpy(buf2, config_str.c_str()); 
    std::cout << config_str << std::endl;

    
    


//std::thread Tcommand(Command_task, 1);//start Command task;

#if defined(__linux__) || defined (EDIT)
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

#if defined(__linux__) || defined (EDIT)
    json sendMessage;
    sendMessage["Head"]["Pos"]["X"] = 1.12;
    sendMessage["Head"]["Pos"]["Y"] = 100.12;
    sendMessage["Head"]["Pos"]["Z"] = 1.12;
    sendMessage["Head"]["Rot"]["P"] = 1.12;
    sendMessage["Head"]["Rot"]["R"] = 1.12;
    sendMessage["Head"]["Rot"]["Y"] = 1.12;

    sendMessage["Gun"]["Pos"]["X"] = 1.12;
    sendMessage["Gun"]["Pos"]["Y"] = 168.12;
    sendMessage["Gun"]["Pos"]["Z"] = 1.12;
    sendMessage["Gun"]["Rot"]["P"] = 1.12;
    sendMessage["Gun"]["Rot"]["R"] = 1.12;
    sendMessage["Gun"]["Rot"]["Y"] = 1.12;
    sendMessage["Gun"]["Pin"]["T"] = "ON";
    sendMessage["Gun"]["Pin"]["R"] = "OFF";
    sendMessage["Gun"]["Pin"]["S"] = "ON";
    sendMessage["Gun"]["Pin"]["F1"] = "OFF";
    sendMessage["Gun"]["Pin"]["F2"] = "OFF";

    //init the socket
    int client_sockfd, ret, on;
    int len;
    struct sockaddr_in remote_addr; 
    int sin_size;
    char buf[BUFSIZ];  
    memset(&remote_addr, 0, sizeof(remote_addr)); 
    remote_addr.sin_family = AF_INET; 
    remote_addr.sin_addr.s_addr = inet_addr("192.168.1.169");
    remote_addr.sin_port = htons(36361); 


    struct sockaddr_in local_addr; //Local
    memset(&local_addr, 0, sizeof(local_addr)); 
    local_addr.sin_family = AF_INET; 
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(36363); 
   
    if ((client_sockfd = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creat error");
        return 1;
    }
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
/*
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
    }*/
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

    const std::string environmentData = "AAVSaWdpZIQDAQZ5ZWxsb3ckFbhTiT_cRqA-r45jvZqZmT4AAAAAAAAAAACamRk_dAkLAxMDARwKAB4HARAMAwkDARcIAAUKABkMARYQAAMHAREQABsAAhwCAxoUAiALABkGAx0QABUEAw0IAyAQAAQUACMIAxYAABAUAAsOAgkHAwoRAw0QARIPAg4AAgwLAiEOAxMHAQQJAA8BABYUAhUPAgsPACMSASMOAQQAAAACAw8OABcSABESAgIEAhILAA8HAAQSAx8DAAcCAxQKAhgCAyMMAw4KAiEAABkCAwcMAhQTAhwIAxwNAiMCAwcRACEUAAMOAxMAAgsFAAYDARsQAxUGAAILAQMDAhgOAwMRAQ4EAQgOAhYMARoFAQ0DAx8OAxoKAwQOARkRAwsTACAIARIDAx4MAwEAAA4UAhQNAwcFASARAgcUAAEUAB0SAiMEAR0EAgYQAhgJAgAGAR0UAAsJACIIAQ0MAB8FAAgIAh8AAhAFAxEJAgASAQgAAAAOAQUHAwALAwsBAQ";

    //const std::string environmentData = "AntilatencyAltEnvironmentHorizontalGrid~AgZ5ZWxsb3cEBLhTiT_cRqA-r45jvZqZmT4AAAAAAAAAAACamRk_AQQAAQEBAwABAAADAQE";
    //const std::string placementData = "AAAAAAAAAAAAAAAAAAAAAIAAAAAAAAAAAA"; 
      const std::string placementData = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";

    // Create environment object from the serialized data.
    const Antilatency::Alt::Environment::IEnvironment environment = environmentSelectorLibrary.createEnvironment(environmentData);
    if (environment == nullptr) {
        std::cout << "Failed to create environment" << std::endl;
        return 1;
    }

    // Create placement from the serialized data.
    const Antilatency::Math::floatP3Q placement = altTrackingLibrary.createPlacement(placementData);

    // Create alt tracking cotask constructor to find tracking-supported nodes and start tracking task on node.
    Antilatency::Alt::Tracking::ITrackingCotaskConstructor altTrackingCotaskConstructor = altTrackingLibrary.createTrackingCotaskConstructor();
    if (altTrackingCotaskConstructor == nullptr) {
        std::cout << "Failed to create Antilatency Alt Tracking Cotask Constructor" << std::endl;
        return 1;
    }

    // Each time the device network is changed due to connection or disconnection of a device that matches the device filter of the network,
    // or start or stop of a task on any network device, the network update id is incremented by 1. 
    uint32_t prevUpdateId = 0;
    uint num=1;

    while (network != nullptr) {
        // Check if the network has been changed.
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(100));
        const uint32_t currentUpdateId = network.getUpdateId();

        if (prevUpdateId != currentUpdateId) {
            prevUpdateId = currentUpdateId;
            std::cout << "--- Device network changed, update id: " << currentUpdateId << " ---" << std::endl;

            // Get first idle node that supports tracking task.
            const Antilatency::DeviceNetwork::NodeHandle trackingNode_head = getIdleTrackingNode(network, altTrackingCotaskConstructor, "Head");
            //const Antilatency::DeviceNetwork::NodeHandle trackingNode_head = Antilatency::DeviceNetwork::NodeHandle::Null;
            
            //const Antilatency::DeviceNetwork::NodeHandle trackingNode_gun = getIdleTrackingNode(network, altTrackingCotaskConstructor, "Gun");
            Antilatency::Alt::Tracking::ITrackingCotask altTrackingCotask_head;
            //Antilatency::Alt::Tracking::ITrackingCotask altTrackingCotask_gun;
            if (trackingNode_head != Antilatency::DeviceNetwork::NodeHandle::Null) { 
                altTrackingCotask_head = altTrackingCotaskConstructor.startTask(network, trackingNode_head, environment);
            }
            //if (trackingNode_gun != Antilatency::DeviceNetwork::NodeHandle::Null) {
            //    altTrackingCotask_gun = altTrackingCotaskConstructor.startTask(network, trackingNode_gun, environment);
            //}
            int havetry = 1;
                //if (altTrackingCotask_head != nullptr || altTrackingCotask_gun != nullptr) {
                   // while (altTrackingCotask_head != nullptr || altTrackingCotask_gun != nullptr) {
                
                if (altTrackingCotask_head != nullptr) {
                    while (altTrackingCotask_head != nullptr) {        
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
                            
                            //currentUpdateId = network.getUpdateId();
                            std::cout << "--- Device network update id: " << currentUpdateId << " ---" << std::endl;
                            //prevUpdateId = currentUpdateId;
                            break;
                        }

                        if (altTrackingCotask_head != nullptr) {//altTrackingCotask_head
                            Antilatency::Alt::Tracking::State state = altTrackingCotask_head.getExtrapolatedState(placement, 0.03f);

                            std::cout << "Head State:" << std::endl;

                            std::cout << "\tPose:" << std::endl;
                            std::cout << "\t\tPosition: x: " << state.pose.position.x << ", y: " << state.pose.position.y << ", z: " << state.pose.position.z << std::endl;
                            std::cout << "\t\tRotation: x: " << state.pose.rotation.x << ", y: " << state.pose.rotation.y << ", z: " << state.pose.rotation.z << ", w: " << state.pose.rotation.w << std::endl;

                            std::cout << "\tStability:" << std::endl;
                            std::cout << "\t\tStage: " << static_cast<int32_t>(state.stability.stage) << std::endl;
                            std::cout << "\t\tValue: " << state.stability.value << std::endl;

                            std::cout << "\tVelocity:" << state.velocity.x << ", y: " << state.velocity.y << ", z: " << state.velocity.z << std::endl;

                            std::cout << "\tLocalAngularVelocity: x:" << state.localAngularVelocity.x << ", y: " << state.localAngularVelocity.y << ", z: " << state.localAngularVelocity.z << std::endl << std::endl;
                            int tx,ty,tz;
                            //float_t fx,fy,fz;
                            tx = state.pose.position.x * 10000;
                            ty = state.pose.position.y * 10000;
                            tz = state.pose.position.z * 10000;
                            /*
                            std::cout <<"tx:"<< tx << std::endl;
                            std::cout <<"ty:"<< ty << std::endl;
                            std::cout <<"tz:"<< tz << std::endl;
                            fx = tx/10.0;
                            fy = ty/10.0;
                            fz = tz/10.0;
                            
                            std::cout <<"fx:"<< fx << std::endl;
                            std::cout <<"fy:"<< fy << std::endl;
                            std::cout <<"fz:"<< fz << std::endl;
                            */
                            EulerAngles myeuler;
                            Quaternion rawQ;
                            rawQ.x = state.pose.rotation.x;
                            rawQ.y = state.pose.rotation.y;
                            rawQ.z = state.pose.rotation.z;
                            rawQ.w = state.pose.rotation.w;
                        
                            myeuler = ToEulerAngles2UE(rawQ);
                            
                            int ry,rp,rr;
                            ry= myeuler.yaw*100;
                            //rp= myeuler.pitch*100;
                            //rr= myeuler.roll*100;


#if defined(__linux__) || defined (EDIT)
                            sendMessage["Head"]["Pos"]["X"] = tx;//state.pose.position.x * 100; //to centmeter
                            sendMessage["Head"]["Pos"]["Y"] = -tz;//-state.pose.position.z * 100;
                            sendMessage["Head"]["Pos"]["Z"] = ty;//state.pose.position.y * 100;

                            sendMessage["Head"]["Rot"]["Y"] = ry;//myeuler.yaw;
                            sendMessage["Head"]["Rot"]["P"] = 0;//myeuler.pitch;
                            sendMessage["Head"]["Rot"]["R"] = 0;//myeuler.roll;
#endif
                        }

#if defined(__linux__) || defined (EDIT)
                        sendMessage["NO"] = num;
                        std::string message_str = sendMessage.dump();
                        strcpy(buf, message_str.c_str()); 

                        std::cout << message_str << std::endl;
                        
                        if ((len = sendto(client_sockfd, buf, strlen(buf), 0, (struct sockaddr*)&remote_addr, sizeof(struct sockaddr))) < 0)
                        {
                            perror("socket error when send message");
                            return 1;
                        }
                        num++;
#endif
                        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(2));
                        
                        system("clear");
                    }
                }
                else {
                    std::cout << "Failed to start tracking task on node" << std::endl;
                }
        }
    }

    return 0;
}
