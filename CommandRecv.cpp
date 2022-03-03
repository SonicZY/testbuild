

#include <CommandRecv.h>

#define Local_Port 36362

void Command_task(int n) {
    json recvMessage;
    /*
    //参数设定写入配置文件，并及时生效
    设定进入休眠
    设定进入工作模式
    设置定时休眠
    查询电量
    设置placement？enviroment？
    设置目标客户端ip
    设置帧率，默认70帧

    GPIO相关：
    设置电源开关
    设置颜色（队伍、闪烁模式）
    设置振动开关，强弱、模式
    */

    //头和gun可以不用一个placement么??


    //init the socket
    int client_sockfd, ret, on;
    int len;
    struct sockaddr_in remote_addr; 
    struct sockaddr_in rev_addr; 
    int sin_size;
    char buf[BUFSIZ];  

    
    //int addr_len = sizeof(struct sockaddr_in);

    memset(&remote_addr, 0, sizeof(remote_addr)); 
    remote_addr.sin_family = AF_INET; 
    remote_addr.sin_addr.s_addr = inet_addr("192.168.31.177");
    remote_addr.sin_port = htons(36363); 

    struct sockaddr_in local_addr; 
    memset(&local_addr, 0, sizeof(local_addr)); 
    local_addr.sin_family = AF_INET; 
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(Local_Port); 

    if ((client_sockfd = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creat error");
        return ;
    }
    on = 1;
    ret = setsockopt(client_sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    if (bind(client_sockfd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        perror("socket bind error");
    }

    strcpy(buf, "Prop1 Socket initalnaized,hello there\n"); 
    //sin_size=sizeof(struct sockaddr_in);
    if ((len = sendto(client_sockfd, buf, strlen(buf), 0, (struct sockaddr*)&remote_addr, sizeof(struct sockaddr))) < 0)
    {
        perror("socket error when send message");
        return ;
    }

    while(1){
        uint8_t buffer_recv[256] = {'\0'};
        socklen_t addr_len = sizeof(rev_addr);
        std::cout << "hello thread!!!!! " <<std::endl;

        len = recvfrom(client_sockfd, buffer_recv, sizeof(buffer_recv), 0, (struct sockaddr*)&rev_addr, &addr_len);
        
        std::string Message(&buffer_recv[0],&buffer_recv[len]);
        
        std::cout << "Revived form client:" << Message << std::endl;

        json jMessage = json::parse(Message);

        /*for (auto iter = jMessage.begin(); iter != jMessage.end(); iter++) {
            std::cout << iter.key() << ":" << iter.value() << std::endl;

        }*/
        json jconfig;
        bool config_flag = false;

        if (jMessage.count("Command"))
        {
            if ("start" == jMessage["Command"].as_string())
            {
                std::cout << "start the tracking task" << std::endl;
            }
            else if("idle" == jMessage["Command"].as_string())
            {      
                std::cout << "stop the tracking task" << std::endl;
            }
        }
        //======Config========
        if (jMessage.count("TargetIP"))
        {
            std::cout << "Change the remoter host IP to :"<< jMessage["TargetIP"] << std::endl;
            jconfig["TargetIP"] = jMessage["TargetIP"];
            config_flag = true;
        }

        if (jMessage.count("TargetPort"))
        {
            std::cout << "Change the remoter host port to :"<< jMessage["TargetPort"] << std::endl;
            jconfig["TargetPort"] = jMessage["TargetPort"];
            config_flag = true;
        }

        if (jMessage.count("Enviroment"))
        {
            std::cout << "Change the Enviroment code to :"<< jMessage["Enviroment"] << std::endl;
            jconfig["Enviroment"] = jMessage["Enviroment"];
            config_flag = true;
        }
        
        //Placement for Head
        if (jMessage.count("Placement"))
        {
            std::cout << "Change the Placement code to :"<< jMessage["Placement"] << std::endl;
            jconfig["Placement"] = jMessage["Placement"];
            config_flag = true;
        }
        //gun offset
        if (jMessage.count("GunPositionOffset"))
        {
            std::cout << "Change the GunPositionOffset to :"<< jMessage["GunPositionOffset"] << std::endl;
            jconfig["GunPositionOffset"] = jMessage["GunPositionOffset"];
            config_flag = true;
        }
        if (jMessage.count("GunRotationOffset"))
        {
            std::cout << "Change the GunRotationOffset to :"<< jMessage["GunRotationOffset"] << std::endl;
            jconfig["GunRotationOffset"] = jMessage["GunRotationOffset"];
            config_flag = true;
        }

        if (jMessage.count("SetSPS"))
        {
            std::cout << "Change the SPS  to :"<< jMessage["SetSPS"] << std::endl;
            jconfig["SetSPS"] = jMessage["SetSPS"];
            config_flag = true;
        }
        
        //=========GPIO======

        if (jMessage.count("Vib"))
        {
            std::cout << "Change the Vibration to :"<< jMessage["Vib"] << std::endl;
        }    

        if (jMessage.count("Mode"))
        {
            std::cout << "Change the Vibration Mode to :"<< jMessage["Mode"] << std::endl;   
        }    

        if (jMessage.count("Color"))
        {
            std::cout << "Change the Color to :"<< jMessage["Color"] << std::endl;
        }

        if (jMessage.count("BlinkMode"))
        {
            std::cout << "Change the BlinkMode to :"<< jMessage["BlinkMode"] << std::endl;
        }
        //save the palmeter to the config.json file
        if(config_flag){
            std::ofstream ofs("config.json");
            if(!ofs){
                std::cout << "Can not write config.json,please check the file!!" << std::endl;
                //return 0;
            }
            std::string config_str = jconfig.dump();
            //ofs.write(config_str,config_str.length());
            config_flag=false;
            ofs.close();
            std::cout << "Application rebooting,please wait some second!!" << std::endl;
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));
            //execv("/TrackingMinmaDemoCPP");

        }
        
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(400));
    }
}

/*
1\先做读取json文件测试
2、做保存接送文件测试
3、做发送命令测试
4、做重启程序测试

*/

int readConfig(json& j){
    
    std::ifstream ifs("config.json");
    std::cout << "step 2" << std::endl;
    

    if(!ifs){
        std::cout << "Can not read config.json,please check the file!!" << std::endl;
        return 0;
    }

    json jMessage;
    
    ifs >> jMessage;
    ifs.close();
    /*
    if (jMessage.count("TargetIP"))
    {
        std::cout << "TargetIP :"<< jMessage["TargetIP"] << std::endl;
    }else{
        std::cout << "Can't found TargetIP" std::endl;
        return 0;
    }

    if (jMessage.count("TargetPort"))
    {
        std::cout << "TargetPort :"<< jMessage["TargetPort"] << std::endl;
    }else{
        std::cout << "Can't found TargetPort" std::endl;
        return 0;
    }

    if (jMessage.count("Enviroment"))
    {
        std::cout << "Enviroment :"<< jMessage["Enviroment"] << std::endl;
    }else{
        std::cout << "Can't found Enviroment" std::endl;
        return 0;
    }
    
    //Placement for Head
    if (jMessage.count("Placement"))
    {
        std::cout << "Placement :"<< jMessage["Placement"] << std::endl;
    }else{
        std::cout << "Can't found Placement" std::endl;
        return 0;
    }
    //gun offset
    if (jMessage.count("GunPositionOffset"))
    {
        std::cout << "GunPositionOffset :"<< jMessage["GunPositionOffset"] << std::endl;
    }else{
        std::cout << "Can't found GunPositionOffset" std::endl;
        return 0;
    }
    if (jMessage.count("GunRotationOffset"))
    {
        std::cout << "GunRotationOffset :"<< jMessage["GunRotationOffset"] << std::endl;
    }else{
        
        std::cout << "Can't found GunRotationOffset" std::endl;
        return 0;
    }

    if (jMessage.count("SetSPS"))
    {
        std::cout << "SPS :"<< jMessage["SetSPS"] << std::endl;
    }else{
        std::cout << "Can't found SetSPS" std::endl;
        return 0;
    }*/
    
    j = jMessage;
    
    return 1;

}
