// Place to store formally global rslam parameters
#ifndef SERVER_CONFIG_H
#define SERVER_CONFIG_H

#include <string>

class ServerConfig 
{
public:
   static ServerConfig * getConfig()
   {
     if(!m_configInstance)
       m_configInstance = new ServerConfig;
     return m_configInstance;
   };
   int debug_level;
   int fetch_check_msg_size_skip;
   int client_connect_port;
   int server_bind_port;
   std::string client_connect_ip;
   int upload_depth;
   private:
   ServerConfig(){
    debug_level = 1; 
    fetch_check_msg_size_skip = 50;
    upload_depth = 500;
    client_connect_port = 1776;
    server_bind_port = 1776;
    client_connect_ip = "rpg.robotics.gwu.edu";
   };
   static ServerConfig * m_configInstance;
};

#endif
