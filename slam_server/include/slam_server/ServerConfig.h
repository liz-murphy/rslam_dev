// Place to store formally global rslam parameters

class ServerConfig 
{
public:
   static ServerConfig * getConfig();
   int debug_level;
   private:
   ServerConfig(){
    debug_level = 1; 
   };
   static ServerConfig * m_configInstance;
};
