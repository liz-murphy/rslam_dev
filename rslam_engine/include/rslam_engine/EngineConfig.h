// Place to store formally global rslam parameters
#ifndef ENGINE_CONFIG_H
#define ENGINE_CONFIG_H

class EngineConfig 
{
public:
   static EngineConfig * getConfig();
   int skip_n_frames;
   int debug_level;
   int error_level;

   
private:
   EngineConfig(){
  };
   static EngineConfig * m_configInstance;
};

#endif
