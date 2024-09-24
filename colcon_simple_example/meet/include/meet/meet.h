#pragma once

#ifdef WIN32
  #define MEET_EXPORT __declspec(dllexport) 
#else
  #define MEET_EXPORT  
#endif

MEET_EXPORT void meet();
