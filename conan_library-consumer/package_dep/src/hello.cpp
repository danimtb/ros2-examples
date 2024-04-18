#include <iostream>

#include "Poco/MD5Engine.h"
#include "Poco/DigestStream.h"

#include "hello.h"


void hello() {

    Poco::MD5Engine md5;
    Poco::DigestOutputStream ds(md5);
    ds << "Hello world";
    ds.close();
    std::string msg = Poco::DigestEngine::digestToHex(md5.digest());
    
    std::string build_type;
    #ifdef NDEBUG
    build_type = "Release";
    #else
    build_type = "Debug";
    #endif

    std::cout << msg + " " + build_type << std::endl;
}