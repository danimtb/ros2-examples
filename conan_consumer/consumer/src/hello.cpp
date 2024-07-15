#include <iostream>

#include "hello.h"

#include "Box2D/Box2D.h"


void hello() {
    #ifdef NDEBUG
    std::cout << "Hello World Release! " << b2_maxPolygonVertices <<std::endl;
    #else
    std::cout << "Hello World Debug! " << b2_maxPolygonVertices <<std::endl;
    #endif
}
