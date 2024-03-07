#ifndef Map_h
#define Map_h

// #define ETL_NO_STL
#include<Arduino.h>
#include "coord.h"
#include"Embedded_Template_Library.h"
#include<etl/vector.h>
//C:\Users\tacol\Documents\Arduino\libraries\ArduinoSTL\src\new_handler.cpp
//error line 22
//C:\Users\tacol\Documents\Arduino\libraries\Embedded_Template_Library_-_ETL\src\etl\initializer_list.h
//error lines 204-217

const uint8_t kMaxMapSize = 150;
const uint8_t kInvalidIndex = -1;

class Map{
    public:
        etl::vector<coord, kMaxMapSize> positions;
        Map();
        int getIndex(const coord &position) const;
};

#endif