#define ETL_NO_STL

#ifndef Map_h
#define Map_h

#include<Arduino.h>

#include "Tile.h"

#include"Embedded_Template_Library.h"
//C:\Users\tacol\Documents\Arduino\libraries\ArduinoSTL\src\new_handler.cpp
//error line 22
#include<etl/vector.h>
#include<etl/stack.h>

const int kMaxMapSize = 150;

class Map{
    public:
        // TODO: just leave indexes as a vector of coords.
        etl::vector<coord, kMaxMapSize> positions;
        Map();
        int getIndex(const coord &position) const;
};

#endif