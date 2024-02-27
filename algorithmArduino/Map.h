#define ETL_NO_STL

#ifndef Map_h
#define Map_h

#include<Arduino.h>

#include "Tile.h"

#include"Embedded_Template_Library.h" //new_handler.cpp error line 22
#include<etl/vector.h>
#include<etl/stack.h>

const int kMaxMapSize = 50;

class Map{
    public:
        // TODO: just leave indexes as a vector of coords.
        etl::vector<coord, kMaxMapSize> indexes;
        etl::vector<Tile, kMaxMapSize> tiles;
        etl::vector<bool, kMaxMapSize> bools;
        etl::vector<int, kMaxMapSize> ints;
        etl::vector<coord, kMaxMapSize> coords;
        String type;
        Map();
        Map(String type);
        void addTile(const Tile& tile, const coord& position);
        void addBool(const bool& b, const coord& position);
        void addInt(const int& i, const coord& position);
        void addCoord(const coord& c, const coord& position);
        Tile* getTile(const coord& position);
        bool getBool(const coord& position);
        int getInt(const coord& position);
        coord getCoord(const coord& position);
        void setTile(const Tile& tile, const coord& position);
        void setBool(const bool& b, const coord& position);
        void setInt(const int& in, const coord& position);
        void setCoord(const coord& c, const coord& position);
};

#endif