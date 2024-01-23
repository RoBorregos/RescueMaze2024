#include<iostream>
#include<map>
using namespace std;

#include "Tile.h"

Tile::Tile(){
    this->position_ = coord{1000, 1000};
    adjacentTiles_[TileDirection::kUp] = NULL;
    adjacentTiles_[TileDirection::kDown] = NULL;
    adjacentTiles_[TileDirection::kLeft] = NULL;
    adjacentTiles_[TileDirection::kRight] = NULL;

    weights_[TileDirection::kUp] = 0;
    weights_[TileDirection::kDown] = 0;
    weights_[TileDirection::kLeft] = 0;
    weights_[TileDirection::kRight] = 0;

    this->walls_ = '\0';
}

Tile::Tile(const coord& position){
    this->position_ = position;

    adjacentTiles_[TileDirection::kUp] = NULL;
    adjacentTiles_[TileDirection::kDown] = NULL;
    adjacentTiles_[TileDirection::kLeft] = NULL;
    adjacentTiles_[TileDirection::kRight] = NULL;

    weights_[TileDirection::kUp] = 0;
    weights_[TileDirection::kDown] = 0;
    weights_[TileDirection::kLeft] = 0;
    weights_[TileDirection::kRight] = 0;

    this->walls_ = '\0';
}

void Tile::setWall(const TileDirection direction, const bool wall){
    if(wall){
        this->walls_ |= (1 << static_cast<int>(direction));
    }
}

bool Tile::hasWall(const TileDirection direction){
    return this->walls_ & (1 << static_cast<int>(direction));
}

bool Tile::hasVictim(){
    return this->walls_ & (1 << kVictimBit);
}

void Tile::setVictim(){
    this->walls_ |= (1 << kVictimBit);
}

bool Tile::hasObstacle(){
    return this->walls_ & (1 << kObstacleBit);
}

void Tile::setObstacle(){
    this->walls_ |= (1 << kObstacleBit);
}

void Tile::addAdjacentTile(const TileDirection direction, Tile *tile, const bool wall, const coord& position){
    adjacentTiles_[direction] = tile;
    weights_[direction] = 1;
    this->setWall(direction, wall);
    tile->setPosition(position);//posible error
}

void Tile::setPosition(const coord& position){
    this->position_ = position;
}
