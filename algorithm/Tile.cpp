#include<iostream>
#include<map>
using namespace std;

#include "Tile.h"

Tile::Tile(){ //initialize walls_?
    position_ = coord{1000, 1000};
    adjacentTiles_[TileDirection::kUp] = NULL;
    adjacentTiles_[TileDirection::kDown] = NULL;
    adjacentTiles_[TileDirection::kLeft] = NULL;
    adjacentTiles_[TileDirection::kRight] = NULL;

    weights_[TileDirection::kUp] = 0;
    weights_[TileDirection::kDown] = 0;
    weights_[TileDirection::kLeft] = 0;
    weights_[TileDirection::kRight] = 0;

    // walls_[TileDirection::kUp] = false;
    // walls_[TileDirection::kDown] = false;
    // walls_[TileDirection::kLeft] = false;
    // walls_[TileDirection::kRight] = false;

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

    // walls_[TileDirection::kUp] = false;
    // walls_[TileDirection::kDown] = false;
    // walls_[TileDirection::kLeft] = false;
    // walls_[TileDirection::kRight] = false;

    this->walls_ = '\0';
}

// void Tile::setWall(const TileDirection direction, const bool wall){
//     if(wall){
//         this->wallsChar_ |= (1 << static_cast<int>(direction));
//     }
// }

bool Tile::hasWall(const TileDirection direction){
    return this->walls_ & (1 << static_cast<int>(direction));
}

bool Tile::hasVictim(){
    return this->walls_ & (1 << 4);
}

void Tile::setVictim(){
    this->walls_ |= (1 << 4);
}

bool Tile::hasObstacle(){
    return this->walls_ & (1 << 5);
}

void Tile::setObstacle(){
    this->walls_ |= (1 << 5);
}

void Tile::addAdjacentTile(const TileDirection direction, Tile *tile, const bool wall, const coord& position){
    adjacentTiles_[direction] = tile;
    weights_[direction] = 1;
    if(wall){
        this->walls_ |= (1 << static_cast<int>(direction));
    }
    // walls_[direction] = wall;
    tile->setPosition(position);//posible error
}

void Tile::setPosition(const coord& position){
    this->position_ = position;
}
