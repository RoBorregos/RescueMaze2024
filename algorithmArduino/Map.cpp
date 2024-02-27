#include "Map.h"

Map::Map() {
    this->positions.reserve(kMaxMapSize);
}

int Map::getIndex(const coord &position) const {
    for (int i = 0; i < this->positions.size(); i++) {
        if (this->positions[i] == position) {
            return i;
        }
    }
    return -1;
}