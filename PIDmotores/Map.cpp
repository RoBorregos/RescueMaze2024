#include "Map.h"

Map::Map() {
    this->positions.reserve(kMaxMapSize);
    // this->positions = Vector<coord>();
}

int Map::getIndex(const coord &position) const {
    for (uint8_t i = 0; i < this->positions.size(); ++i) {
        if (this->positions[i] == position) {
            return i;
        }
    }
    return kInvalidIndex;
}