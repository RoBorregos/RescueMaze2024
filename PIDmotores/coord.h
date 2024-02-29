struct coord{
    int x;
    int y;
    int z;
    bool const operator==(const coord &o) const {
        return x == o.x && y == o.y && z == o.z;
    }
    bool const operator!=(const coord &o) const {
        return x != o.x || y != o.y || z != o.z;
    }
    bool const operator<(const coord &o) const {
        return x < o.x || (x == o.x && y < o.y) || (x == o.x && y == o.y && z < o.z);
    }
};

// namespace std{
//     template<>
//     struct hash<coord>{
//         size_t operator()(const coord& c) const{
//             return hash<int>()(c.x)^hash<int>()(c.y)^hash<int>()(c.z);
//         }
//     };
// }