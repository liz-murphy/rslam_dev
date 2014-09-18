#include <common_front_end/Feature.h>

Feature& Feature::operator=(const Feature &rhs) {
    // check for self assignment
    if (this == &rhs) {
        return *this;
    }

    scale = rhs.scale;
    orientation = rhs.orientation;
    // copy data
    _Copy(rhs.id, rhs.x, rhs.y, rhs.response,
          &rhs.descriptor[0], rhs.descsize, rhs.used);

    return *this;
}
