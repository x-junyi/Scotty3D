
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

#include <cmath>

namespace {

const float pi = std::atan(1.0f) * 4.0f;

}

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute the position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: Compute the ray direction in camera space and use
    // the camera transform to transform it back into world space.

    float sh = 2.0f * std::tan(vert_fov * pi / 360.0f);
    auto sw = sh * aspect_ratio;

    auto sx = sw * (screen_coord[0] - 0.5f);
    auto sy = sh * (screen_coord[1] - 0.5f);

    Ray ray(Vec3(0, 0, 0), Vec3(sx, sy, -1.0f).unit());
    ray.transform(iview);

    return ray;
}
