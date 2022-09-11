
#include "../rays/env_light.h"

#include <limits>

namespace PT {

Vec3 Env_Map::sample() const {

    // TODO (PathTracer): Task 7

    // First, implement Samplers::Sphere::Uniform so the following line works.
    // Second, implement Samplers::Sphere::Image and swap to image_sampler

    return image_sampler.sample();
}

float Env_Map::pdf(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // First, return the pdf for a uniform spherical distribution.
    // Second, swap to image_sampler.pdf().

    return image_sampler.pdf(dir);
}

Spectrum Env_Map::evaluate(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // Compute emitted radiance along a given direction by finding the corresponding
    // pixels in the enviornment image. You should bi-linearly interpolate the value
    // between the 4 nearest pixels.

    dir.y = -dir.y;
    auto theta = std::acos(dir.y);
    auto phi = std::atan2(dir.z, dir.x);

    if(phi < 0.0f) phi += 2.0f * PI_F;

    auto [w, h] = image.dimension();
    auto x = phi / (2.0f * PI_F) * float(w);
    auto y = theta / PI_F * float(h);

    auto x1 = size_t(floor(x + 0.5f));
    auto y1 = size_t(floor(y + 0.5f));
    x1 = std::clamp(x1, size_t(1), w - 1);
    y1 = std::clamp(y1, size_t(1), h - 1);

    auto x0 = x1 - 1;
    auto y0 = y1 - 1;

    auto tx = x - float(x0);
    auto ty = y - float(y0);
    tx = std::clamp(tx, 0.0f, 1.0f);
    ;
    ty = std::clamp(ty, 0.0f, 1.0f);

    return (1.0f - ty) * ((1.0f - tx) * image.at(x0, y0) + tx * image.at(x1, y0)) +
           ty * ((1.0f - tx) * image.at(x0, y1) + tx * image.at(x1, y1));
}

Vec3 Env_Hemisphere::sample() const {
    return sampler.sample();
}

float Env_Hemisphere::pdf(Vec3 dir) const {
    return 1.0f / (2.0f * PI_F);
}

Spectrum Env_Hemisphere::evaluate(Vec3 dir) const {
    if(dir.y > 0.0f) return radiance;
    return {};
}

Vec3 Env_Sphere::sample() const {
    return sampler.sample();
}

float Env_Sphere::pdf(Vec3 dir) const {
    return 1.0f / (4.0f * PI_F);
}

Spectrum Env_Sphere::evaluate(Vec3) const {
    return radiance;
}

} // namespace PT
