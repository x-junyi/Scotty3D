
#include "../rays/samplers.h"
#include "../util/rand.h"

namespace Samplers {

Vec2 Rect::sample() const {

    // TODO (PathTracer): Task 1

    // Generate a uniformly random point on a rectangle of size size.x * size.y
    // Tip: RNG::unit()

    return Vec2{RNG::unit() * size[0], RNG::unit() * size[1]};
}

Vec3 Sphere::Uniform::sample() const {

    // TODO (PathTracer): Task 7

    // Generate a uniformly random point on the unit sphere.
    // Tip: start with Hemisphere::Uniform

    auto dir = hemi.sample();
    if(RNG::coin_flip(0.5f)) dir.y = -dir.y;

    return dir;
}

Sphere::Image::Image(const HDR_Image& image) {

    // TODO (PathTracer): Task 7

    // Set up importance sampling data structures for a spherical environment map image.
    // You may make use of the _pdf, _cdf, and total members, or create your own.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;

    auto n = w * h;
    _pdf.resize(n);
    _cdf.resize(n, 0.0f);

    for(size_t j = 0; j < h; ++j) {
        for(size_t i = 0; i < w; ++i) {
            _pdf[j * w + i] = image.at(i, j).luma();
            total += _pdf[j * w + i];
        }
    }

    for(size_t j = 0; j < h; ++j) {
        for(size_t i = 0; i < w; ++i) {
            auto idx = j * w + i;
            _pdf[idx] /= total;
            _cdf[idx] = _pdf[idx];
            if(idx) {
                _cdf[idx] += _cdf[idx - 1];
            }
        }
    }
}

Vec3 Sphere::Image::sample() const {

    // TODO (PathTracer): Task 7

    // Use your importance sampling data structure to generate a sample direction.
    // Tip: std::upper_bound

    auto xi = RNG::unit();
    size_t pixel_idx = std::upper_bound(_cdf.begin(), _cdf.end(), xi) - _cdf.begin();
    pixel_idx = std::clamp(pixel_idx, size_t(0), _cdf.size() - 1);
    auto x = pixel_idx % w;
    auto y = pixel_idx / w;
    auto phi = (float(x) + 0.5f) / float(w) * 2.0f * PI_F;
    auto theta = (float(y) + 0.5f) / float(h) * PI_F;

    return Vec3{std::cos(phi) * std::sin(theta), -std::cos(theta), std::sin(phi) * std::sin(theta)};
}

float Sphere::Image::pdf(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // What is the PDF of this distribution at a particular direction?

    dir.y = -dir.y;
    auto theta = std::acos(dir.y);
    auto phi = std::atan2(dir.z, dir.x);

    if(phi < 0.0f) phi += 2.0f * PI_F;

    auto x = size_t(phi / (2.0f * PI_F) * float(w));
    auto y = size_t(theta / PI_F * float(h));
    x = std::clamp(x, size_t(0), w - 1);
    y = std::clamp(y, size_t(0), h - 1);

    auto jacobian = float(w * h) / (2.0f * PI_F * PI_F * std::sin(theta));

    return _pdf[y * w + x] * jacobian;
}

Vec3 Point::sample() const {
    return point;
}

Vec3 Triangle::sample() const {
    float u = std::sqrt(RNG::unit());
    float v = RNG::unit();
    float a = u * (1.0f - v);
    float b = u * v;
    return a * v0 + b * v1 + (1.0f - a - b) * v2;
}

Vec3 Hemisphere::Uniform::sample() const {

    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    return Vec3(xs, ys, zs);
}

Vec3 Hemisphere::Cosine::sample() const {

    float phi = RNG::unit() * 2.0f * PI_F;
    float cos_t = std::sqrt(RNG::unit());

    float sin_t = std::sqrt(1 - cos_t * cos_t);
    float x = std::cos(phi) * sin_t;
    float z = std::sin(phi) * sin_t;
    float y = cos_t;

    return Vec3(x, y, z);
}

} // namespace Samplers
