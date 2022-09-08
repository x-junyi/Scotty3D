
#include "../rays/bsdf.h"
#include "../util/rand.h"

namespace PT {

static Vec3 reflect(Vec3 dir) {

    // TODO (PathTracer): Task 5
    // Return reflection of dir about the surface normal (0,1,0).
    return Vec3{-dir.x, std::abs(dir.y), -dir.z};
}

static Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {

    // TODO (PathTracer): Task 5
    // Use Snell's Law to refract out_dir through the surface.
    // Return the refracted direction. Set was_internal to true if
    // refraction does not occur due to total internal reflection,
    // and false otherwise.

    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to a
    // ray exiting the surface into vaccum (ior = 1). However, note that
    // you should actually treat this case as _entering_ the surface, because
    // you want to compute the 'input' direction that would cause this output,
    // and to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.

    was_internal = false;

    auto ni = 1.0f;
    auto nt = index_of_refraction;
    if(out_dir.y < 0) std::swap(ni, nt);

    auto cos2_theta_i = out_dir.y * out_dir.y;

    auto sin2_theta_t = (ni / nt) * (ni / nt) * (1 - cos2_theta_i);
    auto cos2_theta_t = 1.0f - sin2_theta_t;

    if(cos2_theta_t < 0) {
        was_internal = true;
        return reflect(out_dir);
    }

    Vec3 in_dir(-out_dir.x, 0.0f, -out_dir.z);
    in_dir.normalize();
    in_dir *= std::sqrt(sin2_theta_t);
    in_dir.y = std::sqrt(cos2_theta_t);
    if(0 < out_dir.y) in_dir.y = -in_dir.y;

    return in_dir;
}

Scatter BSDF_Lambertian::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 4

    // Sample the BSDF distribution using the cosine-weighted hemisphere sampler.
    // You can use BSDF_Lambertian::evaluate() to compute attenuation.

    Scatter ret;
    ret.direction = sampler.sample();
    ret.attenuation = evaluate(out_dir, ret.direction);
    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {

    // TODO (PathTracer): Task 4

    // Compute the ratio of reflected/incoming radiance when light from in_dir
    // is reflected through out_dir: albedo * cos(theta).

    return 0 < in_dir.y ? albedo * in_dir.y : Spectrum{};
}

float BSDF_Lambertian::pdf(Vec3 out_dir, Vec3 in_dir) const {

    // TODO (PathTracer): Task 4

    // Compute the PDF for sampling in_dir from the cosine-weighted hemisphere distribution.
    return in_dir.y / PI_F;
}

Scatter BSDF_Mirror::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5

    Scatter ret;
    ret.direction = reflect(out_dir);
    ret.attenuation = reflectance;
    return ret;
}

Scatter BSDF_Glass::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5

    // (1) Compute Fresnel coefficient. Tip: Schlick's approximation.
    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
    // What happens upon total internal reflection?

    Scatter ret;
    bool was_internal{};
    ret.direction = refract(out_dir, index_of_refraction, was_internal);
    if(was_internal) {
        ret.attenuation = reflectance;
        return ret;
    }

    auto ni = 1.0f;
    auto nt = index_of_refraction;
    if(out_dir.y < 0) std::swap(ni, nt);

    auto cos_theta_i = std::abs(out_dir.y);
    auto cos_theta_t = std::abs(ret.direction.y);

    auto r_para = (nt * cos_theta_i - ni * cos_theta_t) / (nt * cos_theta_i + ni * cos_theta_t);
    auto r_perp = (ni * cos_theta_i - nt * cos_theta_t) / (ni * cos_theta_i + nt * cos_theta_t);
    auto F_r = 0.5f * (r_para * r_para + r_perp * r_perp);
    ret.attenuation = transmittance;

    if(RNG::coin_flip(F_r)) {
        ret.attenuation = reflectance;
        ret.direction = reflect(out_dir);
    }

    return ret;
}

Scatter BSDF_Refract::scatter(Vec3 out_dir) const {

    // OPTIONAL (PathTracer): Task 5

    // When debugging BSDF_Glass, it may be useful to compare to a pure-refraction BSDF

    Scatter ret;
    bool was_internal{};
    ret.direction = refract(out_dir, index_of_refraction, was_internal);
    if(was_internal) {
        ret.attenuation = Spectrum(1.0f, 1.0f, 1.0f);
        return ret;
    }

    //    auto ni = 1.0f;
    //    auto nt = index_of_refraction;
    //    if(out_dir.y < 0) std::swap(ni, nt);
    //
    //    auto cos_theta_i = std::abs(out_dir.y);
    //    auto cos_theta_t = std::abs(ret.direction.y);
    //
    //    auto r_para = (nt * cos_theta_i - ni * cos_theta_t) / (nt * cos_theta_i + ni *
    //    cos_theta_t); auto r_perp = (ni * cos_theta_i - nt * cos_theta_t) / (ni * cos_theta_i + nt
    //    * cos_theta_t); auto F_r = 0.5f * (r_para * r_para + r_perp * r_perp);
    ret.attenuation = transmittance;
    //    ret.attenuation = transmittance * (ni * ni / nt / nt * (1.0f - F_r)) / cos_theta_t;

    return ret;
}

Spectrum BSDF_Diffuse::emissive() const {
    return radiance;
}

} // namespace PT
