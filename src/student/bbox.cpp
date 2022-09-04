
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // TODO (PathTracer): Task 3
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    auto invdir = 1.0f / ray.dir;
    int sign[3]{invdir.x < 0, invdir.y < 0, invdir.z < 0};
    float tymin, tymax, tzmin, tzmax;
    auto& tmin = times[0];
    auto& tmax = times[1];
    const Vec3* bounds[]{&min, &max};

    tmin = (bounds[sign[0]]->x - ray.point.x) * invdir.x;
    tmax = (bounds[1 - sign[0]]->x - ray.point.x) * invdir.x;
    tymin = (bounds[sign[1]]->y - ray.point.y) * invdir.y;
    tymax = (bounds[1 - sign[1]]->y - ray.point.y) * invdir.y;

    if((tmin > tymax) || (tymin > tmax)) return false;

    if(tymin > tmin) tmin = tymin;
    if(tymax < tmax) tmax = tymax;

    tzmin = (bounds[sign[2]]->z - ray.point.z) * invdir.z;
    tzmax = (bounds[1 - sign[2]]->z - ray.point.z) * invdir.z;

    if((tmin > tzmax) || (tzmin > tmax)) return false;

    if(tzmin > tmin) tmin = tzmin;
    if(tzmax < tmax) tmax = tzmax;

    return true;
}
