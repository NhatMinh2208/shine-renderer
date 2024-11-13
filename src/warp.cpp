/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

float tentSampling(float u){
    if(u >= 0 && u < 0.5){
        return sqrt(2 * u) - 1;
    }
    else if(u >= 0.5 && u < 1){
        return 1 - sqrt(2 - 2 * u);
    }
    else return 0.f;
}
Point2f Warp::squareToTent(const Point2f &sample) {
    //throw NoriException("Warp::squareToTent() is not yet implemented!");
    return Point2f(tentSampling(sample[0]), tentSampling(sample[1]));
}

float Warp::squareToTentPdf(const Point2f &p) {
    //throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
    float p0, p1;
    if(p[0] >= -1 && p[0] <= 1){
        p0 = 1 - abs(p[0]);
    } else p0 = 0.f;
    if(p[1] >= -1 && p[1] <= 1){
        p1 = 1 - abs(p[1]);
    } else p1 = 0.f;
    return p0 * p1;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    // throw NoriException("Warp::squareToUniformDisk() is not yet implemented!");
    float r = sqrt(sample[0]);
    float theta = 2 * M_PI * sample[1];
    return Point2f(r * cos(theta), r * sin(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    // throw NoriException("Warp::squareToUniformDiskPdf() is not yet implemented!");
    return p.squaredNorm() <= 1.f ? INV_PI : 0.f;
    
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    // throw NoriException("Warp::squareToUniformSphere() is not yet implemented!");
    float phi = 2 * M_PI * sample[0];
    float z = 1 - 2 * sample[1];
    float sinTheta = sqrt(1 - z * z);
    return Vector3f(sinTheta * cos(phi), sinTheta * sin(phi), z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    // throw NoriException("Warp::squareToUniformSpherePdf() is not yet implemented!");
    return v.squaredNorm() <= 1.f ? INV_FOURPI : 0.f;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    // throw NoriException("Warp::squareToUniformHemisphere() is not yet implemented!");
    float phi = 2 * M_PI * sample[0];
    float z = 1 - sample[1];
    float sinTheta = sqrt(1 - z * z);
    return Vector3f(sinTheta * cos(phi), sinTheta * sin(phi), z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    // throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
    return (v.squaredNorm() <= 1.f && v[2] >= 0.f ) ? INV_TWOPI : 0.f;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    // throw NoriException("Warp::squareToCosineHemisphere() is not yet implemented!");
    Point2f p = Warp::squareToUniformDisk(sample);
    float z = sqrt(1.f - (p[0] * p[0] + p[1] * p[1]));
    return Vector3f(p[0], p[1], z);
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    // throw NoriException("Warp::squareToCosineHemispherePdf() is not yet implemented!");
    // return (v.squaredNorm() <= 1.f && v[2] >= 0.f ) ? v[2] * INV_PI : 0.f;
     return  v[2] >= 0.f ? v[2] * INV_PI : 0.f;
     // add v.squaredNorm() <= 1.f  and it's incorrect
    
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    // throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
    float phi = 2 * M_PI * sample[0];
    float theta = atan(alpha * sqrt(-log(1.f - sample[1])));
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    // throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
    Frame frame;
    float tanTheta = frame.tanTheta(m);
    float tanTheta2 = tanTheta * tanTheta;
    float cosTheta = frame.cosTheta(m);
    float cosTheta3 = cosTheta * cosTheta * cosTheta;
    float alpha2 = alpha * alpha;
    return m[2] > 0.f ? INV_PI * exp(-tanTheta2 / alpha2) / (alpha2 * cosTheta3) : 0.f;
    // can't run with m[2] >= 0.f
}

NORI_NAMESPACE_END
