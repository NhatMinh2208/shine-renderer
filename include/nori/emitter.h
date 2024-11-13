/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN


struct EmitterQueryRecord {
     /// Create a new record for sampling the Emitter
    EmitterQueryRecord() { }

    /// Create a new record for querying the Emitter
    EmitterQueryRecord(Point3f p, Normal3f n, float pdf) : p(p), n(n), pdf(pdf) {}

    Point3f p;
    Normal3f n;
    float pdf;
};

/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:
    virtual Color3f sample(EmitterQueryRecord &eRec, const Point3f p, Sampler* sampler) const = 0;

    /**
     * \brief Evaluate the BSDF for a pair of directions and measure
     * specified in \code bRec
     *
     * \param bRec
     *     A record with detailed information on the BSDF query
     * \return
     *     The BSDF value, evaluated for each color channel
     */
    virtual Color3f eval(const EmitterQueryRecord &eRec, const Vector3f wi) const = 0;

    /**
     * \brief Compute the probability of sampling \c bRec.wo
     * (conditioned on \c bRec.wi).
     *
     * This method provides access to the probability density that
     * is realized by the \ref sample() method.
     *
     * \param bRec
     *     A record with detailed information on the BSDF query
     *
     * \return
     *     A probability/density value expressed with respect
     *     to the specified measure
     */
    virtual float pdf(const EmitterQueryRecord &eRec) const = 0;

    /**
     * \brief Return the type of object (i.e. Mesh/BSDF/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EEmitter; }

    void setMesh(Mesh* m){
        mesh = m;
    }

    protected:
    Mesh* mesh = nullptr;
};

NORI_NAMESPACE_END
