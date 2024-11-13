#pragma once

#include <nori/object.h>
#include <nori/mesh.h>
NORI_NAMESPACE_BEGIN
template <typename T>
class Texture : public NoriObject {
    public:
    virtual T eval(const Intersection &its) const = 0;
    /**
     * \brief Compute the texture value at intersection point
     *
     * \param its
     *     A record of the surface intersection
     *
     * \return
     *     A texture value at intersection point
     */

    EClassType getClassType() const { return ETexture; }
};
NORI_NAMESPACE_END