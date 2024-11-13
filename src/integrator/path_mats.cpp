#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
NORI_NAMESPACE_BEGIN
class MaterialSampling : public Integrator {
    public:
    MaterialSampling(const PropertyList &props){
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        Ray3f ray_ = ray; //Place holder
        int bounces = 0;
        int maxDepth = 50;
        Color3f beta = Color3f(1.f), L = Color3f(0.f);
        Vector3f wi, wo;
        float eta = 1.f;
        float RR = 1.f;
        while(true){
            if (bounces >= maxDepth) break;
            wi = -ray_.d.normalized();
            if (!scene->rayIntersect(ray_, its))
                break;

            if (its.mesh->isEmitter() && its.shFrame.n.dot(wi) > 0){
                EmitterQueryRecord eRec_(its.p, its.shFrame.n, 1.f);
                L += its.mesh->getEmitter()->eval(eRec_, -wi) * beta;
            }
            auto bsdf = its.mesh->getBSDF();
            BSDFQueryRecord bRec(its.shFrame.toLocal(wi));
            beta *= bsdf->sample(bRec, sampler->next2D());
            eta *= bRec.eta;
            ray_ =  Ray3f(its.p, its.shFrame.toWorld(bRec.wo)); //using the Ray3f constructor automatically update the inverse direction
            // ray_.o = its.p;
            // ray_.d = its.shFrame.toWorld(bRec.wo);
            if(bounces >= 3){
                RR = std::min(beta.maxCoeff() * eta * eta, 0.99f);
                if (sampler->next1D() > RR) break;
                beta /= RR;
            }
            bounces++;
        }
        return L;
     }

    std::string toString() const {
        return "MaterialSampling[]";   
    }
    protected:
};

NORI_REGISTER_CLASS(MaterialSampling, "path_mats");
NORI_NAMESPACE_END