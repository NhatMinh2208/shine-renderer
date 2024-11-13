#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
NORI_NAMESPACE_BEGIN
class EmitterSampling : public Integrator {
    public:
    EmitterSampling(const PropertyList &props){
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
        float lightPdf;
        bool specularBounce = false;
        while(true){
            if (bounces >= maxDepth) break;
            wi = -ray_.d.normalized();
            if (!scene->rayIntersect(ray_, its))
                break;
            //only add the light radiance (when we randomly intersect a light surface) 
            // when it came to the first ray and specular case
            // since both case we can not sample emitter (pdf of specular bsdf is zero and we don't sample emitter at camera).
            if(bounces == 0 || specularBounce){
                if (its.mesh->isEmitter() && its.shFrame.n.dot(wi) > 0){
                    EmitterQueryRecord eRec_(its.p, its.shFrame.n, 1.f);
                    L += its.mesh->getEmitter()->eval(eRec_, -wi) * beta;
                }
            }

            //Get bsdf
            auto bsdf = its.mesh->getBSDF();
            if(!bsdf->isDiffuse()) specularBounce = true;
            //Sample light
            else{
                specularBounce = false;
                auto light = scene->sampleEmitter(sampler->next1D(), lightPdf);
                EmitterQueryRecord eRec;
                auto Le = light->sample(eRec, its.p, sampler);
                Vector3f wos = (eRec.p - its.p).normalized(); //shadow ray
                Vector3f wis = -ray_.d.normalized();
                auto f = bsdf->eval(BSDFQueryRecord(its.shFrame.toLocal(wis), its.shFrame.toLocal(wos), ESolidAngle));
                float G = abs(wos.dot(its.shFrame.n)) * abs(eRec.n.dot(-wos)) / (eRec.p - its.p).squaredNorm();
                auto V = Color3f(1.f);
                if(scene->rayIntersect(Ray3f(its.p, wos, Epsilon, (eRec.p - its.p).norm() - Epsilon))) V = Color3f(0.f);
                L += ((V * f * G * Le) /  (eRec.pdf * lightPdf)) * beta;
            }
            
            //Terminate the path sooner
            if(bounces >= 3){
                RR = std::min(beta.maxCoeff() * eta * eta, 0.99f);
                if (sampler->next1D() > RR) break;
                beta /= RR;
            }
            //Sample new direction
            BSDFQueryRecord bRec(its.shFrame.toLocal(wi));
            beta *= bsdf->sample(bRec, sampler->next2D());
            eta *= bRec.eta;
            ray_ =  Ray3f(its.p, its.shFrame.toWorld(bRec.wo)); 
            bounces++;
        }
        return L;
     }

    std::string toString() const {
        return "EmitterSampling[]";   
    }
    protected:
};

NORI_REGISTER_CLASS(EmitterSampling, "path_ems");
NORI_NAMESPACE_END