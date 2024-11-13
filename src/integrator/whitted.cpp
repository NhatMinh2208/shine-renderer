#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
NORI_NAMESPACE_BEGIN
class WhittedIntegrator : public Integrator {
    public:
    WhittedIntegrator(const PropertyList &props){
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        Ray3f ray_ = ray; //Place holder
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        Color3f L = Color3f(0.f);

        if (its.mesh->isEmitter()){
            EmitterQueryRecord eRec_(its.p, its.shFrame.n, 1.f);
            L += its.mesh->getEmitter()->eval(eRec_, ray_.d);
        }


        float lightPdf;
        auto light = scene->sampleEmitter(sampler->next1D(), lightPdf);
        
        EmitterQueryRecord eRec;
        auto Le = light->sample(eRec, its.p, sampler);

        auto bsdf = its.mesh->getBSDF();
        if(bsdf->isDiffuse()){
            Vector3f wo = (eRec.p - its.p).normalized();
            Vector3f wi = -ray_.d.normalized();
            BSDFQueryRecord bRec(its.shFrame.toLocal(wi), its.shFrame.toLocal(wo), ESolidAngle);
            auto f = bsdf->eval(bRec);

            float G = abs(wo.dot(its.shFrame.n)) * abs(eRec.n.dot(-wo)) / (eRec.p - its.p).squaredNorm();
            auto V = Color3f(1.f);
            if(scene->rayIntersect(Ray3f(its.p, wo, Epsilon, (eRec.p - its.p).norm() - Epsilon))) V = Color3f(0.f);
            //i don't know why but if not add epsilon everything fuk up
            L += (V * f * G * Le) /  (eRec.pdf * lightPdf);

            return L;
        }
        else {
            Vector3f wi = -ray_.d.normalized();
            BSDFQueryRecord bRec(its.shFrame.toLocal(wi));
            auto c = bsdf->sample(bRec, sampler->next2D());
            if (sampler->next1D() < 0.95){
                return c * 1.f/0.95 *  Li(scene, sampler, Ray3f(its.p, its.toWorld(bRec.wo.normalized())));
            }
            else return Color3f(0.f);
        };

    }

    std::string toString() const {
        return "WhittedIntegrator[]";   
    }
    protected:
    //std::vector<Mesh*> lights;
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END