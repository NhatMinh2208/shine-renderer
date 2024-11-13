#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN
class MISPathTracer : public Integrator {
    public:
    MISPathTracer(const PropertyList &props){
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
        float lightPdf = 1.f;
        bool specularBounce = false;
        Intersection prevIts;
        float prev_bpdf = 1.f;
        while(true){
            if (bounces >= maxDepth) break;
            wi = -ray_.d.normalized();
            if (!scene->rayIntersect(ray_, its))
                break;
            
            if (its.mesh->isEmitter() && its.shFrame.n.dot(wi) > 0){
                EmitterQueryRecord eRec_(its.p, its.shFrame.n, 1.f / its.mesh->surfaceArea());
                if((bounces == 0 || specularBounce)) {
                    L += its.mesh->getEmitter()->eval(eRec_, -wi) * beta; // don't account for mis term since we don't do 
                                                                        // light sampling at vertex or specular bsdf
                }
                else {
                    eRec_.pdf *= ((eRec_.p - prevIts.p).squaredNorm() / eRec_.n.dot(wi) ); //area -> solid angle
                    //auto lpdf = eRec_.pdf * scene->emitterPDF(its.mesh);
                    lightPdf = (scene->getLights().size() != 0) ? 1.f / scene->getLights().size() : 1.f;
                    auto lpdf = eRec_.pdf * lightPdf;
                    float w_b = (prev_bpdf + lpdf != 0.f) ? prev_bpdf / (prev_bpdf + lpdf) : 0.f;
                    L += w_b * its.mesh->getEmitter()->eval(eRec_, -wi) * beta;
                }
            }
            

            //Get bsdf
            auto bsdf = its.mesh->getBSDF();
            if(!bsdf->isDiffuse()) specularBounce = true;
            //Sample light
            else{
                specularBounce = false;
                //auto light = scene->sampleEmitter(sampler->next1D(), lightPdf);
                int randomEmitterIdx = static_cast<int>(floor(sampler->next1D() * scene->getLights().size()));
                auto light = scene->getLights()[randomEmitterIdx]->getEmitter();
                EmitterQueryRecord eRec;
                auto Le = light->sample(eRec, its.p, sampler);
                Vector3f wos = (eRec.p - its.p).normalized(); //shadow ray
                Vector3f wis = -ray_.d.normalized();
                auto bRec_ = BSDFQueryRecord(its.shFrame.toLocal(wis), its.shFrame.toLocal(wos), ESolidAngle);
                auto f = bsdf->eval(bRec_, its);
                auto V = Color3f(1.f);
                if(scene->rayIntersect(Ray3f(its.p, wos, Epsilon, (eRec.p - its.p).norm() - Epsilon))) V = Color3f(0.f);
                auto bpdf = bsdf->pdf(bRec_, its);  
        
                if (eRec.n.dot(-wos) > 0.f){
                    lightPdf = (scene->getLights().size() != 0) ? 1.f / scene->getLights().size() : 0.f;
                    
                    eRec.pdf = eRec.pdf * lightPdf * (eRec.p - its.p).squaredNorm() / eRec.n.dot(-wos);// change-of-variable area -> solod angle
                    float w_l = (eRec.pdf + bpdf != 0.f) ?  eRec.pdf  / (eRec.pdf + bpdf) : 0.f;
                    L += w_l * ((V * f * Le * its.shFrame.n.dot(wos)) / eRec.pdf) * beta;
                }                
            }
            
            //Terminate the path sooner using Russian Roulette
            if(bounces >= 3){
                RR = std::min(beta.maxCoeff() * eta, 0.99f);
                if (sampler->next1D() > RR) break;
                beta /= RR;
            }
            //Sample new direction
            BSDFQueryRecord bRec(its.shFrame.toLocal(wi));
            beta *= bsdf->sample(bRec, its, sampler->next2D());
            eta *= bRec.eta * bRec.eta;
            prev_bpdf = bsdf->pdf(bRec, its);
            prevIts = its;
            ray_ =  Ray3f(its.p, its.shFrame.toWorld(bRec.wo)); 
            bounces++;
        }
        return L;
     }

    std::string toString() const {
        return "MISPathTracer[]";   
    }
    protected:
};

NORI_REGISTER_CLASS(MISPathTracer, "path_mis");
NORI_NAMESPACE_END