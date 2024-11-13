#include <nori/texture.h>
NORI_NAMESPACE_BEGIN
class CheckerBoard : public Texture<Color3f> {
    public:
    CheckerBoard (const PropertyList &props){
    }
    void activate(){
        if (!m_odd)
            m_odd = static_cast<Texture<Color3f> *>(NoriObjectFactory::createInstance("constant", PropertyList()));
        if (!m_even)
            m_even = static_cast<Texture<Color3f> *>(NoriObjectFactory::createInstance("constant", PropertyList()));            
    }
    Color3f eval(const Intersection &its) const {
        Point2f uv = its.uv * 20;
        if (((int)std::floor(uv[0]) + (int)std::floor(uv[1])) % 2 == 0)
            return m_even->eval(its);
        return m_odd->eval(its);
    }

    void addChild(NoriObject *obj) {
        switch (obj->getClassType()) {
            case ETexture: {
                    if (m_odd && m_even)
                        throw NoriException("There can only be two textures in checkboard !");
                    else{
                        if(!m_odd){
                            m_odd = static_cast<Texture<Color3f> *>(obj);
                        }
                        else if(!m_even){
                            m_even = static_cast<Texture<Color3f> *>(obj);
                        }
                    }
                    
                }
                break;
            default:
                throw NoriException("Scene::addChild(<%s>) is not supported!",
                    classTypeName(obj->getClassType()));
        }
    }

    std::string toString() const {
        return tfm::format(
            "CheckerBoardTexture[\n"
            "  odd = %s\n"
            "  even = %s\n"
            "]",
            m_odd->toString(),
            m_even->toString()
        );
    }
    private:
        Texture<Color3f> *m_odd = nullptr, *m_even = nullptr;
};

NORI_REGISTER_CLASS(CheckerBoard, "checkerboard");
NORI_NAMESPACE_END