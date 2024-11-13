#include <nori/texture.h>
NORI_NAMESPACE_BEGIN
class ConstantTexture : public Texture<Color3f> {
    public:
    ConstantTexture (const PropertyList &props){
        m_val = props.getColor("value", Color3f(0.5f));
    }
    ConstantTexture (float val){
        m_val = Color3f(val);
    }

    Color3f eval(const Intersection &its) const {
        return m_val;
    }
    std::string toString() const {
        return tfm::format(
            "ConstantTexture[\n"
            "  value = %s\n"
            "]",
            m_val.toString()
        );
    }
    private:
        Color3f m_val;
};


// class FloatConstantTexture : public Texture<float> {
//     public:
//     FloatConstantTexture (const PropertyList &props){
//         m_val = props.getFloat("value");
//     }
//     float eval(const Intersection &its) const {
//         return m_val;
//     }
//     std::string toString() const {
//         return tfm::format(
//             "ConstantTexture[\n"
//             "  value = %f\n"
//             "]",
//             m_val
//         );
//     }
//     private:
//         float m_val;
// };

// class ColorConstantTexture : public Texture<Color3f> {
//     public:
//     ColorConstantTexture (const PropertyList &props){
//         m_val = props.getColor("value");
//     }
//     Color3f eval(const Intersection &its) const {
//         return m_val;
//     }
//     std::string toString() const {
//         return tfm::format(
//             "ConstantTexture[\n"
//             "  value = %s\n"
//             "]",
//             m_val.toString()
//         );
//     }
//     private:
//         Color3f m_val;
// };


NORI_REGISTER_CLASS(ConstantTexture, "constant");
NORI_NAMESPACE_END