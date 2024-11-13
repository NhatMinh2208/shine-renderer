#include <nori/texture.h>
#include <nori/bitmap.h>
#include <filesystem/resolver.h>

NORI_NAMESPACE_BEGIN
class BitmapTexture : public Texture<Color3f> {
    public:
    BitmapTexture (const PropertyList &props){
        m_filename = props.getString("filename");
        filesystem::path path = getFileResolver()->resolve(m_filename);
        m_tex = new Bitmap(path.str());
        m_height = m_tex->rows();
        m_width = m_tex->cols();
    }

    Color3f eval(const Intersection &its) const {
        Point2f uv = its.uv;
        Point2i index = Point2i(int(uv[0] * m_width), int(uv[1] * m_height));
        return (*m_tex)(index[0], index[1]);
    }

    std::string toString() const {
        return tfm::format(
            "BitmapTexture[\n"
            "  m_filename = %s\n"
            "]",
            m_filename
        );
    }
    private:
        Bitmap* m_tex;
        std::string m_filename;
        uint16_t m_height, m_width;

};

NORI_REGISTER_CLASS(BitmapTexture, "bitmap");
NORI_NAMESPACE_END