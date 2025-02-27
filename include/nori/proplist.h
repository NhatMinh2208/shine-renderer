/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/color.h>
#include <nori/transform.h>
#include <map>

NORI_NAMESPACE_BEGIN

/**
 * \brief This is an associative container used to supply the constructors
 * of \ref NoriObject subclasses with parameter information.
 */
class PropertyList {
public:
    PropertyList() { }

    /// Set a boolean property
    void setBoolean(const std::string &name, const bool &value);
    
    /// Get a boolean property, and throw an exception if it does not exist
    bool getBoolean(const std::string &name) const;

    /// Get a boolean property, and use a default value if it does not exist
    bool getBoolean(const std::string &name, const bool &defaultValue) const;

    /// Set an integer property
    void setInteger(const std::string &name, const int &value);
    
    /// Get an integer property, and throw an exception if it does not exist
    int getInteger(const std::string &name) const;

    /// Get am integer property, and use a default value if it does not exist
    int getInteger(const std::string &name, const int &defaultValue) const;

    /// Set a float property
    void setFloat(const std::string &name, const float &value);
    
    /// Get a float property, and throw an exception if it does not exist
    float getFloat(const std::string &name) const;

    /// Get a float property, and use a default value if it does not exist
    float getFloat(const std::string &name, const float &defaultValue) const;

    /// Set a string property
    void setString(const std::string &name, const std::string &value);

    /// Get a string property, and throw an exception if it does not exist
    std::string getString(const std::string &name) const;

    /// Get a string property, and use a default value if it does not exist
    std::string getString(const std::string &name, const std::string &defaultValue) const;

    /// Set a color property
    void setColor(const std::string &name, const Color3f &value);

    /// Get a color property, and throw an exception if it does not exist
    Color3f getColor(const std::string &name) const;

    /// Get a color property, and use a default value if it does not exist
    Color3f getColor(const std::string &name, const Color3f &defaultValue) const;

    /// Set a point property
    void setPoint(const std::string &name, const Point3f &value);

    /// Get a point property, and throw an exception if it does not exist
    Point3f getPoint(const std::string &name) const;

    /// Get a point property, and use a default value if it does not exist
    Point3f getPoint(const std::string &name, const Point3f &defaultValue) const;

    /// Set a vector property
    void setVector(const std::string &name, const Vector3f &value);

    /// Get a vector property, and throw an exception if it does not exist
    Vector3f getVector(const std::string &name) const;

    /// Get a vector property, and use a default value if it does not exist
    Vector3f getVector(const std::string &name, const Vector3f &defaultValue) const;

    /// Set a transform property
    void setTransform(const std::string &name, const Transform &value);

    /// Get a transform property, and throw an exception if it does not exist
    Transform getTransform(const std::string &name) const;

    /// Get a transform property, and use a default value if it does not exist
    Transform getTransform(const std::string &name, const Transform &defaultValue) const;

    /// Check if property have the specific type
    bool hasBoolean(const std::string &name, bool &val) const;
    bool hasInteger(const std::string &name, int &val) const;
    bool hasFloat(const std::string &name, float &val) const;
    bool hasColor(const std::string &name, Color3f &val) const;
    bool hasPoint(const std::string &name, Point3f &val) const;
    bool hasVector(const std::string &name, Vector3f &val) const;
    bool hasString(const std::string &name, std::string &val) const;
    bool hasTransform(const std::string &name, Transform &val) const;
private:
    /* Custom variant data type (stores one of boolean/integer/float/...) */
    struct Property {
        enum {
            boolean_type, integer_type, float_type,
            string_type, color_type, point_type,
            vector_type, transform_type
        } type;

        /* Visual studio lacks support for unrestricted unions (as of ver. 2013) */
        struct Value
        {
            Value() : boolean_value(false) { }
            ~Value() { }

            bool boolean_value;
            int integer_value;
            float float_value;
            std::string string_value;
            Color3f color_value;
            Point3f point_value;
            Vector3f vector_value;
            Transform transform_value;
        } value;

        Property() : type(boolean_type) { }
    };

    std::map<std::string, Property> m_properties;
};

NORI_NAMESPACE_END
