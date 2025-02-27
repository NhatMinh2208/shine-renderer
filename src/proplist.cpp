/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/proplist.h>

NORI_NAMESPACE_BEGIN

#define DEFINE_PROPERTY_ACCESSOR(Type, TypeName, XmlName) \
    void PropertyList::set##TypeName(const std::string &name, const Type &value) { \
        if (m_properties.find(name) != m_properties.end()) \
            cerr << "Property \"" << name <<  "\" was specified multiple times!" << endl; \
        auto &prop = m_properties[name]; \
        prop.value.XmlName##_value = value; \
        prop.type = Property::XmlName##_type; \
    } \
    \
    Type PropertyList::get##TypeName(const std::string &name) const { \
        auto it = m_properties.find(name); \
        if (it == m_properties.end()) \
            throw NoriException("Property '%s' is missing!", name); \
        if (it->second.type != Property::XmlName##_type) \
            throw NoriException("Property '%s' has the wrong type! " \
                "(expected <" #XmlName ">)!", name); \
        return it->second.value.XmlName##_value; \
    } \
    \
    Type PropertyList::get##TypeName(const std::string &name, const Type &defVal) const { \
        auto it = m_properties.find(name); \
        if (it == m_properties.end()) \
            return defVal; \
        if (it->second.type != Property::XmlName##_type) \
            throw NoriException("Property '%s' has the wrong type! " \
                "(expected <" #XmlName ">)!", name); \
        return it->second.value.XmlName##_value; \
    } \
    \
    bool PropertyList::has##TypeName(const std::string &name, Type &val) const { \
        auto it = m_properties.find(name); \
        if (it == m_properties.end()) \
            return false; \
        if (it->second.type != Property::XmlName##_type) \
            throw NoriException("Property '%s' has the wrong type! " \
                "(expected <" #XmlName ">)!", name); \
        val = it->second.value.XmlName##_value; \
        return true;  \
    } \

// var1: the defined type
// var2: the type used for typing (function name)
// var3: the type written in XML file

DEFINE_PROPERTY_ACCESSOR(bool, Boolean, boolean)
DEFINE_PROPERTY_ACCESSOR(int, Integer, integer)
DEFINE_PROPERTY_ACCESSOR(float, Float, float)
DEFINE_PROPERTY_ACCESSOR(Color3f, Color, color)
DEFINE_PROPERTY_ACCESSOR(Point3f, Point, point)
DEFINE_PROPERTY_ACCESSOR(Vector3f, Vector, vector)
DEFINE_PROPERTY_ACCESSOR(std::string, String, string)
DEFINE_PROPERTY_ACCESSOR(Transform, Transform, transform)

NORI_NAMESPACE_END

