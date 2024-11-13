/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

struct OctreeNode {
    public:
        OctreeNode() {}
        OctreeNode(int triangleIndicesOffset, int nTris,  BoundingBox3f bbox) 
                : triangleIndicesOffset(triangleIndicesOffset), nTris(nTris), bbox(bbox) {}
        // OctreeNode(triangle_list triangles){
        //     for(int i = 0; i < 8; i++){
        //         triangles[i] = triangles[i];
        //     }
        // }
        void InitLeaf(int _triangleIndicesOffset, int _nTris,  BoundingBox3f _bbox){
            triangleIndicesOffset = _triangleIndicesOffset;
            nTris = _nTris;
            bbox = _bbox;
        }

        void UpdateInterior(int childIndex, int pointer2childNode){
            children[childIndex] = pointer2childNode;
        }

        void UpdateBbox(BoundingBox3f _bbox){
            bbox = _bbox;
        }
        //std::vector<uint32_t> triangles;
        BoundingBox3f bbox;
        int children[8]; // pointer to  a relative node position
        int triangleIndicesOffset, nTris; //offset from the triangleIndices and number of triangkes over lap
        // int64_t pad1, pad2, pad3, pad4;
};


/**
 * \brief Acceleration data structure for ray intersection queries
 *
 * The current implementation falls back to a brute force loop
 * through the geometry.
 */
class Accel {
public:
    /**
     * \brief Register a triangle mesh for inclusion in the acceleration
     * data structure
     *
     * This function can only be used before \ref build() is called
     */
    void addMesh(Mesh *mesh);

    /// Build the acceleration data structure (currently a no-op)
    void build();
    void build(int nodeNum, int depth, BoundingBox3f bbox, std::vector<uint32_t> triangles);
    /// Return an axis-aligned box that bounds the scene
    const BoundingBox3f &getBoundingBox() const { return m_bbox; }

    /**
     * \brief Intersect a ray against all triangles stored in the scene and
     * return detailed intersection information
     *
     * \param ray
     *    A 3-dimensional ray data structure with minimum/maximum extent
     *    information
     *
     * \param its
     *    A detailed intersection record, which will be filled by the
     *    intersection query
     *
     * \param shadowRay
     *    \c true if this is a shadow ray query, i.e. a query that only aims to
     *    find out whether the ray is blocked or not without returning detailed
     *    intersection information.
     *
     * \return \c true if an intersection was found
     */
    bool rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const;
    bool rayIntersect1(Ray3f &ray_, Intersection &its, bool shadowRay, int nextNode, bool _foundIntersection, uint32_t &face_idx) const;

private:
    Mesh         *m_mesh = nullptr; ///< Mesh (only a single one for now)
    BoundingBox3f m_bbox;           ///< Bounding box of the entire scene
    //OctreeNode *root;     ///< Root of the octree
    OctreeNode *nodes; 
    int maxdepth = 6;      ///< Max depth of octree construction
    int nAllocedNodes, nextFreeNode; ///< Number of allocated nodes and index of the next free node
    std::vector<int> triangleIndices; ///< Index of triangle overlap leaf node in order 
};


NORI_NAMESPACE_END
