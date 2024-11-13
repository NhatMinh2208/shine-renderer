/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/accel.h>
#include <nori/memory.h>
#include <Eigen/Geometry>
#include <numeric>
#include <array>
#include <algorithm>
NORI_NAMESPACE_BEGIN

void Accel::addMesh(Mesh *mesh) {
    if (m_mesh)
        throw NoriException("Accel: only a single mesh is supported!");
    m_mesh = mesh;
    m_bbox = m_mesh->getBoundingBox();
}

void Accel::build() {
    if (!m_mesh)
        return;
    nextFreeNode = nAllocedNodes = 0;
    const uint32_t mesh_size = m_mesh->getTriangleCount();
    if (mesh_size < 10){
        // iterate all the triangle index of mesh and add to the node
        nodes->triangleIndicesOffset = 0;
        nodes->nTris = m_mesh->getTriangleCount();

        for(int id = 0; id < m_mesh->getTriangleCount(); id ++){
            // root->triangles.push_back(id);
            triangleIndices.push_back(id);
        }
    }
    else{
        std::vector<uint32_t> triangleId(mesh_size);
        std::iota(triangleId.begin(), triangleId.end(), 0);
        // root = build(1, m_bbox, triangleId);
        build(0, 1, m_bbox, triangleId);
    }    
    std::cout << "mesh bbox: " << m_bbox.toString();
}

void Accel::build(int nodeNum, int depth ,BoundingBox3f bbox, std::vector<uint32_t> triangles)  {
    Assert(nodeNum == nextFreeNode);
    /* Nothing to do here for now */
    // if (triangles.empty())
    //     return nullptr;
    // Alloc memory
    if (nextFreeNode == nAllocedNodes) {
        int nNewAllocNodes = std::max(2 * nAllocedNodes, 512);
        OctreeNode *n = AllocAligned<OctreeNode>(nNewAllocNodes);
        if (nAllocedNodes > 0) {
            memcpy(n, nodes, nAllocedNodes * sizeof(OctreeNode));
            FreeAligned(nodes);
        }
        nodes = n;
        nAllocedNodes = nNewAllocNodes;
    }
    ++nextFreeNode;

    if (triangles.empty()){
        nodes[nodeNum].nTris = -1;
        return;
    }

    if (triangles.size() < 10 || depth >= maxdepth){
        int IndicesOffset = triangleIndices.size();
        for (auto id : triangles) triangleIndices.push_back(id);
        // return new OctreeNode(triangles, bbox);
        nodes[nodeNum].InitLeaf(IndicesOffset, triangles.size(), bbox);
        // cout << "hello7 ";
        Assert((nodes + nodeNum) != nullptr);
        return;
    }
    BoundingBox3f bboxs[8];
    Point3f center = bbox.getCenter();
    std::vector<uint32_t> triangleId;
    const MatrixXu& F = m_mesh->getIndices();
    const MatrixXf& V = m_mesh->getVertexPositions();
    nodes[nodeNum].UpdateBbox(bbox);
    // OctreeNode* node = new OctreeNode();
    // node->bbox = bbox;
    //split the bounding box into 8 equal part
    for (int i = 0; i < 8; i ++){
        Point3f corner = bbox.getCorner(i);
        // std::array<float, 3> min{}, max{};
        // for(int j = 0; j < 3; j++){
        //     min[j] = (i & (1 << j)) ? midP(j) : minP(j);
        //     max[j] = (i & (1 << j)) ? maxP(j) : midP(j);
        // }

        //make sure the bbox is valid
        Point3f min, max;
        min[0] = corner[0] <= center[0] ? corner[0] : center[0];
        min[1] = corner[1] <= center[1] ? corner[1] : center[1];
        min[2] = corner[2] <= center[2] ? corner[2] : center[2];
        max[0] = corner[0] >= center[0] ? corner[0] : center[0];
        max[1] = corner[1] >= center[1] ? corner[1] : center[1];
        max[2] = corner[2] >= center[2] ? corner[2] : center[2];
        // bool check = (corner[0] <= center[0]) && (corner[1] <= center[1]) &&  (corner[2] <= center[2]);
        // bboxs[i] = check ? BoundingBox3f(corner, center) : BoundingBox3f(center, corner);
        // std::cout << min << " ";
        // std::cout << max << "  ";
        bboxs[i] = BoundingBox3f(min, max);
        // bboxs[i] = BoundingBox3f(Point3f(min[0], min[1], min[2]), 
        //                    Point3f(max[0], max[1], max[2]));
        //std::cout << "mesh bbox: " << bboxs[i].toString() << "\n";
        for (auto id : triangles) {
            uint32_t idx0 = F(0, id), idx1 = F(1, id), idx2 = F(2, id);
            Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);
            BoundingBox3f tri_bbox(p0); //get the triangle bounding box
            tri_bbox.expandBy(p1);
            tri_bbox.expandBy(p2);
            if (bboxs[i].overlaps(tri_bbox)) triangleId.push_back(id);
        }
        // node->children[i] = build(depth + 1, bboxs[i], triangleId);
        Assert((nodes + nodeNum) != nullptr);
        nodes[nodeNum].UpdateInterior(i, nextFreeNode);
        build(nextFreeNode, depth + 1, bboxs[i], triangleId);
        // cout << "hello9 ";
    }
    Assert((nodes + nodeNum) != nullptr);
    Assert((int)(nodes + nodeNum) % 8 == 0);
    return;
}

bool Accel::rayIntersect1(Ray3f &ray, Intersection &its, bool shadowRay, int nextNode, bool _foundIntersection, uint32_t &face_idx) const {
    bool foundIntersection = _foundIntersection;  // Was an intersection found so far?
    // uint32_t f = (uint32_t) -1;      // Triangle index of the closest intersection

    // Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

    /* Iterate the octree */
    OctreeNode* node = nodes + nextNode;
    if(node->nTris > 0){
        //std::cout << "not empty ";
        // for (const auto& idx : (node->triangles)) {
        for (int i = node->triangleIndicesOffset; i <  node->triangleIndicesOffset + node->nTris; i++){
            const auto idx = triangleIndices[i];
            float u, v, t;
            if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
            /* An intersection was found! Can terminate
               immediately if this is a shadow ray query */
                if (shadowRay)
                    return true;
                ray.maxt = its.t = t;
                its.uv = Point2f(u, v);
                its.mesh = m_mesh;
                face_idx = idx;
                foundIntersection = true;
            }
        }
    }
    else{
        // std::cout << "so empty ";
        // cout << node->children << " ";
        // std::sort(node->children, node->children + 7, [](const OctreeNode*a, const OctreeNode* b){
        //     // float tMin1, tMin2, tMax1, tMax2;
        //     // if (!a->bbox.rayIntersect(ray, tMin1, tMax1)) tMin1 = INFINITY;
        //     // if (!b->bbox.rayIntersect(ray, tMin2, tMax2)) tMin2 = INFINITY;
        //     // return tMin1 < tMin2;
        //     return true;
        // });
        //std::cout << "so empty ";
        // for (int i = 0; i < 8; i ++){
        //     for(int j = 0; j < 7 - i ; j++){
        //         float tMin1, tMin2, tMax1, tMax2;
        //         if (!node->children[j]->bbox.rayIntersect(ray, tMin1, tMax1)) tMin1 = INFINITY;
        //         if (!node->children[j+1]->bbox.rayIntersect(ray, tMin2, tMax2)) tMin2 = INFINITY;
        //         if (tMin1 <= tMin2) {  
        //             OctreeNode* temp = node->children[j];  
        //             node->children[j] = node->children[j + 1];  
        //             node->children[j + 1] = temp;  
        //         }  
        //     }
        // }
        // std::array<OctreeNode*, 8> arr;
        // std::copy_n(std::begin(node->children), 8, std::begin(arr));
        // std::sort(arr.begin(), arr.end(), [](OctreeNode* a, OctreeNode* b){
        // //     // float tMin1, tMin2, tMax1, tMax2;
        // //     // if (!a->bbox.rayIntersect(ray, tMin1, tMax1)) tMin1 = INFINITY;
        // //     // if (!b->bbox.rayIntersect(ray, tMin2, tMax2)) tMin2 = INFINITY;
        // //     // return tMin1 < tMin2;
        // return true;
        // });
        std::pair<float, size_t> distances[8] = {};
        // auto child = node->children;
        for (int i = 0; i < 8; i++) {
            float tMin, tMax;
            int childNodeIndex = node->children[i];
            OctreeNode* child = nodes + childNodeIndex;
            if(child->nTris == -1) continue; //This node is null
            // if(!child) continue;
            //BoundingBox3f box = child->bbox;
            if(!child->bbox.rayIntersect(ray, tMin, tMax)) tMin = INFINITY;
            //tMin = node->children[i]->bbox.distanceTo(ray.o);
            distances[i] = std::move(std::make_pair(tMin, i));
        }

        std::sort(distances, distances + 8, [](const auto& l, const auto& r) { return l.first < r.first; });

        for (auto pair : distances) {
            int childNodeIndex = node->children[pair.second];
            OctreeNode* child = nodes + childNodeIndex;
            // if(!child) continue;
            if(child->nTris == -1) continue; //This node is null
            float distance = pair.first;
            //if (distance == INFINITY) continue;
            bool check = child->bbox.rayIntersect(ray);
            if(!check) continue;
            if (distance > ray.maxt) break;
            foundIntersection = foundIntersection || rayIntersect1(ray, its, shadowRay, childNodeIndex, foundIntersection, face_idx);
            if (shadowRay && foundIntersection) return true;
        }
        //  for (int i = 0; i < 8; i++){
        //     OctreeNode* child = node->children[i];
        //     if(!child) continue;
        //     bool check = child->bbox.rayIntersect(ray);
        //     if(!check) continue;
        //     BoundingBox3f bbox1 = child->bbox;
        // }

        // for (int i = 0; i < 8; i++){
        //     OctreeNode* child = node->children[i];
        //     if(!child) continue;
        //     bool check = child->bbox.rayIntersect(ray);
        //     if(!check) continue;
        //     foundIntersection = foundIntersection || rayIntersect1(ray, its, shadowRay, child, foundIntersection, face_idx);
        //     if(foundIntersection) break;
        // }
    }    
    return foundIntersection;
}

bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const {
    bool foundIntersection = false;  // Was an intersection found so far?
    uint32_t f = (uint32_t) -1;      // Triangle index of the closest intersection
    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)
    /* Iterate the octree */
    if (!nodes) return false;  // Checking the root
    //if (nodes->nTris == -1) return false;
    if (!nodes->bbox.rayIntersect(ray)) return false;
    foundIntersection = rayIntersect1(ray, its, shadowRay, 0, foundIntersection, f);
    if (shadowRay && foundIntersection) return true;
    if (foundIntersection) {
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.

           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */
        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1-its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh *mesh   = its.mesh;
        const MatrixXf &V  = mesh->getVertexPositions();
        const MatrixXf &N  = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F  = mesh->getIndices();
        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);
        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                bary.y() * UV.col(idx1) +
                bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                 bary.y() * N.col(idx1) +
                 bary.z() * N.col(idx2)).normalized());
        } else {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}

// bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const {
//     bool foundIntersection = false;  // Was an intersection found so far?
//     uint32_t f = (uint32_t) -1;      // Triangle index of the closest intersection

//     Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

//     /* Brute force search through all triangles */
//     for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
//         float u, v, t;
//         if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
//             /* An intersection was found! Can terminate
//                immediately if this is a shadow ray query */
//             if (shadowRay)
//                 return true;
//             ray.maxt = its.t = t;
//             its.uv = Point2f(u, v);
//             its.mesh = m_mesh;
//             f = idx;
//             foundIntersection = true;
//         }
//     }

//     if (foundIntersection) {
//         /* At this point, we now know that there is an intersection,
//            and we know the triangle index of the closest such intersection.

//            The following computes a number of additional properties which
//            characterize the intersection (normals, texture coordinates, etc..)
//         */

//         /* Find the barycentric coordinates */
//         Vector3f bary;
//         bary << 1-its.uv.sum(), its.uv;

//         /* References to all relevant mesh buffers */
//         const Mesh *mesh   = its.mesh;
//         const MatrixXf &V  = mesh->getVertexPositions();
//         const MatrixXf &N  = mesh->getVertexNormals();
//         const MatrixXf &UV = mesh->getVertexTexCoords();
//         const MatrixXu &F  = mesh->getIndices();

//         /* Vertex indices of the triangle */
//         uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

//         Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

//         /* Compute the intersection positon accurately
//            using barycentric coordinates */
//         its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

//         /* Compute proper texture coordinates if provided by the mesh */
//         if (UV.size() > 0)
//             its.uv = bary.x() * UV.col(idx0) +
//                 bary.y() * UV.col(idx1) +
//                 bary.z() * UV.col(idx2);

//         /* Compute the geometry frame */
//         its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

//         if (N.size() > 0) {
//             /* Compute the shading frame. Note that for simplicity,
//                the current implementation doesn't attempt to provide
//                tangents that are continuous across the surface. That
//                means that this code will need to be modified to be able
//                use anisotropic BRDFs, which need tangent continuity */

//             its.shFrame = Frame(
//                 (bary.x() * N.col(idx0) +
//                  bary.y() * N.col(idx1) +
//                  bary.z() * N.col(idx2)).normalized());
//         } else {
//             its.shFrame = its.geoFrame;
//         }
//     }

//     return foundIntersection;
// }

NORI_NAMESPACE_END

