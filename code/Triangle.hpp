#pragma once

#include "BVH.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "OBJ_Loader.hpp"
#include "Object.hpp"
#include "Triangle.hpp"
#include <cassert>
#include <array>
#include <cstring>
#include <map>


class Triangle : public Object
{
public:
    Vector3f v0, v1, v2; // vertices A, B, C counter-clockwise order
    Vector3f e1, e2;     // 2 edges v1-v0, v2-v0
    Vector2f t0, t1, t2; // texture coords
    Vector3f n0, n1, n2; // vertex normals
    bool hasVertexNormals = false;
    Vector3f normal;
    float area;
    Material* m;

    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material* _m = nullptr)
        : v0(_v0), v1(_v1), v2(_v2), m(_m)
    {
        e1 = v1 - v0;
        e2 = v2 - v0;
        normal = normalize(crossProduct(e1, e2));
        area = crossProduct(e1, e2).norm() * 0.5f;
        n0 = 0;
        n1 = 0;
        n2 = 0;
    }

    Intersection getIntersection(Ray ray) override;
    void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
                              const uint32_t& index, const Vector2f& uv,
                              Vector3f& N, Vector2f& st) const override
    {
        N = normal;
        //        throw std::runtime_error("triangle::getSurfaceProperties not
        //        implemented.");
    }
    Vector3f evalDiffuseColor(const Vector2f&) const override;
    Bounds3 getBounds() override;
    void Sample(Intersection &pos, float &pdf){
        float x = std::sqrt(get_random_float()), y = get_random_float();
        pos.coords = v0 * (1.0f - x) + v1 * (x * (1.0f - y)) + v2 * (x * y);
        pos.normal = this->normal;
        pdf = 1.0f / area;
    }
    float getArea(){
        return area;
    }
    bool hasEmit(){
        return m->hasEmission();
    }
private:
    inline static std::map<std::tuple<float, float, float>, Vector3f> vertex_normals;
};

class MeshTriangle : public Object
{
public:
    MeshTriangle(const Vector3f* verts, const uint32_t* vertsIndex, const uint32_t& numTris, const Vector2f* st, Material *mt = new Material())
    {
        uint32_t maxIndex = 0;
        for (uint32_t i = 0; i < numTris * 3; ++i)
            if (vertsIndex[i] > maxIndex)
                maxIndex = vertsIndex[i];
        stCoordinates = std::unique_ptr<Vector2f[]>(new Vector2f[maxIndex]);
        memcpy(stCoordinates.get(), st, sizeof(Vector2f) * maxIndex);
        m=mt;

        Vector3f min_vert = Vector3f{std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity()};
        Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()};
        for (int i = 0; i < numTris; i++) {
            std::array<Vector3f, 3> face_vertices;

            for (int j = 0; j < 3; j++) {
                auto vert = Vector3f(verts[vertsIndex[i*3+j]].x,
                                     verts[vertsIndex[i*3+j]].y,
                                     verts[vertsIndex[i*3+j]].z);
                face_vertices[j] = vert;

                min_vert = Vector3f(std::min(min_vert.x, vert.x),
                                    std::min(min_vert.y, vert.y),
                                    std::min(min_vert.z, vert.z));
                max_vert = Vector3f(std::max(max_vert.x, vert.x),
                                    std::max(max_vert.y, vert.y),
                                    std::max(max_vert.z, vert.z));
            }
            Triangle* tri = new Triangle(face_vertices[0], face_vertices[1],
                                        face_vertices[2], mt);
            tri->t0=st[vertsIndex[i*3]];
            tri->t1=st[vertsIndex[i*3+1]];
            tri->t2=st[vertsIndex[i*3+2]];

            triangles.push_back(*tri);


//            triangles.emplace_back(face_vertices[0], face_vertices[1],
//                                   face_vertices[2], mt);
        }

        bounding_box = Bounds3(min_vert, max_vert);

        std::vector<Object*> ptrs;
        for (auto& tri : triangles){
            ptrs.push_back(&tri);
            area += tri.area;
        }
        bvh = new BVHAccel(ptrs);
    }

    MeshTriangle(const std::string& filename, Vector3f offset, Material *mt = new Material())
    {
        objl::Loader loader;
        loader.LoadFile(filename);
        area = 0;
        m = mt;
        assert(loader.LoadedMeshes.size() == 1);
        objl::Mesh mesh = loader.LoadedMeshes[0];
        std::map<std::tuple<float, float, float>, Vector3f> vertex_counts;

        Vector3f min_vert = Vector3f{std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity()};
        Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()};

        if (mt->m_type == GLASS && TASK_N == 6) {
            for (int i = 0; i < mesh.Vertices.size(); i += 3) {
                float x1 = mesh.Vertices[i].Position.X;
                float y1 = mesh.Vertices[i].Position.Y;
                float z1 = mesh.Vertices[i].Position.Z;
                float x2 = mesh.Vertices[i + 1].Position.X;
                float y2 = mesh.Vertices[i + 1].Position.Y;
                float z2 = mesh.Vertices[i + 1].Position.Z;
                float x3 = mesh.Vertices[i + 2].Position.X;
                float y3 = mesh.Vertices[i + 2].Position.Y;
                float z3 = mesh.Vertices[i + 2].Position.Z;
                if (!vertex_counts.count({x1, y1, z1})) vertex_counts.insert({{x1, y1, z1}, 0});
                if (!vertex_counts.count({x2, y2, z2})) vertex_counts.insert({{x2, y2, z2}, 0});
                if (!vertex_counts.count({x3, y3, z3})) vertex_counts.insert({{x3, y3, z3}, 0});

                Vector3f e1 = Vector3f(x2, y2, z2) - Vector3f(x1, y1, z1);
                Vector3f e2 = Vector3f(x3, y3, z3) - Vector3f(x1, y1, z1);
                vertex_counts.at({x1, y1, z1}) += crossProduct(e1, e2);
                vertex_counts.at({x2, y2, z2}) += crossProduct(e1, e2);
                vertex_counts.at({x3, y3, z3}) += crossProduct(e1, e2);
            }
        }

        for (int i = 0; i < mesh.Vertices.size(); i += 3) {
            std::array<Vector3f, 3> face_vertices;
            std::array<Vector3f, 3> vertex_normals;

            for (int j = 0; j < 3; j++) {
                auto vert = Vector3f(mesh.Vertices[i + j].Position.X+offset.x,
                                     mesh.Vertices[i + j].Position.Y+offset.y,
                                     mesh.Vertices[i + j].Position.Z+offset.z);
                
                face_vertices[j] = vert;
                vertex_normals[j] = { mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z };

                min_vert = Vector3f(std::min(min_vert.x, vert.x),
                                    std::min(min_vert.y, vert.y),
                                    std::min(min_vert.z, vert.z));
                max_vert = Vector3f(std::max(max_vert.x, vert.x),
                                    std::max(max_vert.y, vert.y),
                                    std::max(max_vert.z, vert.z));
            }

            Triangle tri(face_vertices[0], face_vertices[1],
                                   face_vertices[2], mt);


            if (mt->m_type == GLASS && TASK_N >= 6) {
                tri.n0 = vertex_counts.at({tri.v0.x - offset.x, tri.v0.y - offset.y, tri.v0.z - offset.z}).normalized();
                tri.n1 = vertex_counts.at({tri.v1.x - offset.x, tri.v1.y - offset.y, tri.v1.z - offset.z}).normalized();
                tri.n2 = vertex_counts.at({tri.v2.x - offset.x, tri.v2.y - offset.y, tri.v2.z - offset.z}).normalized();
                tri.hasVertexNormals = true;
            }
            triangles.emplace_back(tri);
        }

        bounding_box = Bounds3(min_vert, max_vert);

        std::vector<Object*> ptrs;
        for (auto& tri : triangles){
            ptrs.push_back(&tri);
            area += tri.area;
        }
        bvh = new BVHAccel(ptrs);
    }


    Bounds3 getBounds() { return bounding_box; }

    void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
                              const uint32_t& index, const Vector2f& uv,
                              Vector3f& N, Vector2f& st) const
    {
        const Vector3f& v0 = vertices[vertexIndex[index * 3]];
        const Vector3f& v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f& v2 = vertices[vertexIndex[index * 3 + 2]];
        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);
        N = normalize(crossProduct(e0, e1));
        const Vector2f& st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f& st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f& st2 = stCoordinates[vertexIndex[index * 3 + 2]];
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
    }

    Vector3f evalDiffuseColor(const Vector2f& st) const
    {
        float scale = 5;
        float pattern =
            (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031),
                    Vector3f(0.937, 0.937, 0.231), pattern);
    }

    Intersection getIntersection(Ray ray)
    {
        Intersection intersec;

        if (bvh) {
            intersec = bvh->Intersect(ray);
        }

        return intersec;
    }
    
    void Sample(Intersection &pos, float &pdf){
        bvh->Sample(pos, pdf);
        pos.obj=this;
        pos.material=this->m;
    }
    float getArea(){
        return area;
    }
    bool hasEmit(){
        return m->hasEmission();
    }

    Bounds3 bounding_box;
    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;

    std::vector<Triangle> triangles;

    BVHAccel* bvh;
    float area;

    Material* m;
};


inline Bounds3 Triangle::getBounds() { return Union(Bounds3(v0, v1), v2); }

inline Intersection Triangle::getIntersection(Ray ray)
{
    const float epsilon = std::numeric_limits<float>::epsilon();
    Intersection inter;
    inter.happened = false;

    Vector3f ray_cross_e2 = crossProduct(ray.direction, this->e2);
    float det = dotProduct(this->e1, ray_cross_e2);

    if (det > -epsilon && det < epsilon) return inter;

    float inv_det = 1.0 / det;
    Vector3f s = ray.origin - this->v0;
    float u = inv_det * dotProduct(s, ray_cross_e2);

    if (u < epsilon || u > (1 - epsilon)) return inter;

    Vector3f s_cross_e1 = crossProduct(s, this->e1);
    float v = inv_det * dotProduct(ray.direction, s_cross_e1);
    if (v < epsilon || u + v > 1 - epsilon) return inter;

    float t = inv_det * dotProduct(this->e2, s_cross_e1);
    if (t > epsilon) {
        inter.coords = ray.origin + ray.direction * t;
        inter.happened = true;
        inter.normal = this->hasVertexNormals ? (u * n0.normalized() + v * n1.normalized() + (1.f - u - v) * n2.normalized()).normalized() : this->normal;
        inter.material = this->m;
        inter.obj = this;
        inter.tcoords = {u, v};
        inter.tnear = (ray.origin - inter.coords).norm();
    }
    return inter;
}

inline Vector3f Triangle::evalDiffuseColor(const Vector2f& st) const
{
    if (!m->textured) return m->getColor();
    auto uv = t0 * (1 - st.x - st.y) + t1 * st.x + t2 * st.y;
    float scale = 5;
    float pattern = (fmodf(uv.x * scale, 1) > 0.5) ^ (fmodf(uv.y * scale, 1) > 0.5);
    return lerp(Vector3f(0.815, 0.235, 0.031), Vector3f(0.937, 0.937, 0.231), pattern);
}
