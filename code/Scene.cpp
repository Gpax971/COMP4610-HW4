//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}


void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            pos.happened=true;  // area light that has emission exists
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}


// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Vector3f hitColor = 0;
    Intersection inter = intersect(ray);
    if (!inter.happened) return backgroundColor;


    Material* mat = inter.material;

    if (mat->m_type == EMIT) {
        return mat->m_emission;
    }

    Vector3f pointColor = inter.obj->evalDiffuseColor(inter.tcoords);
    Vector3f hitPoint = inter.coords;
    Vector3f N = inter.normal; // normal
    Vector3f dir = ray.direction;
    
    if (mat->m_type == GLASS && TASK_N >= 3) {
        if (depth < MAX_DEPTH) {
            float Kr = fresnel(dir.normalized(), N.normalized(), mat->ior);
            Vector3f reflectionDir = reflect(dir.normalized(), N.normalized());
            Vector3f refractionDir = refract(dir.normalized(), N.normalized(), mat->ior);
            Vector3f reflectionColour = castRay(Ray(hitPoint + reflectionDir.normalized() * EPSILON, reflectionDir), depth + 1);
            Vector3f refractionColour = castRay(Ray(hitPoint + refractionDir.normalized() * EPSILON, refractionDir), depth + 1);
            return Kr * reflectionColour + (1.f - Kr) * refractionColour;
        }
        return 0;
    }

    float p = mat->specularExponent;
    Vector3f v = -dir.normalized();

    for (const std::unique_ptr<PointLight>& light : lights) {
        Vector3f lightDiff = light->position - hitPoint;
        Vector3f l = lightDiff.normalized();
        Intersection lightInter = intersect(Ray(hitPoint, lightDiff));
        if (lightInter.happened || lightInter.tnear < lightDiff.norm() + EPSILON) continue;

        Vector3f I = light->intensity;
        Vector3f R = reflect(-l, N);
    
        Vector3f specular = (mat->Ks * I) * pow(std::max(0.f, dotProduct(v, R)), p);
        hitColor += specular;

        Vector3f diffuse = (mat->Kd * I) * std::max(0.f, dotProduct(N, l));
        hitColor += diffuse;
    }

    if (TASK_N >= 5) {
        int light_sample = 8;
        for (int i = 0; i < light_sample; ++i) {
            Intersection lightInter;
            float pdf_light = 0.0f;
            sampleLight(lightInter, pdf_light);
            if (!lightInter.happened) break;

            Vector3f lightPoint = lightInter.coords;
            Vector3f ws = lightPoint - hitPoint;
            Vector3f ws_n = ws.normalized();
            Intersection shadowInter = intersect(Ray(lightPoint, -ws));
            if (shadowInter.obj != inter.obj || (shadowInter.coords - inter.coords).norm() > EPSILON) continue;

            float ws_norm = ws.norm();
            Vector3f lightColor = lightInter.material->m_emission * mat->eval(ws_n, N) 
                                * dotProduct(ws_n, N) / (ws_norm * ws_norm) / pdf_light;
            hitColor += lightColor / light_sample;
        }
    }

    return hitColor * pointColor;
}
