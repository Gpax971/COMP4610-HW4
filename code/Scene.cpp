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

    Vector3f hitPoint = inter.coords;
    Vector3f N = inter.normal; // normal
    Vector3f dir = ray.direction;
    
    if (mat->m_type == GLASS && TASK_N>=3) {
        if (depth < MAX_DEPTH) {
            float Kr = fresnel(dir.normalized(), N.normalized(), mat->ior);
            Vector3f reflectionDir = reflect(dir.normalized(), N.normalized());
            Vector3f refractionDir = refract(dir.normalized(), N.normalized(), mat->ior);
            Vector3f reflectionColour = castRay(Ray(hitPoint + reflectionDir.normalized() * EPSILON, reflectionDir), depth + 1);
            Vector3f refractionColour = castRay(Ray(hitPoint + refractionDir.normalized() * EPSILON, refractionDir), depth + 1);
            if (refractionColour.x == -1) return reflectionColour;
            if (reflectionColour.x == -1) return refractionColour;
            return Kr * reflectionColour + (1.f - Kr) * refractionColour;
        }
        return -1;
    }

    float p = mat->specularExponent;
    Vector3f v = -dir.normalized();
    Vector3f color = inter.obj->evalDiffuseColor(inter.tcoords);

    for (const std::unique_ptr<PointLight>& light : lights) {
        Vector3f lightDiff = light->position - hitPoint;
        Intersection lightInter = intersect(Ray(light->position, -lightDiff));
        if (lightInter.obj != inter.obj || (lightInter.coords - inter.coords).norm() > EPSILON) continue;

        Vector3f I = light->intensity;
        Vector3f l = lightDiff.normalized();
        Vector3f h = (l + v).normalized();
    
        Vector3f specular = (mat->Ks * I) * pow(std::max(0.f, dotProduct(N, h)), p);
        hitColor += specular;

        Vector3f diffuse = (mat->Kd * I) * std::max(0.f, dotProduct(N, l));
        hitColor += diffuse;
    }

    if (TASK_N >= 5) {
        int light_sample=4;
        for (int i = 0; i < light_sample; ++i) {
            Intersection lightInter;
            float pdf_light = 0.0f;
            sampleLight(lightInter, pdf_light);  // sample a point on the area light
            if (!lightInter.happened) break;

            Vector3f lightPoint = lightInter.coords;
            Vector3f ws = lightPoint - hitPoint;
            if (intersect(Ray(lightPoint, -ws)).obj != inter.obj) continue;
            float ws_norm = ws.norm();
            Vector3f lightColor = lightInter.material->m_emission * mat->eval(ws.normalized(), N.normalized()) 
                                * dotProduct(ws.normalized(), N.normalized()) / (ws_norm * ws_norm) / pdf_light;
            hitColor += lightColor / light_sample;
        }
    }

    return hitColor * color;
}
