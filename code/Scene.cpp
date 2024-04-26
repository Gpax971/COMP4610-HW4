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
    Vector3f hitColor = backgroundColor;
    auto inter = intersect(ray);
    if (!inter.happened)return backgroundColor;

    Vector3f hitPoint = inter.coords;
    Vector3f N = inter.normal; // normal
    Vector2f st = inter.tcoords; // texture coordinates
    Vector3f dir = ray.direction;

    if (inter.material->m_type == EMIT) {
        return inter.material->m_emission;
    }else if (inter.material->m_type == DIFFUSE || TASK_N<3) {
        Vector3f diffuseColor = 0, specularColor = 0;

        // sample area light
        int light_sample=4;
        for (int i = 0; i < light_sample && TASK_N >= 5; ++i) {
            Intersection lightInter;
            float pdf_light = 0.0f;
            sampleLight(lightInter, pdf_light);  // sample a point on the area light
            // TODO: task 5 soft shadow


        }
        // TODO: task 1.3 Basic shading


    } else if (inter.material->m_type == GLASS && TASK_N>=3) {
        // TODO: task 3 glass material


    }


    return hitColor;
}
