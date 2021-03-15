//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
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
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        //计算发光物体的总面积
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        //找到光源
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                //按概率选取一条光线
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    //求一条光线与场景的交点
    Intersection objInter = intersect(ray);
    Vector3f hitcolor = Vector3f(0);
    if (objInter.emit.norm() > 0)
        hitcolor = Vector3f(1);
    else if (objInter.happened)
    {
        //根据pdf_light来采样光源点
        float pdf;
        Intersection lightInter;
        //选取的光线，从选取的光源射向物体
        sampleLight(lightInter, pdf);
        Vector3f x = lightInter.coords; //光源的坐标
        Vector3f p = objInter.coords;   //物体的坐标

        //实际的光线
        Ray sampled_ray = Ray(p, (x - p).normalized());

        Vector3f L_dir = Vector3f(0);
        Vector3f n = normalize(objInter.normal);
        Vector3f nn = normalize(lightInter.normal);

        Vector3f wo = normalize(-ray.direction);
        Vector3f ws = Vector3f(x - p);

        //探测是否有障碍
        Intersection t = intersect(sampled_ray);
        if (std::abs(t.distance - (x - p).norm()) < EPSILON)
        {
            L_dir = lightInter.emit * lightInter.m->eval(wo, ws, n) * dotProduct(ws, n) * dotProduct(-ws, nn) / ((x - p).norm() * (x - p).norm() * pdf);
        }

        Vector3f L_indir = Vector3f(0);

        float prob = get_random_float();
        if (prob < RussianRoulette)
        {
            Vector3f wi = objInter.m->sample(wo, n); //得到该物体的一个出射方向，作为下一个不发光物体对该物体的入射方向
            L_indir = castRay(Ray(p, wi), depth) * objInter.m->eval(wi, wo, n) * dotProduct(wi, n) / (objInter.m->pdf(wi, wo, n) * Scene::RussianRoulette);
        }
        hitcolor = L_dir + L_indir;
    }

    return hitcolor;
}
// // TO DO Implement Path Tracing Algorithm here
// Intersection intersection = intersect(ray);
// Vector3f hitcolor = Vector3f(0);

//     //deal with light source
//     if (intersection.emit.norm() > 0)
//         hitcolor = Vector3f(1);
//     else if (intersection.happened)
//     {
//         Vector3f wo = normalize(-ray.direction);
//         Vector3f p = intersection.coords;
//         Vector3f N = normalize(intersection.normal);

//         float pdf_light = 0.0f;
//         Intersection inter;
//         sampleLight(inter, pdf_light);
//         Vector3f x = inter.coords;
//         Vector3f ws = normalize(x - p);
//         Vector3f NN = normalize(inter.normal);

//         Vector3f L_dir = Vector3f(0);
//         //direct light
//         if ((intersect(Ray(p, ws)).coords - x).norm() < 0.01)
//         {
//             L_dir = inter.emit * intersection.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / (((x - p).norm() * (x - p).norm()) * pdf_light);
//         }

//         Vector3f L_indir = Vector3f(0);
//         float P_RR = get_random_float();
//         //indirect light
//         if (P_RR < Scene::RussianRoulette)
//         {
//             Vector3f wi = intersection.m->sample(wo, N);
//             L_indir = castRay(Ray(p, wi), depth) * intersection.m->eval(wi, wo, N) * dotProduct(wi, N) / (intersection.m->pdf(wi, wo, N) * Scene::RussianRoulette);
//         }
//         hitcolor = L_indir + L_dir;
//     }
//     return hitcolor;
// }
