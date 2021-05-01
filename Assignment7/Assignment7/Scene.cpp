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
    if (!objInter.happened)
    {
        return Vector3f(0.f);
    }
    Vector3f hitColor(0.f);
    if (objInter.m->hasEmission())
    {
        return objInter.m->getEmission();
    }

    // 采样光源点
    float light_pdf;
    Intersection lightInter;
    sampleLight(lightInter, light_pdf);
    Vector3f obj2light =  lightInter.coords-objInter.coords ;
    Vector3f obj2lightDir = obj2light.normalized();
    Ray toLightRay(objInter.coords, obj2lightDir);
    Vector3f Lo_dir(0.f), Lo_indir(0.f);
    Vector3f w_o = -normalize(ray.direction), w_i;
    Vector3f objN = normalize(objInter.normal);
    Vector3f lightN = normalize(lightInter.normal);
    if (intersect(toLightRay).distance - obj2light.norm()> -EPSILON)
    {
        //直接光照
        //入射方向为光源射向物体，出射方向（所求的方向)为参数ray的方向
        Vector3f f_r = objInter.m->eval(obj2lightDir, w_o, objN);
        //对光源采样
        float r2 = dotProduct(obj2light, obj2light);
        float cosA = std::max(.0f, dotProduct(objN, obj2lightDir));
        float cosB = std::max(.0f, dotProduct(lightN, -obj2lightDir));
        Lo_dir = lightInter.emit * f_r * cosA * cosB / r2 / light_pdf;
  
    }
    hitColor += Lo_dir;
    if (get_random_float() < RussianRoulette)
    {
        //间接光照
        w_i = objInter.m->sample(w_o, objN).normalized();
        float cos = std::max(.0f, dotProduct(w_i, objN));
        Vector3f f_r = objInter.m->eval(w_o, w_i,objN);
        float pdf = objInter.m->pdf(w_o, w_i, objN);
        Lo_indir = castRay(Ray(objInter.coords, w_i), depth) * f_r * cos / pdf / RussianRoulette;
    }
    hitColor += Lo_indir;
    return hitColor;
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
