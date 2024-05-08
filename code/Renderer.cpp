//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <sstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include "Material.hpp"
#ifdef _OPENMP
    #include <omp.h>
#endif


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.01;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.

void Renderer::Render(const Scene& scene)
{
    Vector3f framebuffer[scene.width * scene.height];
    // std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    std::cout << "SPP: " << scene.spp << "\n";

    float progress = 0.f;


#pragma omp parallel num_threads(8)
{
#pragma omp for nowait schedule(dynamic, 128) collapse(2) // use multi-threading for speedup if openmp is available
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            Vector3f out = 0;
            
            float x_offset = (scene.spp == 1) ? 0.5 : get_random_float();
            float y_offset = (scene.spp == 1) ? 0.5 : get_random_float();
            for (int k = 0; k < scene.spp; ++k) {
                
                float x = scale * (1 - ((float)(i << 1) + 2 * x_offset) / scene.width);
                float y = scale * (1 - ((float)(j << 1) + 2 * y_offset) / scene.height);
                Vector3f ray_dir {x, y, 1};

                Vector3f dir = normalize(ray_dir);
                out += scene.castRay(Ray(eye_pos, dir), 0);
                x_offset = get_random_float();
                y_offset = get_random_float();
            }
            
            framebuffer[i + j * scene.width] = out / scene.spp;
            if (i < scene.width - 1) continue;
            UpdateProgress(progress);
            progress += 1.0f / (float)scene.height;
        }
    }
}
    UpdateProgress(1.f);

    // save framebuffer to file
    std::stringstream ss;
    ss << "../final_output_image/binary_task" << TASK_N ;
    if (scene.width != 256 || scene.height != 256) ss << "_" << scene.width << "x" << scene.height;
    ss << ".ppm";
    std::string str = ss.str();
    const char* file_name = str.c_str();
    FILE* fp = fopen(file_name, "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
