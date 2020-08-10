#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Cube.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{
	/*Vector3f N(0, 0, 1);
	Vector3f I = normalize(Vector3f(0, 1, 1));
	float ior = 1.2;
	float cosi = clamp(-1, 1, dotProduct(I, N));
	float etai = 1, etat = ior;
	Vector3f n = N;
	if (cosi < 0) { cosi = -cosi; }
	else { std::swap(etai, etat); n = -N; }
	float eta = etai / etat;
	float k = 1 - eta * eta * (1 - cosi * cosi);
	auto p = k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
	std::cout << "this is p : " << p << std::endl;*/

    // Change the definition here to change resolution
    Scene scene(SceneRate, SceneRate);

    Material* red = new Material(DIFFUSE_COS, Vector3f(0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material* green = new Material(DIFFUSE_COS, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material* white = new Material(DIFFUSE_COS, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material* light = new Material(DIFFUSE_COS, (8.0f * Vector3f(0.747f+0.058f, 0.747f+0.258f, 0.747f) + 15.6f * Vector3f(0.740f+0.287f,0.740f+0.160f,0.740f) + 18.4f *Vector3f(0.737f+0.642f,0.737f+0.159f,0.737f)));
    light->Kd = Vector3f(0.65f);

	Material* white1 = new Material(BSDF, Vector3f(0.0f));
	white1->Kd = Vector3f(0.725f, 0.71f, 0.68f);

    MeshTriangle floor("../../models/cornellbox/floor.obj", green);
    MeshTriangle shortbox("../../models/cornellbox/shortbox.obj", white);
    MeshTriangle tallbox("../../models/cornellbox/tallbox.obj", white1);
    MeshTriangle left("../../models/cornellbox/left.obj", red);
    MeshTriangle right("../../models/cornellbox/right.obj", green);
    MeshTriangle light_("../../models/cornellbox/light.obj", light);
	MeshTriangle Sakura("../../models/sf2-sakura/sakura.obj", white);
	Vector3f center(200, 150, 350);
	Sphere boll(center, 100, white1);
	Cube cube1(Vector3f(150, 100, 200), Vector3f(300, 250, 220), white1);


    scene.Add(&floor);
   // scene.Add(&shortbox);
    scene.Add(&tallbox);
    scene.Add(&left);
	scene.Add(&right);
    scene.Add(&light_);
	//scene.Add(&Sakura);
	//scene.Add(&cube1);

    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}