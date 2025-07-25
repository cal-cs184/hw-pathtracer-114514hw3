#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

	PathTracer::PathTracer() {
		gridSampler = new UniformGridSampler2D();
		hemisphereSampler = new UniformHemisphereSampler3D();

		tm_gamma = 2.2f;
		tm_level = 1.0f;
		tm_key = 0.18;
		tm_wht = 5.0f;
	}

	PathTracer::~PathTracer() {
		delete gridSampler;
		delete hemisphereSampler;
	}

	void PathTracer::set_frame_size(size_t width, size_t height) {
		sampleBuffer.resize(width, height);
		sampleCountBuffer.resize(width * height);
	}

	void PathTracer::clear() {
		bvh = NULL;
		scene = NULL;
		camera = NULL;
		sampleBuffer.clear();
		sampleCountBuffer.clear();
		sampleBuffer.resize(0, 0);
		sampleCountBuffer.resize(0, 0);
	}

	void PathTracer::write_to_framebuffer(ImageBuffer& framebuffer, size_t x0,
		size_t y0, size_t x1, size_t y1) {
		sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
	}

	Vector3D
		PathTracer::estimate_direct_lighting_hemisphere(const Ray& r,
			const Intersection& isect) {
		// Estimate the lighting from this intersection coming directly from a light.
		// For this function, sample uniformly in a hemisphere.

		// Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
		// This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

		// make a coordinate system for a hit point
		// with N aligned with the Z direction.
		Matrix3x3 o2w;
		make_coord_space(o2w, isect.n);
		Matrix3x3 w2o = o2w.T();

		// w_out points towards the source of the ray (e.g.,
		// toward the camera if this is a primary ray)
		const Vector3D hit_p = r.o + r.d * isect.t;
		const Vector3D w_out = w2o * (-r.d);

		// This is the same number of total samples as
		// estimate_direct_lighting_importance (outside of delta lights). We keep the
		// same number of samples for clarity of comparison.
		int num_samples = scene->lights.size() * ns_area_light;
		Vector3D L_out;

		// TODO (Part 3): Write your sampling loop here
		// TODO BEFORE YOU BEGIN
		// UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
		for (int i = 0; i < num_samples; ++i) {
			Vector3D w_in = hemisphereSampler->get_sample();
			double   pdf = 1.0 / (2.0 * PI);
			double   cosTheta = w_in.z;
			if (cosTheta <= 0) continue;

			Vector3D dir_world = o2w * w_in;
			Ray shadow(hit_p + dir_world * EPS_F, dir_world);
			shadow.min_t = EPS_F;
			shadow.max_t = INF_D;

			Intersection light_isect;
			if (!bvh->intersect(shadow, &light_isect)) continue;

			Vector3D L_i = light_isect.bsdf->get_emission();
			if (L_i == Vector3D()) continue;

			Vector3D f = isect.bsdf->f(w_out, w_in);                 // ��/��
			L_out += f * L_i * cosTheta / pdf;
		}

		return L_out / (double)num_samples;

	}

	Vector3D
		PathTracer::estimate_direct_lighting_importance(const Ray& r,
			const Intersection& isect) {
		// Estimate the lighting from this intersection coming directly from a light.
		// To implement importance sampling, sample only from lights, not uniformly in
		// a hemisphere.

		// make a coordinate system for a hit point
		// with N aligned with the Z direction.
		Matrix3x3 o2w;
		make_coord_space(o2w, isect.n);
		Matrix3x3 w2o = o2w.T();

		// w_out points towards the source of the ray (e.g.,
		// toward the camera if this is a primary ray)
		const Vector3D hit_p = r.o + r.d * isect.t;
		const Vector3D w_out = w2o * (-r.d);
		Vector3D L_out;
		for (SceneLight* light : scene->lights) {

			int ns = light->is_delta_light() ? 1
				: ns_area_light;

			for (int j = 0; j < ns; ++j) {

				Vector3D  w_i_world;
				double    distToLight, pdf;
				Vector3D  L_i = light->sample_L(hit_p, &w_i_world, &distToLight, &pdf);
				if (pdf == 0) continue;

				double cosTheta = dot(isect.n, w_i_world);
				if (cosTheta <= 0) continue;

				Ray shadow(hit_p + w_i_world * EPS_F, w_i_world);
				shadow.min_t = EPS_F;
				shadow.max_t = distToLight - EPS_F;

				Intersection tmp;
				if (bvh->intersect(shadow, &tmp)) continue;

				Vector3D w_in_local = w2o * w_i_world;
				Vector3D f = isect.bsdf->f(w_out, w_in_local);
				L_out += f * L_i * cosTheta / pdf;
			}
		}

		return L_out / (double)ns_area_light;

	}

	Vector3D PathTracer::zero_bounce_radiance(const Ray& r,
		const Intersection& isect) {
		// TODO: Part 3, Task 2
		// Returns the light that results from no bounces of light


		return isect.bsdf->get_emission();
	}

	Vector3D PathTracer::one_bounce_radiance(const Ray& r,
		const Intersection& isect) {
		// TODO: Part 3, Task 3
		// Returns either the direct illumination by hemisphere or importance sampling
		// depending on `direct_hemisphere_sample`


		if (direct_hemisphere_sample) {
			return estimate_direct_lighting_hemisphere(r, isect);
		}
		else {
			return estimate_direct_lighting_importance(r, isect);
		}

	}

	Vector3D PathTracer::at_least_one_bounce_radiance(const Ray& r,
		const Intersection& isect) {
		Matrix3x3 o2w;
		make_coord_space(o2w, isect.n);
		Matrix3x3 w2o = o2w.T();

		Vector3D hit_p = r.o + r.d * isect.t;
		Vector3D w_out = w2o * (-r.d);

		Vector3D L_out(0, 0, 0);

		// TODO: Part 4, Task 2
		// Returns the one bounce radiance + radiance from extra bounces at this point.
		// Should be called recursively to simulate extra bounces.


		return L_out;
	}

	Vector3D PathTracer::est_radiance_global_illumination(const Ray& r) {
		Intersection isect;
		Vector3D L_out;

		// You will extend this in assignment 3-2.
		// If no intersection occurs, we simply return black.
		// This changes if you implement hemispherical lighting for extra credit.

		// The following line of code returns a debug color depending
		// on whether ray intersection with triangles or spheres has
		// been implemented.
		//
		// REMOVE THIS LINE when you are ready to begin Part 3.

		if (!bvh->intersect(r, &isect))
			return envLight ? envLight->sample_dir(r) : L_out;


		L_out = zero_bounce_radiance(r, isect);

		// TODO (Part 3): Return the direct illumination.
		L_out += one_bounce_radiance(r, isect);

		// TODO (Part 4): Accumulate the "direct" and "indirect"
		// parts of global illumination into L_out rather than just direct

		return L_out;
	}

	void PathTracer::raytrace_pixel(size_t x, size_t y) {
		// TODO (Part 1.2):
		// Make a loop that generates num_samples camera rays and traces them
		// through the scene. Return the average Vector3D.
		// You should call est_radiance_global_illumination in this function.

		// TODO (Part 5):
		// Modify your implementation to include adaptive sampling.
		// Use the command line parameters "samplesPerBatch" and "maxTolerance"
		int num_samples = ns_aa;          // total samples to evaluate
		Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
		Vector3D sum_samples(0, 0, 0);

		for (int i = 0; i < num_samples; i++)
		{
			Ray new_ray = camera->generate_ray((double)(x + random_uniform()) / sampleBuffer.w, (double)(y + random_uniform()) / sampleBuffer.h);
			sum_samples += est_radiance_global_illumination(new_ray);
		}
		sum_samples /= num_samples;
		sampleBuffer.update_pixel(sum_samples, x, y);
		sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
		//sampleBuffer.update_pixel(Vector3D(0.2, 1.0, 0.8), x, y);
		//sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;

	}

	void PathTracer::autofocus(Vector2D loc) {
		Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
		Intersection isect;

		bvh->intersect(r, &isect);

		camera->focalDistance = isect.t;
	}

} // namespace CGL
