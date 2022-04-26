//=============================================================================================
// Computer Graphics Sample Program: GPU ray casting
//=============================================================================================
#include "framework.h"

// vertex shader in GLSL
const char* vertexSource = R"(
	#version 330
    precision highp float;

	uniform vec3 wLookAt, wRight, wUp;          // pos of eye

	layout(location = 0) in vec2 cCamWindowVertex;	// Attrib Array 0
	out vec3 p;

	void main() {
		gl_Position = vec4(cCamWindowVertex, 0, 1);
		p = wLookAt + wRight * cCamWindowVertex.x + wUp * cCamWindowVertex.y;
	}
)";
// fragment shader in GLSL
const char* fragmentSource = R"(
#version 330
    precision highp float;

	struct Material {
		vec3 ka, kd, ks;
		float  shininess;
		vec3 F0;
		int rough, reflective;
	};

	struct Light {
		vec3 position;
		vec3 Le, La;
	};

	struct Sphere {
		vec3 center;
		float radius;
	};

	struct Plain {
		vec3 normal;
		vec3 point;
		float radius;
	};

	struct Cylinder {
		vec3 A;
		vec3 B;
		float radius;
	};

	struct Paraboloid {
		vec3 center;
		vec3 direction;
		float radius;
	};

	struct Hit {
		float t;
		vec3 position, normal;
		int mat;	// material index
	};

	struct Ray {
		vec3 start, dir;
	};

	struct Quaternion {
		float s;
		vec3 d;
	};

	const int nMaxObjects = 500;

	uniform vec3 wEye; 
	uniform Light lights[2];     
	uniform Material materials[2];  // diffuse, specular, ambient ref
	uniform int nObjects;
	uniform Sphere objects[3];
	uniform Plain plains[2];
	uniform Cylinder cylinders[3];
	uniform Paraboloid paraboloids[1];

	in  vec3 p;					// point on camera window corresponding to the pixel
	out vec4 fragmentColor;		// output that goes to the raster memory as told by glBindFragDataLocation

	Hit intersectSphere(const Sphere object, const Ray ray, Hit hit) {
		vec3 dist = ray.start - object.center;
		float a = dot(ray.dir, ray.dir);
		float b = dot(dist, ray.dir) * 2.0f;
		float c = dot(dist, dist) - object.radius * object.radius;
		float discr = b * b - 4.0f * a * c;
		if (discr < 0) return hit;
		float sqrt_discr = sqrt(discr);
		float t1 = (-b + sqrt_discr) / 2.0f / a;	// t1 >= t2 for sure
		float t2 = (-b - sqrt_discr) / 2.0f / a;
		if (t1 <= 0) return hit;
		if (t2 > 0) t1 = t2;
		if(t1 < hit.t || hit.t < 0){
			hit.t = t1;
			hit.position = ray.start + ray.dir * hit.t;
			hit.normal = (hit.position - object.center) / object.radius;
			hit.mat = 0;
		}
		return hit;
	}

	Hit intersectPlain(const Plain plain, const Ray ray, Hit hit) {
		float num = dot(plain.point, plain.normal)-dot(ray.start, plain.normal);
		float denum = dot(ray.dir, plain.normal);
		if (denum  == 0) return  hit;
		float t = num/denum;
		if(t < 0) return hit;
		if(plain.radius > 0){
			vec3 position = ray.start + ray.dir * t;
			float radius = sqrt(dot(plain.point-position, plain.point-position));
			if(radius > plain.radius) return hit;
		}
		if(t < hit.t ||  hit.t < 0){
			hit.t = t;
			hit.position = ray.start + ray.dir * hit.t;
			hit.normal = plain.normal;
			if(plain.radius < 0) hit.mat = 1;
			else hit.mat = 0;
		}
		return hit;
	}

	Hit intersectCylinder(const Cylinder cylinder, const Ray ray, Hit hit) {
		vec3 base_i = normalize(cross(ray.start - cylinder.A, cylinder.B - cylinder.A));
		vec3 base_j = normalize(cross(base_i, cylinder.B - cylinder.A));
		vec2 start_proj = vec2(dot(ray.start, base_i), dot(ray.start, base_j));
		vec2 dir_proj = vec2(dot(ray.dir, base_i), dot(ray.dir, base_j));
		vec2 A_proj = vec2(dot(cylinder.A, base_i), dot(cylinder.A, base_j));
		vec2 dist = start_proj - A_proj;
		float a = dot(dir_proj, dir_proj);
		float b = dot(dist, dir_proj) * 2.0f;
		float c = dot(dist, dist) - cylinder.radius * cylinder.radius;
		float discr = b * b - 4.0f * a * c;
		if (discr < 0) return hit;
		float sqrt_discr = sqrt(discr);
		float t1 = (-b + sqrt_discr) / 2.0f / a;	// t1 >= t2 for sure
		float t2 = (-b - sqrt_discr) / 2.0f / a;
		if (t1 <= 0) return hit;
		if (t2 > 0) t1 = t2;
		vec3 position = ray.start + ray.dir * t1;
		float h = dot((position - cylinder.A), normalize(cylinder.B - cylinder.A));
		if(h < 0 || h > length(cylinder.B - cylinder.A)) return hit;
		if(t1 < hit.t || hit.t < 0){
			hit.t = t1;
			hit.position = position;
			hit.normal = normalize(position - cylinder.A - h * normalize(cylinder.B - cylinder.A));
			hit.mat = 0;
		}
		return hit;
	}

	Quaternion multiply(Quaternion q1, Quaternion q2) {
		Quaternion result;
		result.s = q1.s * q2.s - dot(q1.d, q2.d);
		result.d = q1.s * q2.d + q2.s * q1.d + cross(q1.d, q2.d);
		return result;
	}

	vec3 rotate(vec3 t, float cos_theta, vec3 point) {
		float sin_theta_half = sqrt((1.0f - cos_theta) / 2.0f);
		float cos_theta_half = sqrt(1.0f - pow(sin_theta_half, 2.0f));
		Quaternion q, p, q_inv;
		q.s = cos_theta_half;
		q.d = t * sin_theta_half;
		p.s = 0;
		p.d = point;
		q_inv.s = cos_theta_half;
		q_inv.d = -1.0 * t * sin_theta_half;

		p = multiply(q, p);
		p = multiply(p, q_inv);
		return p.d;
	}

	Hit intersectParaboloid(const Paraboloid paraboloid, Ray ray, Hit hit) {
		vec3 move = -1.0f * paraboloid.center;
		ray.start = ray.start + move;
		vec3 t = normalize(cross(paraboloid.direction, vec3(0, 1, 0))); //forgastengely
		float cos_theta = dot(normalize(paraboloid.direction), normalize(vec3(0, 1, 0))); //forgasszog cosinusa
		ray.start = rotate(t, cos_theta, ray.start);
		ray.dir = rotate(t, cos_theta, ray.dir);

		float a = pow(ray.dir.x, 2.0f) + pow(ray.dir.z, 2.0f);
		float b = 2.0f * ray.start.x * ray.dir.x + 2.0f * ray.start.z * ray.dir.z - ray.dir.y;
		float c = pow(ray.start.x, 2.0f) + pow(ray.start.z, 2.0f) - ray.start.y;
		float discr = b * b - 4.0f * a * c;
		if (discr < 0) return hit;
		float sqrt_discr = sqrt(discr);
		float t1 = (-b + sqrt_discr) / 2.0f / a;	// t1 >= t2 for sure
		float t2 = (-b - sqrt_discr) / 2.0f / a;
		if (t1 <= 0) return hit;
		float t1_y = ray.start.y + t1 * ray.dir.y;
		float t2_y = ray.start.y + t2 * ray.dir.y;
		if (t2_y < paraboloid.radius && t2 > 0) t1 = t2;
		else if (t1_y >= paraboloid.radius) return hit; //innentol tuti t1 a jo
		if(t1 < hit.t || hit.t < 0){
			hit.t = t1;
			hit.position = ray.start + ray.dir * hit.t;
			vec3 dx = vec3(1.0f, 2.0f * hit.position.x, 0.0f);
			vec3 dz = vec3(0.0f, 2.0f * hit.position.z, 1.0f);
			hit.normal = normalize(cross(dx, dz));
			//visszaforgatjuk a pontokat
			t = normalize(cross(vec3(0, 1, 0), paraboloid.direction)); //forgastengely
			cos_theta = dot(normalize(vec3(0, 1, 0)), normalize(paraboloid.direction)); //forgasszog cosinusa
			hit.position = rotate(t, cos_theta, hit.position);
			hit.normal = rotate(t, cos_theta, hit.normal);
			hit.position = hit.position - move;
			hit.mat = 0;
		}
		return hit;
	}

	Hit firstIntersect(Ray ray) {
		Hit bestHit;
		bestHit.t = -1;
		bestHit.mat = 0;    //all objects are rough
        //forciklus a hengerekre is, es a parabolara is egy fgv, es az alapsikra is
        //kapja meg a kovetkezo fgv az elozo legjobb besthit-et (lasd: mirascope)
        //NOTE: LEHET HOGY A SUGARAT ERDEMES TRANSZFORMALNI? NEM PEDIG MAGAT AZ ALAKZATOT?
		for (int o = 0; o < 3; o++) {
			bestHit = intersectSphere(objects[o], ray, bestHit); //  hit.t < 0 if no intersection
		}
		for (int o = 0; o < 2; o++) {
			bestHit = intersectPlain(plains[o], ray, bestHit);
		}
		for (int o = 0; o < 3; o++) {
			bestHit = intersectCylinder(cylinders[o], ray, bestHit);
		}
		for (int o = 0; o < 1; o++) {
			bestHit = intersectParaboloid(paraboloids[o], ray, bestHit);
		}
		if (dot(ray.dir, bestHit.normal) > 0) bestHit.normal = bestHit.normal * (-1);
		return bestHit;
	}

	bool shadowIntersect(Ray ray, float t) {	// for central lights: if hit is greater than t, no intersection
		Hit hit;
		hit.t = -1;
		for (int o = 0; o < nObjects; o++ ){
			hit = intersectSphere(objects[o], ray, hit);
			if(hit.t > 0 && hit.t < t)
			return true;
		}
		for (int o = 0; o < 2; o++) {
			hit = intersectPlain(plains[o], ray, hit);
			if(hit.t > 0 && hit.t < t)
			return true;
		}
		for (int o = 0; o < 3; o++) {
			hit = intersectCylinder(cylinders[o], ray, hit);
			if(hit.t > 0 && hit.t < t)
			return true;
		}
		for (int o = 0; o < 1; o++) {
			hit = intersectParaboloid(paraboloids[o], ray, hit);
			if(hit.t > 0 && hit.t < t)
			return true;
		}
		return false;
	}

	vec3 Fresnel(vec3 F0, float cosTheta) { 
		return F0 + (vec3(1, 1, 1) - F0) * pow(cosTheta, 5);
	}

	const float epsilon = 0.001f;
	const int maxdepth = 5;

	vec3 trace(Ray ray) {
		vec3 weight = vec3(1, 1, 1);
		vec3 outRadiance = vec3(0, 0, 0);
		for(int d = 0; d < maxdepth; d++) {
			Hit hit = firstIntersect(ray);
			if (hit.t < 0) return weight * lights[0].La;
			if (materials[hit.mat].rough == 1) {
				for(int o = 0; o < 2; o++){
					outRadiance += weight * materials[hit.mat].ka * lights[o].La;
					Ray shadowRay;
					shadowRay.start = hit.position + hit.normal * epsilon;
					float t = length(lights[o].position - hit.position);
					float intensity = 1.0f / pow(0.1 * t, 2.0f);
					if(intensity > 1.0f) intensity = 1.0f;	//nem biztos hogy kell
					shadowRay.dir = normalize(lights[o].position - hit.position);
					float cosTheta = dot(hit.normal, shadowRay.dir);
					if (cosTheta > 0 && !shadowIntersect(shadowRay, t)) {
						//DIFFUZ VISSZAVERODES
						outRadiance += weight * lights[o].Le * materials[hit.mat].kd * cosTheta * intensity;
						//SPEKULARIS VISSZAVERODES
						vec3 halfway = normalize(-ray.dir + shadowRay.dir);
						float cosDelta = dot(hit.normal, halfway);
						if (cosDelta > 0) outRadiance += weight * lights[o].Le * materials[hit.mat].ks * pow(cosDelta, materials[hit.mat].shininess) * intensity;
					}
				}
			}
            //SUGARKOVETES: NEM KELL
			if (materials[hit.mat].reflective == 1) {
				weight *= Fresnel(materials[hit.mat].F0, dot(-ray.dir, hit.normal));
				ray.start = hit.position + hit.normal * epsilon;
				ray.dir = reflect(ray.dir, hit.normal);
			}
			else return outRadiance;
		}
	}

	void main() {
		Ray ray;
		ray.start = wEye; 
		ray.dir = normalize(p - wEye);
		fragmentColor = vec4(trace(ray), 1); 
	}
)";

//---------------------------
struct Material {
	//---------------------------
	vec3 ka, kd, ks;
	float  shininess;
	vec3 F0;
	int rough, reflective;
};

//---------------------------
struct RoughMaterial : Material {
	//---------------------------
	RoughMaterial(vec3 _kd, vec3 _ks, float _shininess) {
		ka = _kd * M_PI;
		kd = _kd;
		ks = _ks;
		shininess = _shininess;
		rough = true;
		reflective = false;
	}
};

//---------------------------
struct SmoothMaterial : Material {
	//---------------------------
	SmoothMaterial(vec3 _F0) {
		F0 = _F0;
		rough = false;
		reflective = true;
	}
};

//---------------------------
struct Sphere {
	//---------------------------
	vec3 center;
	float radius;

	Sphere(const vec3& _center, float _radius) { center = _center; radius = _radius; }
};

//---------------------------
struct Plain {
	//---------------------------
	vec3 normal;
	vec3 point;
	float radius;

	Plain(const vec3& _normal, const vec3& _point, float _radius) { normal = _normal; point = _point; radius = _radius; }
};

//---------------------------
struct Cylinder {
	//---------------------------
	vec3 A;
	vec3 B;
	float radius;

	Cylinder(const vec3& _A, const vec3& _B, float _radius) { A = _A; B = _B; radius = _radius; }
};

//---------------------------
struct Paraboloid {
	//---------------------------
	vec3 center;
	vec3 direction;
	float radius;

	Paraboloid(const vec3& _center, const vec3& _direction, float _radius) { center = _center; direction = _direction; radius = _radius; }
};

//---------------------------
struct Quaternion {
	//---------------------------
	float s;
	vec3 d;
};

//---------------------------
struct Camera {
	//---------------------------
	vec3 eye, lookat, right, up;
	float fov;
public:
	void set(vec3 _eye, vec3 _lookat, vec3 vup, float _fov) {
		eye = _eye;
		lookat = _lookat;
		fov = _fov;
		vec3 w = eye - lookat;
		float f = length(w);
		right = normalize(cross(vup, w)) * f * tanf(fov / 2);
		up = normalize(cross(w, right)) * f * tanf(fov / 2);
	}
	void Animate(float dt) {
		eye = vec3((eye.x - lookat.x) * cos(dt) + (eye.z - lookat.z) * sin(dt) + lookat.x,
			eye.y,
			-(eye.x - lookat.x) * sin(dt) + (eye.z - lookat.z) * cos(dt) + lookat.z);
		set(eye, lookat, vec3(0, 1, 0), fov);
	}
};

//---------------------------
struct Light {
	//---------------------------
	vec3 position;
	vec3 Le, La;
	Light(vec3 _position, vec3 _Le, vec3 _La) {
		position = _position;
		Le = _Le; La = _La;
	}
};

//---------------------------
class Shader : public GPUProgram {
	//---------------------------
public:
	void setUniformMaterials(const std::vector<Material*>& materials) {
		char name[256];
		for (unsigned int mat = 0; mat < materials.size(); mat++) {
			sprintf(name, "materials[%d].ka", mat); setUniform(materials[mat]->ka, name);
			sprintf(name, "materials[%d].kd", mat); setUniform(materials[mat]->kd, name);
			sprintf(name, "materials[%d].ks", mat); setUniform(materials[mat]->ks, name);
			sprintf(name, "materials[%d].shininess", mat); setUniform(materials[mat]->shininess, name);
			sprintf(name, "materials[%d].F0", mat); setUniform(materials[mat]->F0, name);
			sprintf(name, "materials[%d].rough", mat); setUniform(materials[mat]->rough, name);
			sprintf(name, "materials[%d].reflective", mat); setUniform(materials[mat]->reflective, name);
		}
	}

	void setUniformLight(const std::vector<Light*>& lights) {
		char name[256];
		for (unsigned int o = 0; o < lights.size(); o++) {
			sprintf(name, "lights[%d].La", o);  setUniform(lights[o]->La, name);
			sprintf(name, "lights[%d].Le", o);  setUniform(lights[o]->Le, name);
			sprintf(name, "lights[%d].position", o);  setUniform(lights[o]->position, name);
		}
	}

	void setUniformCamera(const Camera& camera) {
		setUniform(camera.eye, "wEye");
		setUniform(camera.lookat, "wLookAt");
		setUniform(camera.right, "wRight");
		setUniform(camera.up, "wUp");
	}

	void setUniformObjects(const std::vector<Sphere*>& objects, const std::vector<Plain*>& plains, const std::vector<Cylinder*> cylinders, const std::vector<Paraboloid*> paraboloids) {
		setUniform((int)objects.size(), "nObjects");
		char name[256];
		for (unsigned int o = 0; o < objects.size(); o++) {
			sprintf(name, "objects[%d].center", o);  setUniform(objects[o]->center, name);
			sprintf(name, "objects[%d].radius", o);  setUniform(objects[o]->radius, name);
		}
		for (unsigned int p = 0; p < plains.size(); p++) {
			sprintf(name, "plains[%d].normal", p);  setUniform(plains[p]->normal, name);
			sprintf(name, "plains[%d].point", p);  setUniform(plains[p]->point, name);
			sprintf(name, "plains[%d].radius", p);  setUniform(plains[p]->radius, name);
		}

		for (unsigned int p = 0; p < cylinders.size(); p++) {
			sprintf(name, "cylinders[%d].A", p);  setUniform(cylinders[p]->A, name);
			sprintf(name, "cylinders[%d].B", p);  setUniform(cylinders[p]->B, name);
			sprintf(name, "cylinders[%d].radius", p);  setUniform(cylinders[p]->radius, name);
		}

		for (unsigned int p = 0; p < paraboloids.size(); p++) {
			sprintf(name, "paraboloids[%d].center", p);  setUniform(paraboloids[p]->center, name);
			sprintf(name, "paraboloids[%d].direction", p);  setUniform(paraboloids[p]->direction, name);
			sprintf(name, "paraboloids[%d].radius", p);  setUniform(paraboloids[p]->radius, name);
		}
	}
};

float rnd() { return (float)rand() / RAND_MAX; }

Shader shader; // vertex and fragment shaders

//---------------------------
class Scene {
	//---------------------------
	std::vector<Sphere*> objects;
	std::vector<Light*> lights;
	std::vector<Plain*> plains;
	std::vector<Cylinder*> cylinders;
	std::vector<Paraboloid*> paraboloids;
	Camera camera;
	std::vector<Material*> materials;
	vec3 t_AB = normalize(vec3(1, -5, 1)); //0, -5, 2
	vec3 t_BC = normalize(vec3(-1, 2, 1)); //1, 3, 1
	vec3 t_CL = normalize(vec3(1, 2, -1)); //-1, 5, 2
public:
	void build() {
		vec3 eye = vec3(0, 6, 8);
		vec3 vup = vec3(0, 1, 0);
		vec3 lookat = vec3(0, 1, 0);
		float fov = 60 * (float)M_PI / 180;
		camera.set(eye, lookat, vup, fov);

		float sphere_rad = 0.2f;
		float bottom_rad = 1.2f;
		float arms_rad = 0.08f;
		vec3 A = vec3(0, sphere_rad, 0);
		vec3 B = vec3(-1.0, 2.0, 0);
		vec3 C = vec3(-2.0, 3.0, 0.5); //-1.0, 3.0, 2.0
		vec3 Lamp_Dir = vec3(0, -2, 1); //-1, -1, 1
		vec3 L = C + normalize(Lamp_Dir);

		lights.push_back(new Light(vec3(8, 8, 8), vec3(1, 1, 1), vec3(0.2f, 0.15f, 0.15f))); //kulso feny
		lights.push_back(new Light(L, vec3(5, 5, 5), vec3(0,0,0)));	//lampa fenye

		vec3 kd_floor(0.3f, 0.2f, 0.1f), kd_lamp(0.1f, 0.2f, 0.3f), ks(1, 1, 1);
		materials.push_back(new RoughMaterial(kd_lamp, ks, 50));	//minden mas
		materials.push_back(new RoughMaterial(kd_floor, ks, 50)); //talaj

		objects.push_back(new Sphere(A, sphere_rad));
		objects.push_back(new Sphere(B, sphere_rad));
		objects.push_back(new Sphere(C, sphere_rad));

		plains.push_back(new Plain(vec3(0, 1, 0), vec3(0, 0, 0), -1.0f));
		plains.push_back(new Plain(vec3(0, 1, 0), vec3(0, sphere_rad, 0), bottom_rad));

		cylinders.push_back(new Cylinder(vec3(0, 0, 0), A, bottom_rad));
		cylinders.push_back(new Cylinder(A,B, arms_rad));
		cylinders.push_back(new Cylinder(B, C, arms_rad));

		paraboloids.push_back(new Paraboloid(C, Lamp_Dir, 1.5f));
	}

	void setUniform(Shader& shader) {
		shader.setUniformObjects(objects, plains, cylinders, paraboloids);
		shader.setUniformMaterials(materials);
		shader.setUniformLight(lights);
		shader.setUniformCamera(camera);
	}

	Quaternion multiply(Quaternion q1, Quaternion q2) {
		Quaternion result;
		result.s = q1.s * q2.s - dot(q1.d, q2.d);
		result.d = q1.s * q2.d + q2.s * q1.d + cross(q1.d, q2.d);
		return result;
	}

	//start: a point on the rotation axis
	//t: rotation axis
	//cos_theta: cos() of rotation angle
	//_point: point to rotate
	vec3 rotate(vec3 start, vec3 t, float cos_theta, vec3 _point) {
		vec3 point = _point - start;
		float sin_theta_half = sqrt((1.0f - cos_theta) / 2.0f);
		float cos_theta_half = sqrt(1.0f - pow(sin_theta_half, 2.0f));
		Quaternion q, p, q_inv;
		q.s = cos_theta_half;
		q.d = t * sin_theta_half;
		p.s = 0;
		p.d = point;
		q_inv.s = cos_theta_half;
		q_inv.d = -1.0 * t * sin_theta_half;

		p = multiply(q, p);
		p = multiply(p, q_inv);
		return p.d + start;
	}

	void Animate(float dt) {
		camera.Animate(dt);
		vec3 O = vec3(0, 0, 0);
		vec3 A = objects[0]->center;
		vec3 B = objects[1]->center;
		vec3 C = objects[2]->center;
		vec3 L = lights[1]->position;
		//first axis rotation
		B = rotate(A, t_AB, cosf(dt), B);
		C = rotate(A, t_AB, cosf(dt), C);
		L = rotate(A, t_AB, cosf(dt), L);
		t_BC = rotate(O, t_AB, cosf(dt), t_BC); //tengely egy vektor ezert eleg origo korul forgatni
		t_CL = rotate(O, t_AB, cosf(dt), t_CL);
		//second axis rotation
		C = rotate(B, t_BC, cosf(dt), C);
		L = rotate(B, t_BC, cosf(dt), L);
		t_CL = rotate(O, t_BC, cosf(dt), t_CL);
		//third axis rotation
		L = rotate(C, t_CL, cosf(dt), L);
		//set the  new positions
		objects[1]->center = B;
		cylinders[1]->B = B;
		cylinders[2]->A = B;
		objects[2]->center = C;
		cylinders[2]->B = C;
		paraboloids[0]->center = C;
		paraboloids[0]->direction = L - C;
		lights[1]->position = L;
	}
};

Scene scene;

//---------------------------
class FullScreenTexturedQuad {
	//---------------------------
	unsigned int vao = 0;	// vertex array object id and texture id
public:
	void create() {
		glGenVertexArrays(1, &vao);	// create 1 vertex array object
		glBindVertexArray(vao);		// make it active

		unsigned int vbo;		// vertex buffer objects
		glGenBuffers(1, &vbo);	// Generate 1 vertex buffer objects

		// vertex coordinates: vbo0 -> Attrib Array 0 -> vertexPosition of the vertex shader
		glBindBuffer(GL_ARRAY_BUFFER, vbo); // make it active, it is an array
		float vertexCoords[] = { -1, -1,  1, -1,  1, 1,  -1, 1 };	// two triangles forming a quad
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertexCoords), vertexCoords, GL_STATIC_DRAW);	   // copy to that part of the memory which is not modified 
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, NULL);     // stride and offset: it is tightly packed
	}

	void Draw() {
		glBindVertexArray(vao);	// make the vao and its vbos active playing the role of the data source
		glDrawArrays(GL_TRIANGLE_FAN, 0, 4);	// draw two triangles forming a quad
	}
};

FullScreenTexturedQuad fullScreenTexturedQuad;

// Initialization, create an OpenGL context
void onInitialization() {
	glViewport(0, 0, windowWidth, windowHeight);
	scene.build();
	fullScreenTexturedQuad.create();

	// create program for the GPU
	shader.create(vertexSource, fragmentSource, "fragmentColor");
	shader.Use();
}

// Window has become invalid: Redraw
void onDisplay() {
	static int nFrames = 0;
	nFrames++;
	static long tStart = glutGet(GLUT_ELAPSED_TIME);
	long tEnd = glutGet(GLUT_ELAPSED_TIME);
	printf("%d msec\r", (tEnd - tStart) / nFrames);

	glClearColor(1.0f, 0.5f, 0.8f, 1.0f);							// background color 
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear the screen

	scene.setUniform(shader);
	fullScreenTexturedQuad.Draw();

	glutSwapBuffers();									// exchange the two buffers
}

// Key of ASCII code pressed
void onKeyboard(unsigned char key, int pX, int pY) {
}

// Key of ASCII code released
void onKeyboardUp(unsigned char key, int pX, int pY) {

}

// Mouse click event
void onMouse(int button, int state, int pX, int pY) {
}

// Move mouse with key pressed
void onMouseMotion(int pX, int pY) {
}

// Idle event indicating that some time elapsed: do animation here
void onIdle() {
	scene.Animate(0.01f);
	glutPostRedisplay();
}
