#ifndef BOXSPRINGSYSTEM_H
#define BOXSPRINGSYSTEM_H

#include <vecmath.h>
#include <vector>

#ifdef _WIN32
#include "GL/freeglut.h"
#else
#include <GL/glut.h>
#endif

#include "particleSystem.h"
#include "Wall.h"

class BoxSpringSystem : public ParticleSystem
{
public:

	BoxSpringSystem(float width, float length, float height, int n_particles, float mass);
	vector<Vector3f> evalF(vector<Vector3f> state);
	void draw();
	void breeze() {}
	void dragMotion();
	void setSprings(const vector<Vector4f>  & newSpring) { springs = newSpring; }
	void setBoxPos(Vector3f BoxPos) {}
	void setBoxRot(Vector3f BoxRot) {}

	void toggleWire() { count_wire = (count_wire < 6) ? count_wire + 1 : 0; }
	void toggleBoxFrame() { count_box = (count_box < 3)? count_box + 1 : 0; }

	int index(int cur_row, int cur_col, int cur_lyr, int tot_row, int tot_col, int tot_lyr);


protected:

	int m_clothNumParticles;

	int rows;
	int columns;
	int layers;

	float part_mass;

	bool switch_breeze;

	// Variables for particles
	int n_particles;
	float part_size;

	// Variables for box configuration
	int count_box;
	int count_wire;
	float box_length;
	float box_thickness;

private:
	vector<Vector3f> m_face;
	vector<Vector4f> springs;
	vector<Wall> walls;


};

#endif
