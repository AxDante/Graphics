#ifndef MULTIPLEPARTICLESYSTEM_H
#define MULTIPLEPARTICLESYSTEM_H

#include <vecmath.h>
#include <vector>

#ifdef _WIN32
#include "GL/freeglut.h"
#else
#include <GL/glut.h>
#endif

#include "particleSystem.h"

class MultipleParticleSystem : public ParticleSystem
{
public:

	MultipleParticleSystem(int rows, int columns, int thickness, float mass);
	vector<Vector3f> evalF(vector<Vector3f> state);
	void draw();
	void dragMotion();
	void breeze();
	void setSprings(const vector<Vector4f>  & newSpring) { springs = newSpring; };
	void toggleBreeze() { switch_breeze = !switch_breeze; };
	void toggleWire() { count_wire = (count_wire < 6) ? count_wire + 1 : 0; };
	int index(int cur_row, int cur_col, int cur_lyr, int tot_row, int tot_col, int tot_lyr);

protected:

	int m_clothNumParticles;
	int m_rows;
	int m_columns;
	int m_layers;
	float m_mass;
	bool switch_breeze;
	int count_wire;


private:
	vector<Vector3f> m_face;
	vector<Vector4f> springs;

};

#endif
