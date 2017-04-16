#ifndef CLOTHSYSTEM_H
#define CLOTHSYSTEM_H

#include <vecmath.h>
#include <vector>

#ifdef _WIN32
#include "GL/freeglut.h"
#else
#include <GL/glut.h>
#endif

#include "particleSystem.h"

class ClothSystem: public ParticleSystem
{
public:

	ClothSystem(int rows, int columns, float mass);
	vector<Vector3f> evalF(vector<Vector3f> state);

	void draw();
	void dragMotion();
	void breeze();
	void setSprings(const vector<Vector4f>  & newSpring) { springs = newSpring; }
	void toggleBreeze() { switch_breeze = !switch_breeze; }
	void toggleWire() { count_wire = (count_wire < 6) ? count_wire + 1 : 0; }

	void setXTrans(int x) { ballPos.x() += 0.1 * x;}
	void setZTrans(int z) { ballPos.z() += 0.1 * z;}
	int index(int cur_row, int cur_col, int tot_row, int tot_col);

protected:

	int m_clothNumParticles;
	int m_rows;
	int m_columns;
	float m_mass;
	bool switch_breeze;
	bool switch_ball;
	int count_wire;
	Vector3f ballPos;
	float ballRad;

private:
	vector<Vector3f> m_face;
	vector<Vector4f> springs;

};


#endif
