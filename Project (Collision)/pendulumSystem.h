#ifndef PENDULUMSYSTEM_H
#define PENDULUMSYSTEM_H

#include <vecmath.h>
#include <vector>

#ifdef _WIN32
#include "GL/freeglut.h"
#else
#include <GL/glut.h>
#endif

#include "particleSystem.h"

class PendulumSystem: public ParticleSystem
{
public:
	PendulumSystem(int numParticles);
	
	vector<Vector3f> evalF(vector<Vector3f> state);

	void dragMotion();
	void breeze();
	void draw();
	void toggleWire() { count_wire = (count_wire < 1) ? count_wire + 1 : 0; };

protected:

	int count_wire = 0;

};

#endif
