#ifndef WAVESYSTEMPARALLEL_H
#define WAVESYSTEMPARALLEL_H

#include <vecmath.h>
#include <vector>

#ifdef _WIN32
#include "GL/freeglut.h"
#else
#include <GL/glut.h>
#endif

#include "particleSystem.h"
#include "Wall.h"

class WaveSystemParallel : public ParticleSystem
{
public:

	WaveSystemParallel(int row, int column, float mass, float step);
	vector<Vector3f> evalF(vector<Vector3f> state) { return vector<Vector3f>{}; };
	void draw();
	void breeze() {}
	void dragMotion();
	//void setSprings(const vector<Vector4f>  & newSpring) { springs = newSpring; }
	void setBoxPos(Vector3f BoxPos) {}
	void setBoxRot(Vector3f BoxRot) {}

	void takeTimeStep();


	void toggleBreeze() { switch_breeze = !switch_breeze; }
	void toggleWire() { count_wire = (count_wire < 6) ? count_wire + 1 : 0; }
	void toggleBoxFrame() { count_box = (count_box < 3)? count_box + 1 : 0; }
	void takeStep(float stepSize);

	int index(int cur_row, int cur_col, int tot_row, int tot_col);

	vector< float > getEnergy() { return energyStored; }
	void setEnergy(int i, float energy) { energyStored[i] = energy; }

	//vector< Vector3f > vibrationDir;
	//vector< float > vibrationValue;
	vector< float > energyStored;
	vector< vector< float >  > phaseStored;
	vector< vector< float > > magnitudeStored;
	vector< vector< int > > spreadCounter;
	vector< int > center;
	int baseSpreadSpeed;

protected:

	int numParticles;

	int rows;
	int columns;
	//int layers;

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

	float timeStep;
	float sysCounter;

private:

	vector< Vector3f > m_face;
	vector< vector <int> > springs;
	vector<Wall> walls;

	vector< Vector3f > initPos;
	vector< Vector3f > initVel;
	vector< Vector3f > forcedPos;
	vector< Vector3f > forcedVel;
	vector< Vector3f > statePos;
	vector< Vector3f > stateVel;
	



};


#endif
