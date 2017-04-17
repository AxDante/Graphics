#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include <vector>
#include <vecmath.h>

using namespace std;

class ParticleSystem
{
public:

	ParticleSystem(int numParticles = 0 );

	int m_numParticles;

	
	// for a given state, evaluate derivative f(X,t)
	virtual vector<Vector3f> evalF(vector<Vector3f> state) = 0;
	
	// getter method for the system's state
	vector<Vector3f> getState() { return m_vVecState; };
	
	// setter method for the system's state
	void setState(const vector<Vector3f>  & newState) { m_vVecState = newState; };
	void setParticlePos(int i, Vector3f newPos) { m_vVecState[i] = newPos; }

	virtual void breeze() = 0;
	virtual void dragMotion() = 0;
	virtual void draw() = 0;
	vector<Vector3f> m_vVecState;
	virtual void takeTimeStep() = 0;
protected:
};

#endif
