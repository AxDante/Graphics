
#include "simpleSystem.h"

using namespace std;

SimpleSystem::SimpleSystem()
{
	m_numParticles = 1;
	vector<Vector3f> startState(1);
	startState[0] = Vector3f(1, 1, 1);
	setState(startState);
};


// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> SimpleSystem::evalF(vector<Vector3f> state)
{
	// f(X,t) = (-y, x, 0);
	vector<Vector3f> f;

	for (int i = 0; i < state.size(); i++) {
		Vector3f f_der = Vector3f(-state[i][1], state[i][0], 0);
		f.push_back(f_der);
	}

	return f;
}

void SimpleSystem::dragMotion()
{
	return;
}
void SimpleSystem::breeze()
{
	return;
}
// render the system (ie draw the particles)
void SimpleSystem::draw()
{
	vector<Vector3f> state = getState();
	Vector3f pos = state[0];//YOUR PARTICLE POSITION
	glPushMatrix();
	glTranslatef(pos[0], pos[1], pos[2] );
	glutSolidSphere(0.075f,10.0f,10.0f);
	glPopMatrix();

}

