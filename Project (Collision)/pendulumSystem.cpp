
#include "pendulumSystem.h"

PendulumSystem::PendulumSystem(int numParticles):ParticleSystem(numParticles)
{
	
	m_numParticles = numParticles;
	vector<Vector3f> startPosVel; 
	// fill in code for initializing the state based on the number of particles
	for (int i = 0; i < m_numParticles; i++) {

		// for this system, we care about the position and the velocity
		Vector3f initPos = Vector3f(0.25*i, 0, 0);
		Vector3f initVel = Vector3f(0, 0, 0);

		startPosVel.push_back(initPos);
		startPosVel.push_back(initVel);

	}
	setState(startPosVel);
}


// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> PendulumSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;
	
	// Variable Definition

	const float mass = 2; 
	const float G = 9.80665;		// Graviational acceleration
	const float k_drag = 1;			// drag constant
	const float k_spring = 50;		// spring stiffness
	const float r = 0.2;			// spring rest length

	for (int i = 0; i < m_numParticles; i++) {

		Vector3f pos = state[2 * i];
		Vector3f vel = state[2 * i + 1];

		Vector3f F_grav = mass * Vector3f(0, -G, 0);
		Vector3f F_vic = -k_drag * vel;

		Vector3f F_spring = Vector3f(0,0,0);

		if (i != 0) {
			Vector3f prevPos = state[2 * i - 2];
			Vector3f dis = pos - prevPos;
			F_spring += -k_spring * (dis.abs() - r) * (dis / dis.abs());
		}
		if (i != m_numParticles - 1) {
			Vector3f nextPos = state[2 * i + 2];
			Vector3f dis = pos - nextPos;
			F_spring += -k_spring * (dis.abs() - r) * (dis / dis.abs());
		}



		Vector3f totalForce = F_grav + F_vic + F_spring;
		Vector3f Pos_def = vel;
		Vector3f Vel_def = totalForce / mass;

		if (i == 0) {
			// fixed point
			Vel_def = Vector3f(0.0f, 0.0f, 0.0f);
		}

		f.push_back(Pos_def);
		f.push_back(Vel_def);
		
	}
	return f;
}

void PendulumSystem::dragMotion()
{
	return;
}

void PendulumSystem::breeze()
{
	return;
}
// render the system (ie draw the particles)
void PendulumSystem::draw()
{
	vector<Vector3f> state = getState();
	if (count_wire == 0 || count_wire == 1) {
		for (int i = 0; i < state.size() / 2; i++) {
			Vector3f pos = state[i * 2];//  position of particle i. YOUR CODE HERE
			glPushMatrix();
			glTranslatef(pos[0], pos[1], pos[2]);
			glutSolidSphere(0.075f, 10.0f, 10.0f);
			glPopMatrix();

		}
	}
	if (count_wire == 1 ) {

		for (unsigned int i = 0; i < state.size()/2 - 1; i++) {
			Vector3f vv1 = state[i * 2];
			Vector3f vv2 = state[i * 2 + 2];

			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glDisable(GL_LIGHTING);
			glLineWidth(1.0f);
			glPushMatrix();
			glBegin(GL_LINES);
			glColor4f(0.5, 1.0, 0.5, 1.0);
			glVertex3f(vv1.x(), vv1.y(), vv1.z());
			glVertex3f(vv2.x(), vv2.y(), vv2.z());
			glEnd();
			glPopMatrix();
			glPopAttrib();

		}
	}
}
