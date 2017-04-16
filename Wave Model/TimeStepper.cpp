#include "TimeStepper.hpp"


void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize)
{
	vector<Vector3f> currentState = particleSystem->getState();
	vector<Vector3f> f0 = particleSystem->evalF(currentState);
	vector<Vector3f> nextState;

	for (int i = 0; i < f0.size(); i++) {
		nextState.push_back(currentState[i] + stepSize * f0[i]);
	}

	particleSystem->setState(nextState);
}


void Trapzoidal::takeStep(ParticleSystem* particleSystem, float stepSize)
{
	vector<Vector3f> currentState = particleSystem->getState();
	vector<Vector3f> f0 = particleSystem->evalF(currentState);
	vector<Vector3f> f0_state;

	for (int i = 0; i < f0.size(); i++) {
		f0_state.push_back(currentState[i] + stepSize * f0[i]);
	}

	vector<Vector3f> f1 = particleSystem->evalF(f0_state);
	vector<Vector3f> nextState;

	for (int i = 0; i < f0.size(); i++) {
		nextState.push_back(currentState[i] + 0.5 * stepSize * (f0[i] + f1[i]));
	}

	particleSystem->setState(nextState);
}
