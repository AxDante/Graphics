
#define _USE_MATH_DEFINES

#include <iostream>
#include <cstdio>
#include <ctime>
#include <math.h>
#include "WaveSystemWall.h"
#include "Wall.h"

WaveSystemWall::WaveSystemWall(int row, int col, float mass, float step)
{
	part_mass = mass;
	part_size = 0.2;
	
	columns = col;
	rows = row;

	sysCounter = 0.0f;
	timeStep = step;
	baseSpreadSpeed = 5;
	vector<Vector3f> startPosVel(numParticles * 2);
	//vector<Vector4f> springs;
	vector<Vector3f> m_face; 

	vector< Vector3f > initPos;
	vector< Vector3f > initVel;
	vector< Vector3f > forcedPos;
	vector< Vector3f > forcedVel;
	vector< Vector3f > statePos;
	vector< Vector3f > stateVel;
	//vector< Vector3f > vibrationDir;
	//vector< float > vibrationValue;
	//vector< float > energyStored;

	switchBoarderReflection = false;


	m_vVecState.empty();

	const float r_struct = 0.7f;				// structural spring rest length
	const float r_init = 1.00f * r_struct;		// initial rest length

	//center.push_back((int)(row + 1) * (col) / 2 + col / 4);
	//center.push_back((int)(row + 1) * (col) / 2 + col / 8);
	//center.push_back((int)(row + 1) * (col) / 2 - col / 4);
	//center.push_back((int)(row + 1) * (col) / 2 - col / 8);
	center.push_back((int)(row) * (col) / 2 + col / 4 +  10 * row);

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < columns; j++) {

			initPos.push_back(Vector3f(0.5 * r_init * columns - r_init * j, 0, 0.5 * r_init * rows - r_init * i));
			initVel.push_back(Vector3f(0, 0, 0));

			statePos.push_back(Vector3f(r_init * j, 0, r_init * i));
			stateVel.push_back(Vector3f(0, 0, 0));

			Vector3f initState = Vector3f(0.5 * r_init * columns - r_init * j, 0, 0.5 * r_init * rows - r_init * i);
			this->m_vVecState.push_back(initState);
			this->m_vVecState.push_back(Vector3f(0, 0, 0));

			//energyStored.push_back(1.0f - abs(i - 0.5f*rows) / 20.0 - abs(j - 0.5f*columns) / 20.0);

			energyStored.push_back(0.0f);
			

			if (i < columns) {
				forcedPos.push_back(Vector3f(0, 0, 0));
				forcedVel.push_back(Vector3f(0, -1, 0));
			}
			else {
				forcedPos.push_back(Vector3f(-100, -100, -100));
				forcedVel.push_back(Vector3f(-100, -100, -100));
			}

		}
	}
	
	for (int source = 0; source < center.size(); source++) {
		vector<float > phasePush;
		vector<float > magnitudePush;
		vector<int > spreadCountPush;
		vector<int > sourceCheckPush;
		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < columns; j++) {
				if (i * columns + j == center[source]) {
					magnitudePush.push_back(1.00f);
					phasePush.push_back(0.001f);
				}
				else {
					magnitudePush.push_back(0.0f);
					phasePush.push_back(0.0f);
				}
				spreadCountPush.push_back(baseSpreadSpeed);
				sourceCheckPush.push_back(0);
			}
		}
		phaseStored.push_back(phasePush);
		magnitudeStored.push_back(magnitudePush);
		sourceChecked.push_back(sourceCheckPush);
		spreadCounter.push_back(spreadCountPush);
	}

	const float k_spring_struct = 50;			//spring 
	const float k_spring_shear = 50;			// spring stiffness
	const float k_spring_flex = 50;				// spring stiffness
	const float r_shear = sqrt(2) * r_struct;	// shear spring rest length
	const float r_flex = 2.0f * r_struct;		// flexible spring rest length
	
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < columns; j++) {

			int index_I = index(i, j, rows, columns);
			int index_UL = index(i - 1, j + 1, rows, columns);
			int index_U = index(i, j + 1, rows, columns);
			int index_UR = index(i + 1, j + 1, rows, columns);
			int index_L = index(i - 1, j, rows, columns);
			int index_R = index(i + 1, j, rows, columns);
			int index_B = index(i, j - 1, rows, columns);
			int index_BL = index(i - 1, j - 1, rows, columns);
			int index_BR = index(i + 1, j - 1, rows, columns);

			int index_RR = index(i, j + 2, rows, columns);
			vector<int> toPush;
			toPush.push_back(index_I);
			toPush.push_back(index_UL);
			toPush.push_back(index_U);
			toPush.push_back(index_UR);
			toPush.push_back(index_L);
			toPush.push_back(index_R);
			toPush.push_back(index_BL);
			toPush.push_back(index_B);
			toPush.push_back(index_BR);
			springs.push_back(toPush);
		}
	}

}


void WaveSystemWall::dragMotion() {
	return;
}


void WaveSystemWall::takeTimeStep()
{
	vector<Vector3f> f;
	vector< float > energy = getEnergy();
	const float k_drag = 0.001;			// drag constant

	vector<Vector3f > nState = getState();

	for (int i = 0; i < nState.size() / 2; i++) {
		for (int source = 0; source < center.size(); source++) {
			if (phaseStored[source][i] != 0 && sourceChecked[source][i] == 0) {
				for (int j = 0; j < this->springs.size(); j++) {
					if (this->springs[j][0] == i) {
						if (spreadCounter[source][i] == 0) {
							bool newSource = false;
							bool isSource = false;
							int sequence[9] = { 0, 2, 4, 5, 7, 1, 3, 6, 8 };
							for (int seq = 1; seq < 5; seq++) {
								int k = sequence[seq];
								int thisParticle = springs[j][0];
								int nextParticle = springs[j][k];
								float disToThis = (nState[center[source] * 2] - nState[thisParticle * 2]).abs();
								float magnitudeNew = magnitudeStored[source][center[source]] * exp(-disToThis * waveDecayRate);
								//printf("%.2f ", magnitudeNew);
								if (springs[j][k] == -1){ // && magnitudeNew > magnitudeReflectThreshold) {
									newSource = true;
									springs[j][k] = -2;  // Prevent multiple calculation on the same obstacle and increase calculation speed
								}
								for (int sc = 0; sc < center.size(); sc++) {
									if (center[sc] == i) {
										isSource = true;
										float disToNext = (nState[center[source] * 2] - nState[nextParticle * 2]).abs() + 1;
										magnitudeStored[source][i] = magnitudeStored[source][center[source]] * exp(-disToNext * waveDecayRate);
										phaseStored[source][i] = disToNext;
									}
								}
							}
							if (!newSource || isSource){
								for (int seq = 1; seq < 5; seq++) {
									int k = sequence[seq];
									int thisParticle = springs[j][0];
									int nextParticle = springs[j][k];
									if (nextParticle != -1) {
										if (phaseStored[source][nextParticle] < phaseStored[source][thisParticle]) {
											float disToNext = (nState[center[source] * 2] - nState[nextParticle * 2]).abs();
											magnitudeStored[source][nextParticle] = magnitudeStored[source][center[source]] * exp(-disToNext * waveDecayRate);
											phaseStored[source][nextParticle] = disToNext;
											spreadCounter[source][i] = baseSpreadSpeed;
											sourceChecked[source][i] = 1;
										}
									}
								}
							}
							else {
								if (center.size() < maxCenter) {
									center.push_back(i);
									vector<float > phasePush;
									vector<float > magnitudePush;
									vector<int > spreadCountPush;
									vector<int > sourceCheckPush;
									for (int r = 0; r < rows; r++) {
										for (int c = 0; c < columns; c++) {
											if (r * columns + c == i) {
												magnitudePush.push_back(magnitudeStored[source][i]/4.0f);
												phasePush.push_back(phaseStored[source][i]);
											}
											else {
												magnitudePush.push_back(0.0f);
												phasePush.push_back(0.0f);
											}
											spreadCountPush.push_back(baseSpreadSpeed);
											sourceCheckPush.push_back(0);
										}
									}
									phaseStored.push_back(phasePush);
									magnitudeStored.push_back(magnitudePush);
									sourceChecked.push_back(sourceCheckPush);
									spreadCounter.push_back(spreadCountPush);
								}
							}
						}
						else {
							spreadCounter[source][i] -= 1;
						}
					}
				}
			}
		}

		float totY = 0;
		for (int source = 0; source < center.size(); source++) {
			totY += magnitudeStored[source][i] * sin(500 * sysCounter - phaseStored[source][i]);
		}
		nState[2 * i].y() = totY;// + totY2;

		f.push_back(nState[2 * i]);
		f.push_back(nState[2 * i + 1]);
	}

	sysCounter += timeStep;
	this->setState(f);
}

void WaveSystemWall::draw()
{
	
	vector<Vector3f> state = getState();
	vector< float > energy = getEnergy();

	//if (count_wire == 0 || count_wire == 2 || count_wire == 3 || count_wire == 4) {
		for (int i = 0; i < state.size() / 2; i++) {
			//printf("%.2f", energy[i]);				
			Vector3f pos = state[i * 2];
			GLfloat particleColor[] = { 0.5f, 1.0f * (0.5 * pos[1] / 3 + 0.5), 0.5f, 1.0f };
			 

			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, particleColor);
			glPushMatrix();
			glTranslatef(pos[0], pos[1], pos[2]);
			glutSolidSphere(part_size + 0.2*energy[i], 10.0f, 10.0f);
			glPopMatrix();

		}
	//}

}

int WaveSystemWall::index(int cur_row, int cur_col, int tot_row, int tot_col) {
	if (cur_row >= tot_row || cur_col >= tot_col || cur_row < 0 || cur_col < 0) {
		return -1;
	}
	else {
		return cur_row * tot_col + cur_col;
	}
}


void WaveSystemWall::takeStep(float stepSize)
{
	vector<Vector3f> currentState = this->getState();
	vector<Vector3f> f0 = this->evalF(currentState);
	vector<Vector3f> f0_state;
	
	for (int i = 0; i < f0.size(); i++) {
		f0_state.push_back(currentState[i] + stepSize * f0[i]);
	}

	vector<Vector3f> f1 = this->evalF(f0_state);
	vector<Vector3f> nextState;

	for (int i = 0; i < f0.size(); i++) {
		nextState.push_back(currentState[i] + 0.5 * stepSize * (f0[i] + f1[i]));
	}

	this->setState(nextState);
}