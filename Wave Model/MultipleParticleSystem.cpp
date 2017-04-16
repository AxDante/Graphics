
#define _USE_MATH_DEFINES

#include <iostream>
#include <cstdio>
#include <ctime>
#include <math.h>
#include "MultipleParticleSystem.h"


using namespace std;

//std::clock_t start = std::clock();
//double duration;
float counter_breeze2;

//TODO: Initialize here
MultipleParticleSystem::MultipleParticleSystem(int rows, int columns, int layers, float mass)
{
	m_mass = mass;
	m_rows = rows;
	m_columns = columns;
	m_layers = layers;
	m_clothNumParticles = rows * columns * layers;

	vector<Vector3f> startPosVel(m_clothNumParticles * 2);

	const float k_spring_struct = 50;			// spring 
	const float k_spring_shear = 50;			// spring stiffness
	const float k_spring_flex = 50;				// spring stiffness
	const float r_struct = 0.2f;				// structural spring rest length
	const float r_shear = sqrt(2) * r_struct;	// shear spring rest length
	const float r_flex = 2.0f * r_struct;		// flexible spring rest length
	const float r_init = 1.3f * r_struct;		// initial rest length

	for (int k = 0; k < layers; k++) {
		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < columns; j++) {
				int index_I = index(i, j, k, rows, columns, layers);
				int index_B = index(i + 1, j, k, rows, columns, layers);
				int index_R = index(i, j + 1, k, rows, columns, layers);
				int index_P = index(i, j, k + 1, rows, columns, layers);
				int index_BB = index(i + 2, j, k, rows, columns, layers);
				int index_RR = index(i, j + 2, k, rows, columns, layers);
				int index_PP = index(i, j, k + 2, rows, columns, layers);
				//int index_

				if (index_B != -1) {
					this->springs.push_back(Vector4f(index_I, index_B, r_struct, k_spring_struct));
				}
				if (index_R != -1) {
					this->springs.push_back(Vector4f(index_I, index_R, r_struct, k_spring_struct));
				}
				if (index_P != -1) {
					this->springs.push_back(Vector4f(index_I, index_P, r_struct, k_spring_struct));
				}
				//if (index_BL != -1) {
				//	this->springs.push_back(Vector4f(index_I, index_BL, r_shear, k_spring_shear));
				//}
				//if (index_BR != -1) {
				//	this->springs.push_back(Vector4f(index_I, index_BR, r_shear, k_spring_shear));
				//}
				if (index_BB != -1) {
					this->springs.push_back(Vector4f(index_I, index_BB, r_flex, k_spring_flex));
				}
				if (index_RR != -1) {
					this->springs.push_back(Vector4f(index_I, index_RR, r_flex, k_spring_flex));
				}
				if (index_PP != -1) {
					this->springs.push_back(Vector4f(index_I, index_PP, r_flex, k_spring_flex));
				}


				if (i < rows - 1 && j < columns - 1 && k < layers - 1) {

					Vector3f vv1 = { (float)(i + 1) * columns + j + 1,
						(float)i * columns + (j + 1),
						(float)(i + 1) * columns + j };
					Vector3f vv2 = { (float)i * columns + j + 1,
						(float)i * columns + j,
						(float)(i + 1) * columns + j };
					Vector3f vv3 = { (float)(i + 1) * columns + j + 1,
						(float)(i + 1) * columns + j,
						(float)i * columns + j + 1 };
					Vector3f vv4 = { (float)i * columns + j + 1,
						(float)(i + 1) * columns + j,
						(float)i * columns + j };

					this->m_face.push_back(vv1);
					this->m_face.push_back(vv2);
					this->m_face.push_back(vv3);
					this->m_face.push_back(vv4);

				}
				this->m_vVecState.push_back(Vector3f(r_init * i, r_init * j, r_init * k));
				this->m_vVecState.push_back(Vector3f(0, 0, 0));
			}
		}
	}
}


void MultipleParticleSystem::dragMotion() {
	return;
}
void MultipleParticleSystem::breeze()
{
	return;
}
// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> MultipleParticleSystem::evalF(vector<Vector3f> state)
{

	vector<Vector3f> f;

	// Variable Definition
	const float G = 9.80665;		// Graviational acceleration
	const float k_drag = 1;			// drag constant


	for (int i = 0; i < state.size() / 2; i++) {

		Vector3f pos = state[2 * i];
		Vector3f vel = state[2 * i + 1];

		//Vector3f F_grav = m_mass * Vector3f(0, -G, 0);
		Vector3f F_grav = 0;
		Vector3f F_vic = -k_drag * vel;
		Vector3f F_spring;
		Vector3f F_breeze;
		Vector3f F_pressure;


		for (int j = 0; j < this->springs.size(); j++) {

			if (this->springs[j][0] == i) {
				Vector3f dis = pos - state[2 * this->springs[j][1]];
				F_spring += -this->springs[j][3] * (dis.abs() - this->springs[j][2]) * (dis / dis.abs());
			}
			if (this->springs[j][1] == i) {
				Vector3f dis = pos - state[2 * this->springs[j][0]];
				F_spring += -this->springs[j][3] * (dis.abs() - this->springs[j][2]) * (dis / dis.abs());
			}
		}


		if (switch_breeze) {

			//float fraction = float(rand()) / RAND_MAX;
			//F_breeze = Vector3f(1 + 0.25 * cos(counter_breeze2 + fraction), 0, -1 + 2 * sin(counter_breeze2));
			//counter_breeze2 += 0.000015 * M_PI;
			/*
			float phi = 0;
			float lambda = 10;
			float omega = 5;
			float k = 2 * M_PI / lambda;
			float A = 0.25;
			int x = i / (m_rows * m_layers);
			vel.x() = vel.x() + 0.01;
			F_pressure = vel*vel*pos;
			*/
			float fraction = float(rand()) / RAND_MAX;
			F_breeze = Vector3f(1 + 0.25 * cos(counter_breeze2 + fraction), 0, -1 + 2 * sin(counter_breeze2));
			counter_breeze2 += 0.000005 * M_PI;

		}
		else {
			F_breeze = Vector3f(0, 0, 0);
		}

		Vector3f totalForce = F_grav + F_vic + F_breeze + F_pressure + F_spring;
		Vector3f Pos_def = vel;
		Vector3f Vel_def = totalForce / m_mass;

		if (i == 0 || i == m_columns - 1) {
			// fixed point
			//Pos_def = Vector3f(0.0f, 0.0f, 0.0f);
			Vel_def = Vector3f(0.0f, 0.0f, 0.0f);
		}

		f.push_back(Pos_def);
		f.push_back(Vel_def);

	}
	return f;
}

///TODO: render the system (ie draw the particles)
void MultipleParticleSystem::draw()
{
	vector<Vector3f> state = getState();
	if (count_wire == 0 || count_wire == 2 || count_wire == 3 || count_wire == 4) {
		for (int i = 0; i < state.size() / 2; i++) {
			Vector3f pos = state[i * 2];//  position of particle i. YOUR CODE HERE
			glPushMatrix();
			glTranslatef(pos[0], pos[1], pos[2]);
			glutSolidSphere(0.075f, 10.0f, 10.0f);
			glPopMatrix();

		}
	}

	if (count_wire == 1 || count_wire == 2 || count_wire == 4 || count_wire == 5) {

		for (unsigned int i = 0; i < springs.size(); i++) {
			Vector3f vv1 = getState()[2 * (int)springs[i][0]];
			Vector3f vv2 = getState()[2 * (int)springs[i][1]];

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

	if (count_wire == 3 || count_wire == 4 || count_wire == 5 || count_wire == 6) {
		// Make sure the wire is drawn at different wire view
		glPushMatrix();
		glTranslatef(state[(m_columns - 1) * 2][0], state[(m_columns - 1) * 2][1], state[(m_columns - 1) * 2][2]);
		glutSolidSphere(0.075f, 10.0f, 10.0f);
		glPopMatrix();

		for (int i = 0; i < this->m_face.size(); i++) {

			Vector3f v1 = getState()[2 * this->m_face[i][0]];
			Vector3f v2 = getState()[2 * this->m_face[i][1]];
			Vector3f v3 = getState()[2 * this->m_face[i][2]];
			Vector3f n = Vector3f::cross(v2 - v1, v3 - v1).normalized();

			glBegin(GL_TRIANGLES);
			glNormal3f(n[0], n[1], n[2]);
			glVertex3f(v1[0], v1[1], v1[2]);
			glVertex3f(v2[0], v2[1], v2[2]);
			glVertex3f(v3[0], v3[1], v3[2]);
			glEnd();

		}
	}
}




int MultipleParticleSystem::index(int cur_row, int cur_col, int cur_lyr, int tot_row, int tot_col, int tot_lyr) {
	if (cur_row >= tot_row || cur_col >= tot_col || cur_lyr >= tot_lyr || cur_row < 0 || cur_col < 0 || cur_lyr < 0) {
		return -1;
	}
	else {
		return cur_lyr * tot_col * tot_row + cur_row * tot_col + cur_col;
	}
}