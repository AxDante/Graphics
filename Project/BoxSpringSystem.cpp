
#define _USE_MATH_DEFINES

#include <iostream>
#include <cstdio>
#include <ctime>
#include <math.h>
#include "BoxSpringSystem.h"
#include "Wall.h"

//std::clock_t start = std::clock();
//double duration;

//TODO: Initialize here
BoxSpringSystem::BoxSpringSystem(float width, float length, float height, int n_particles, float mass)
{
	box_length = 4;
	box_thickness = 0.2;
	part_size = 0.15;


	float len = box_length;
	float thk = box_thickness;

	walls.push_back(Wall(Vector3f(0, -len, 0), Vector3f(2 * len + 0.5*thk, thk, 2 * len + 0.5*thk), Vector3f(0, 1, 0), Vector3f(0, 0, 0)));
	walls.push_back(Wall(Vector3f(-len, 0, 0), Vector3f(thk, 2 * len + 0.5*thk, 2 * len + 0.5*thk), Vector3f(1, 0, 0), Vector3f(0, 0, 0)));
	walls.push_back(Wall(Vector3f(0,  0, -len), Vector3f(2 * len + 0.5*thk, 2 * len + 0.5*thk, thk), Vector3f(0, 0, 1), Vector3f(0, 0, 0)));
	walls.push_back(Wall(Vector3f(len, 0, 0), Vector3f(thk, 2 * len + 0.5*thk, 2 * len + 0.5*thk), Vector3f(-1, 0, 0), Vector3f(0, 0, 0)));
	walls.push_back(Wall(Vector3f(0, 0, len), Vector3f(2 * len + 0.5*thk, 2 * len + 0.5*thk, thk), Vector3f(0, 0, -1), Vector3f(0, 0, 0)));
	walls.push_back(Wall(Vector3f(0, len, 0), Vector3f(2 * len + 0.5*thk, thk, 2 * len + 0.5*thk), Vector3f(0, -1, 0), Vector3f(0, 0, 0)));
	m_numParticles = n_particles;
	
	part_mass = mass;
	layers = 5;
	columns = 5;
	rows = 5;


	vector<Vector3f> startPosVel(m_numParticles * 2);
	
	const float k_spring_struct = 50;			// spring 
	const float k_spring_shear = 50;			// spring stiffness
	const float k_spring_flex = 50;				// spring stiffness
	const float r_struct = 0.6;				// structural spring rest length
	const float r_shear = sqrt(2) * r_struct;	// shear spring rest length
	const float r_flex = 2.0f * r_struct;		// flexible spring rest length
	const float r_init = 1.0f * r_struct;		// initial rest length

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
				int index_BL = index(i - 1, j - 1, k, rows, columns, layers);
				int index_BR = index(i - 1, j + 1, k, rows, columns, layers);
				int index_BQ = index(i - 1, j, k - 1, rows, columns, layers);
				int index_BP = index(i - 1, j, k + 1, rows, columns, layers);
				int index_AB = index(i, j - 1, k - 1, rows, columns, layers);
				int index_AC = index(i, j + 1, k - 1, rows, columns, layers);

				if (index_B != -1) {
					this->springs.push_back(Vector4f(index_I, index_B, r_struct, k_spring_struct));
				}
				if (index_R != -1) {
					this->springs.push_back(Vector4f(index_I, index_R, r_struct, k_spring_struct));
				}
				if (index_P != -1) {
					this->springs.push_back(Vector4f(index_I, index_P, r_struct, k_spring_struct));
				}
				if (index_BL != -1) {
					this->springs.push_back(Vector4f(index_I, index_BL, r_shear, k_spring_shear));
				}
				if (index_BR != -1) {
					this->springs.push_back(Vector4f(index_I, index_BR, r_shear, k_spring_shear));
				}
				if (index_BP != -1) {
					this->springs.push_back(Vector4f(index_I, index_BP, r_shear, k_spring_shear));
				}
				if (index_BQ != -1) {
					this->springs.push_back(Vector4f(index_I, index_BQ, r_shear, k_spring_shear));
				}
				if (index_AB != -1) {
					this->springs.push_back(Vector4f(index_I, index_AB, r_shear, k_spring_shear));
				}
				if (index_AC != -1) {
					this->springs.push_back(Vector4f(index_I, index_AC, r_shear, k_spring_shear));
				}
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

				float rdm = 0.5*((double)rand() / (RAND_MAX));
				this->m_vVecState.push_back(Vector3f(r_init * i, r_init * j, r_init * k));
				this->m_vVecState.push_back(Vector3f(0, 0, 0));
			}
		}
	}

}


void BoxSpringSystem::dragMotion() {
	return;
}

vector<Vector3f> BoxSpringSystem::evalF(vector<Vector3f> state)
{
	
	vector<Vector3f> f;


	
	// Variable Definition
	const float G = 0.980665;		// Graviational acceleration
	const float k_drag = 0.3;			// drag constant
	const float wall_k = 1.0; // wall spring constant
	const float wall_damping = 1.0111; // wall damping constant

	for (int i = 0; i < state.size() / 2; i++) {

		Vector3f pos = state[2 * i];
		Vector3f vel = state[2 * i + 1];

		

		Vector3f F_grav = part_mass * Vector3f(0, -G, 0);
		Vector3f F_vic = -k_drag * vel;
		Vector3f F_spring;
		Vector3f F_breeze;
		Vector3f F_pressure;
		Vector3f F_wallCollision;

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

	//	Vector3f inputForce = F_grav + F_vic  + F_spring; // + F_breeze + F_pressure

		for (int k = 0; k < walls.size(); k++) {
			Wall w = walls[k];
			double d = Vector3f::dot(pos - w.getWallPos(), w.getWallNorm());
			if (d < part_size + box_thickness) {
				F_wallCollision = wall_damping * Vector3f::dot(vel, w.getWallNorm()) * w.getWallNorm() + wall_k * w.getWallNorm() * d;
				//F_wallCollision += wall_damping * Vector3f::dot(w.getWallNorm(), vel) * w.getWallNorm() + wall_k * w.getWallNorm() * d;
				//vel = vel - 2 * Vector3f::dot(vel, w.getWallNorm()) * w.getWallNorm();
				//this->m_vVecState[2 * i] = pos + (part_size + box_thickness - d) * w.getWallNorm();
			}
		}


		Vector3f totalForce = F_grav + F_vic  + F_wallCollision + F_spring; // + F_breeze + F_pressure
		Vector3f Pos_def = vel;
		Vector3f Vel_def = totalForce / part_mass;

		if (i == 0 ) {
			// fixed point
			Pos_def = Vector3f(0.0f, 0.0f, 0.0f);
			//Vel_def = Vector3f(0.0f, 0.0f, 0.0f);
		}

		f.push_back(Pos_def);
		f.push_back(Vel_def);

	}
	return f;
}

///TODO: render the system (ie draw the particles)
void BoxSpringSystem::draw()
{
	if (count_box == 0) {
		for (int i = 0; i < walls.size() - 1; i++) {
			glColor3f(0, 1, 0);
			glPushMatrix();
			glTranslatef(walls[i].getWallPos().x(), walls[i].getWallPos().y(), walls[i].getWallPos().z());
			glScalef(walls[i].getWallScale().x(), walls[i].getWallScale().y(), walls[i].getWallScale().z());
			glRotatef(0, walls[i].getWallRot().y(), walls[i].getWallRot().x(), walls[i].getWallRot().z());
			glutSolidCube(1);
			glPopMatrix();
		}
	}
	else if (count_box == 1) {
		for (int i = 0; i < walls.size() - 3; i++) {
			glColor3f(0, 1, 0);
			glPushMatrix();
			glTranslatef(walls[i].getWallPos().x(), walls[i].getWallPos().y(), walls[i].getWallPos().z());
			glScalef(walls[i].getWallScale().x(), walls[i].getWallScale().y(), walls[i].getWallScale().z());
			glRotatef(0, walls[i].getWallRot().y(), walls[i].getWallRot().x(), walls[i].getWallRot().z());
			glutSolidCube(1);
			glPopMatrix();
		}
	}

	
	vector<Vector3f> state = getState();
	if (count_wire == 0 || count_wire == 2 || count_wire == 3 || count_wire == 4) {
		for (int i = 0; i < state.size() / 2; i++) {
			Vector3f pos = state[i * 2];
			glPushMatrix();
			glTranslatef(pos[0], pos[1], pos[2]);
			glutSolidSphere(part_size, 10.0f, 10.0f);
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
		glPushMatrix();
		glTranslatef(state[(columns - 1) * 2][0], state[(columns - 1) * 2][1], state[(columns - 1) * 2][2]);
		glutSolidSphere(part_size, 10.0f, 10.0f);
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

int BoxSpringSystem::index(int cur_row, int cur_col, int cur_lyr, int tot_row, int tot_col, int tot_lyr) {
	if (cur_row >= tot_row || cur_col >= tot_col || cur_lyr >= tot_lyr || cur_row < 0 || cur_col < 0 || cur_lyr < 0) {
		return -1;
	}
	else {
		return cur_lyr * tot_col * tot_row + cur_row * tot_col + cur_col;
	}
}