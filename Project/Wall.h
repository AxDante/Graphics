#ifndef WALL_H
#define WALL_H

#include <vector>
#include <vecmath.h>
#include <cstdint>


// helper for uniform distribution
float rand_uniform(float low, float hi);


struct GLProgram;
class Wall
{
public:

	Wall(Vector3f pos, Vector3f scale, Vector3f norm, Vector3f rot);

	Vector3f getWallPos() { return wall_pos; };

	Vector3f getWallScale() { return wall_scale; };

	Vector3f getWallRot() { return wall_rot; }
	
	Vector3f getWallNorm() { return wall_norm;  }


	// void draw(GLProgram& gl, float len);

protected:


	Vector3f wall_pos;
	Vector3f wall_scale;
	Vector3f wall_rot;
	Vector3f wall_norm;

};

#endif
