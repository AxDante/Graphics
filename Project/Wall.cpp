#include "Wall.h"

#include <iostream>
using namespace std;


Wall::Wall(const Vector3f pos, const Vector3f scale, const Vector3f norm, const Vector3f  rot)
{
	wall_pos = pos;
	wall_rot = rot;
	wall_norm = norm;
	wall_scale = scale;

}