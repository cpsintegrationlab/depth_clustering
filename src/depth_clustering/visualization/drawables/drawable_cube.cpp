// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include <GL/glut.h>

#include "utils/radians.h"
#include "visualization/drawables/drawable_cube.h"
#include <iostream>

namespace depth_clustering
{
void
DrawableCube::Draw() const
{
	glPushMatrix();
	glTranslatef(_center.x(), _center.y(), _center.z());
	glRotatef(Radians::FromRadians(rotation_).ToDegrees(), 0, 0, 1);
	glScalef(_scale.x(), _scale.y(), _scale.z());
	glColor3f(_color[0], _color[1], _color[2]);
	glLineWidth(4.0f);
	glBegin(GL_LINE_STRIP);

	// Bottom of Box
	glVertex3f(-0.5, -0.5, -0.5);
	glVertex3f(-0.5, -0.5, 0.5);
	glVertex3f(0.5, -0.5, 0.5);
	glVertex3f(0.5, -0.5, -0.5);
	glVertex3f(-0.5, -0.5, -0.5);

	// Top of Box
	glVertex3f(-0.5, 0.5, -0.5);
	glVertex3f(-0.5, 0.5, 0.5);
	glVertex3f(0.5, 0.5, 0.5);
	glVertex3f(0.5, 0.5, -0.5);
	glVertex3f(-0.5, 0.5, -0.5);

	glEnd();

	glBegin(GL_LINES);
	// For the Sides of the Box

	glVertex3f(-0.5, 0.5, -0.5);
	glVertex3f(-0.5, -0.5, -0.5);

	glVertex3f(-0.5, -0.5, 0.5);
	glVertex3f(-0.5, 0.5, 0.5);

	glVertex3f(0.5, -0.5, 0.5);
	glVertex3f(0.5, 0.5, 0.5);

	glVertex3f(0.5, -0.5, -0.5);
	glVertex3f(0.5, 0.5, -0.5);

	glEnd();
	glPopMatrix();
}
}  // namespace depth_clustering
