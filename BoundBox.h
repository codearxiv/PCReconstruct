//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included.

#ifndef BOUNDBOX_H
#define BOUNDBOX_H

#include "pt_to_pt_distsq.h"

#include <qopengl.h>
#include <Eigen/Dense>
#include <array>

class MessageLogger;
class Cloud;

class BoundBox
{
	using Index = Eigen::Index;
	using Vector3f = Eigen::Vector3f;

public:
	BoundBox(MessageLogger* msgLogger = nullptr): m_msgLogger(msgLogger){}
	BoundBox(
			const float minBBox[3], const float maxBBox[3],
			 MessageLogger* msgLogger = nullptr);
	BoundBox(const Cloud& cloud,  MessageLogger* msgLogger = nullptr);

    void set(const float minBBox[3], const float maxBBox[3]);
	void set(const Cloud& cloud);
	void pad(float padX, float padY, float padZ);
	void rescale(float frac);

	int vertCount() const { return m_vertCount; }
	void getExtents(float minBBox[], float maxBBox[]) const {
		for(int i=0; i < 3; ++i){
			minBBox[i] = m_minBBox[i];
			maxBBox[i] = m_maxBBox[i];
		}
	}
	float diagonalSize() const {
		return sqrt(pt_to_pt_distsq(m_minBBox, m_maxBBox));
	}

	const GLfloat *vertGLData();
	const GLuint *elemGLData() const
	{ return static_cast<const GLuint*>(m_elemGL.data()); }

	bool pointInBBox(const Vector3f& p) const {
		return  p(0) >= m_minBBox[0] && p(0) <= m_maxBBox[0] &&
				p(1) >= m_minBBox[1] && p(1) <= m_maxBBox[1] &&
				p(2) >= m_minBBox[2] && p(2) <= m_maxBBox[2];
	}

	float ballInBBox(const Vector3f& p, float radius) const {
		return  p(0) - m_minBBox[0] >= radius &&
				p(1) - m_minBBox[1] >= radius &&
				p(2) - m_minBBox[2] >= radius &&
				m_maxBBox[0] - p(0) >= radius &&
				m_maxBBox[1] - p(1) >= radius &&
				m_maxBBox[2] - p(2) >= radius;
	}

	void logMessageBBox() const;


private:
	float m_minBBox[3] = {0.0f,0.0f,0.0f};
	float m_maxBBox[3] = {0.0f,0.0f,0.0f};
	int m_vertCount = 0;
	std::array<GLfloat, 8*6> m_vertGL;

    static constexpr std::array<GLuint, 24> m_elemGL = {
		0, 1,
		0, 2,
		0, 4,
		1, 3,
		1, 5,
		2, 3,
		2, 6,
		3, 7,
		4, 5,
		4, 6,
		5, 7,
		6, 7
	};

	MessageLogger* m_msgLogger;


};

#endif // BOUNDBOX_H
