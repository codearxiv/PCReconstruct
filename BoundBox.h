#ifndef BOUNDBOX_H
#define BOUNDBOX_H

//#include <qopengl.h>
//#include <QVector>
//#include <QVector3D>
//#include <Eigen/Dense>
//#include <array>
#include "pt_to_pt_distsq.h"


class Cloud;

class BoundBox
{
	template<typename T> using vector = std::vector<T>;
	using Index = Eigen::Index;
	using Vector3f = Eigen::Vector3f;

public:
	BoundBox() { m_vertCount = 0; }
	BoundBox(const float minBBox[3], const float maxBBox[3]);
	BoundBox(Cloud cloud);

	void set(const float minBBox[3], const float maxBBox[3]);
	void set(Cloud cloud);

	int vertCount() const { return m_vertCount; }
	float diagonalSize() const {
		return sqrt(pt_to_pt_distsq(m_minBBox, m_maxBBox));
	}

	const GLfloat *vertGLData();
	const GLuint *elemGLData() const
	{ return static_cast<const GLuint*>(m_elemGL.data()); }

	bool pointInBBox(const Vector3f& v) {
		return  v[0] > m_minBBox[0] && v[0] < m_maxBBox[0] &&
				v[1] > m_minBBox[1] && v[1] < m_maxBBox[1] &&
				v[2] > m_minBBox[2] && v[2] < m_maxBBox[2];
	}
private:
	float m_minBBox[3];
	float m_maxBBox[3];
	int m_vertCount;
	std::array<GLfloat, 8*6> m_vertGL;

	constexpr static const std::array<GLuint, 24> m_elemGL = {
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

};

#endif // BOUNDBOX_H
