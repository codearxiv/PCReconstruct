//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included.

#include "BoundBox.h"
#include "Cloud.h"
#include "MessageLogger.h"
#include "constants.h"

using Vector3f = Eigen::Vector3f;


constexpr const std::array<GLuint, 24> BoundBox::m_elemGL;

//---------------------------------------------------------

BoundBox::BoundBox(const float minBBox[3], const float maxBBox[3],
				   MessageLogger* msgLogger)
{
	m_msgLogger = msgLogger;
	set(minBBox, maxBBox);
}

//---------------------------------------------------------

BoundBox::BoundBox(const Cloud& cloud, MessageLogger* msgLogger)
{
	m_msgLogger = msgLogger;
	set(cloud);
}
//---------------------------------------------------------


void BoundBox::set(const float minBBox[3], const float maxBBox[3])
{
	for(int i=0; i<3; ++i) {
		m_minBBox[i] = minBBox[i];
		m_maxBBox[i] = maxBBox[i];
	}
	m_vertCount = 8;

}

//---------------------------------------------------------

void BoundBox::set(const Cloud& cloud)
{
	for(int j=0; j<3; ++j) {
		m_minBBox[j] = float_infinity;
		m_maxBBox[j] = -float_infinity;
	}

	size_t numPoints = cloud.pointCount();
	for(size_t i = 0; i < numPoints; ++i){
		Vector3f v = cloud.point(i);
		for(int j=0; j<3; ++j) {
			m_minBBox[j] = std::min(v[j], m_minBBox[j]);
			m_maxBBox[j] = std::max(v[j], m_maxBBox[j]);
		}
	}

	m_vertCount = 8;

}


//---------------------------------------------------------

void BoundBox::pad(float padX, float padY, float padZ)
{
	float padding[3] = {padX, padY, padZ};
	for(int j=0; j<3; ++j) {
		m_minBBox[j] -= padding[j];
		m_maxBBox[j] += padding[j];
	}

}

//---------------------------------------------------------

void BoundBox::rescale(float frac)
{

	for(int j=0; j<3; ++j) {
		float padding = frac*(m_maxBBox[j] - m_minBBox[j]);
		m_minBBox[j] -= padding;
		m_maxBBox[j] += padding;
	}
}


//---------------------------------------------------------

const GLfloat *BoundBox::vertGLData()
{
	for(int i = 0, j = 0; i<m_vertCount; ++i){
		for(int k = 0; k < 3; ++k){
			m_vertGL[j+k] =
					(((i>>k)&1) == 0) ? m_minBBox[k] : m_maxBBox[k];

		}
		m_vertGL[j+3] = 1.0f;
		m_vertGL[j+4] = 0.0f;
		m_vertGL[j+5] = 0.0f;
		j += 6;
	}
	return static_cast<const GLfloat*>(m_vertGL.data());
}
//---------------------------------------------------------

void BoundBox::logMessageBBox() const
{
	if(m_msgLogger != nullptr) {
		m_msgLogger->logMessage(
					QString("Bounding box minimum extent:\n") +
					QString("(") +
					QString::number(m_minBBox[0]) +
					QString(", ") + QString::number(m_minBBox[1]) +
					QString(", ") + QString::number(m_minBBox[2]) +
					QString(")\n") +
					QString("Bounding box maximum extent:\n") +
					QString("(") +
					QString::number(m_maxBBox[0]) +
					QString(", ") + QString::number(m_maxBBox[1]) +
					QString(", ") + QString::number(m_maxBBox[2]) +
					QString(")\n"));
	}
}
//---------------------------------------------------------
