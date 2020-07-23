#include "BoundBox.h"
#include "Cloud.h"
#include "constants.h"


using Vector3f = Eigen::Vector3f;


//---------------------------------------------------------

BoundBox::BoundBox(const float minBBox[3], const float maxBBox[3])
{
	set(minBBox, maxBBox);
}

//---------------------------------------------------------

BoundBox::BoundBox(const Cloud& cloud)
{
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

