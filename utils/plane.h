#ifndef PLANE_H
#define PLANE_H


class plane
{
	using Index = Eigen::Index;
	using Matrix3f = Eigen::Matrix3f;
	using Vector3f = Eigen::Vector3f;
	using Vector2f = Eigen::Vector2f;

public:
	plane(Vector3f p0, Vector3f norm) { set(p0,norm); }

	void set(Vector3f p0, Vector3f norm);
	void project(Vector3f p, Vector3f& q, Vector2f& uv);

private:
	Vector3f p0;
	Vector3f norm;
	Vector3f u;
	Vector3f v;

};

#endif // PLANE_H
