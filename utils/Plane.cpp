#include "plane.h"

void plane::set(Vector3f p0, Vector3f norm)
{

  const double RRT2 = 1.0f/sqrt(2.0f); 
  _p0 = p0;
  _norm = norm;
  
  if( abs(norm(0)) < RRT2 ) {
    _u(0) = 0.0f;  
    _u(1) = norm(2);  
    _u(2) = -norm(1);  
  }
  else{
    _u(0) = -norm(2);  
    _u(1) = 0.0f;  
    _u(2) = norm(0);  
  }
  _v = _u.cross(norm);

  _u.normalize();
  _v.normalize();
  
}

Vector2f plane::project_uv(Vector3f q)
{
  Vector3f w = q - p0;
  puv(0) = w.dot(u);
  puv(1) = w.dot(v);
  return puv;  
}



Vector2f plane::project(Vector3f q, Vector2f& puv)
{
  puv = project_uv(q, puv);
  Vector3f p = puv(0)*_u + puv(1)*_v; 
  return p;
}



