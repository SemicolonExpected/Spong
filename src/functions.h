#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <string>
#include <vector>
// Linear Algebra Library
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#ifdef _WIN32
#  include <windows.h>
#  undef max
#  undef min
#  undef DrawText
#endif

#ifndef __APPLE__
#  define GLEW_STATIC
#  include <GL/glew.h>
#endif

#ifdef __APPLE__
#   include <OpenGL/gl3.h>
#   define __gl_h_ /* Prevent inclusion of the old gl.h */
#else
#   ifdef _WIN32
#       include <windows.h>
#   endif
#   include <GL/gl.h>
#endif

using namespace Eigen;
using namespace std;

float toRadian(float degrees);
int isInsideObject(MatrixXi objects, MatrixXf vertices, MatrixXi indices,
		   MatrixXf rot, MatrixXf scale, MatrixXf translate,
		   Vector3f w_rot, float w_scale,
		   Vector2f point, int ifPerspective);
int ifBallCollided(Vector4f powerupObj, Vector4f ballObj, 
  MatrixXf vertices, MatrixXi indices, 
  Vector3f rot, float objScale, Vector3f translate, 
  float ballScale, Vector3f ballTranslate);
int isInsideTriangle(MatrixXf vertexList, double x, double y);
Vector3f getTriangleNormal(Vector3f v1, Vector3f v2, Vector3f v3);
MatrixXf getVertexNormals(MatrixXf vertices, MatrixXi indices, MatrixXf faceNormals, int offset = 0);
Vector2d getTriangleBarycenter(double x1, double x2, double x3, double y1, double y2, double y3);
Vector2d rotateAroundPoint(double x, double y, double rotX, double rotY, double angle);
Vector2d scaleWith1Direction(double x, double y, double rotX, double rotY, double scale);
Vector3d getMeshBarycenter(Vector3f vertices[], int numVertices);
MatrixXf centerOnOrigin(MatrixXf vertices, int numvertices, Vector3d centroid);
MatrixXf scaleToUnit(MatrixXf vertices, int numvertices, Vector3d centroid);
Vector3f rotate3D(Vector3f vector, Vector3f origin, Vector3f rot);

/*IBO Related*/
class IndexBufferObject{
 public:
  typedef unsigned int GLuint;
  typedef int GLint;

  GLuint id;

 IndexBufferObject() : id(0) {}

  void init();
  void update(const MatrixXi& M);
  void bind();
  void free();
};
 
#endif
