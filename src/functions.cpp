#include "functions.h"
#include "Helpers.h"

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>
#else
// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>
#endif


// Linear Algebra Library
#include <Eigen/Core>
#include <string>
#include <iostream>
#include <fstream>
#include <cstdlib>
#define _USE_MATH_DEFINES
#include <cmath> //for abs
#include <math.h>
#include <vector>


using namespace std;

float toRadian(float degrees){
  float radian = degrees*(M_PI/180);
  return radian;
}
Vector3f rotate3D(Vector3f vector, Vector3f origin, Vector3f rot){
  //translate vector and origin wrt true origin of 0,0,0, rotate, then move back to origin
  Vector3f translated = vector - origin;

  MatrixXf rotX(3,3), rotY(3,3), rotZ(3,3);
  rotX <<
    1, 0,             0,
    0, cos(rot(0)),-sin(rot(0)),
    0, sin(rot(0)), cos(rot(0));
  rotY <<
     cos(rot(1)), 0, sin(rot(1)),
     0,           1, 0,
    -sin(rot(1)), 0, cos(rot(1));
  rotZ <<
    cos(rot(2)),  -sin(rot(2)),0,
    sin(rot(2)),   cos(rot(2)),0,
    0,             0          ,1;
  MatrixXf rotM = rotX*rotY*rotZ;
  translated = (translated.transpose() * rotM).transpose() + origin;
  return translated;
}
int isInsideTriangle(MatrixXf vertexList, double x, double y){
  //returns which triangle if hit, returns 0 if miss
  int triangle = 0;
  for(int i = vertexList.cols()/3; i > 0; i--){
    //get last triangle first. In case of overlap the last added triangle gets changed
    //get indices (i*3)-1, i*3-2, i*3-3
    
    //Area A = [ x1(y2 – y3) + x2(y3 – y1) + x3(y1-y2)]/2
    double x1 = (double)vertexList(0, (i*3)-1), x2 = (double) vertexList(0, (i*3)-2), x3 =  (double)vertexList(0, (i*3)-3);
    double y1 = (double)vertexList(1, (i*3)-1), y2 = (double)vertexList(1, (i*3)-2), y3 = (double)vertexList(1,(i*3)-3);

    double a, b, c; //the barycenter weights
    a = ( ((y2-y3)*(x-x3))+((x3-x2)*(y-y3)) )/( ((y2-y3)*(x1-x3))+((x3-x2)*(y1-y3)) );
    b = ( ((y3-y1)*(x-x3))+((x1-x3)*(y-y3)) )/( ((y2-y3)*(x1-x3))+((x3-x2)*(y1-y3)) );
    c = 1 - a - b;

    if(a > 0 && b > 0 && c > 0){
      cout <<"hit"<<endl;
      triangle = i;
      return (triangle-1);
    }
    else{
      cout<<"miss"<<endl;
    }
  }
  return (triangle-1);
}
int ifBallCollided(Vector4f powerupObj, Vector4f ballObj, 
  MatrixXf vertices, MatrixXi indices, 
  Vector3f rot, float objScale, Vector3f translate, 
  float ballScale, Vector3f ballTranslate){

  MatrixXf rotX(3,3), rotY(3,3), rotZ(3,3);
  rotX <<
      1, 0,             0,
      0, cos(rot(0)),-sin(rot(0)),
      0, sin(rot(0)), cos(rot(0));
  rotY <<
       cos(rot(1)), 0, sin(rot(1)),
       0,           1, 0,
      -sin(rot(1)), 0, cos(rot(1));
  rotZ <<
      cos(rot(2)),  -sin(rot(2)),0,
      sin(rot(2)),   cos(rot(2)),0,
      0,               0            ,1;
  MatrixXf rotM = rotX*rotY*rotZ;

  //for each vertex in cube check if in range of powerup
  //if so check if in any face
  //int j = objects(0,i); j < objects(1,i); j+=3
  //MatrixXf ifVertexChecked = Eigen::MatrixXf::Zero(1, vertices.cols());
  int closestDistance = objScale/2;
  int closestVert = -1; 
  for(int i = ballObj(0); i < ballObj(1); i++){
    Vector3f point(vertices(0,i), vertices(1,i), vertices(2,i));
    if((point - translate).norm() < closestDistance){
      //check to see if it touches any faces
      for(int j = powerupObj(0); j < powerupObj(1); j+=3){
        Vector3f v1 = vertices.col(j);
        Vector3f v2 = vertices.col(j+1);
        Vector3f v3 = vertices.col(j+2);

        v1 = (v1.transpose() * rotM * objScale + translate.transpose());
        v2 = (v2.transpose() * rotM * objScale + translate.transpose());
        v3 = (v3.transpose() * rotM * objScale + translate.transpose());

        float a,b,c;
        a = ( ((v2(1)+v3(1))*(point(0)-v3(0)))+((v3(0)-v2(0))*(point(1)-v3(1))) )/
            ( ((v2(1)+v3(1))*(v1(0)   -v3(0)))+((v3(0)-v2(0))*(v1(1)-v3(1))) );
        b = ( ((v3(1)+v1(1))*(point(0)-v3(0)))+((v1(0)-v3(0))*(point(1)-v3(1))) )/
            ( ((v2(1)+v3(1))*(v1(0)   -v3(0)))+((v3(0)-v2(0))*(v1(1)-v3(1))) );
        c = 1 - a - b;

        //if hit
        if(a > 0 && b > 0 && c > 0){
          closestVert = i;
          closestDistance = (point-translate).norm();
          break;
        }
      }
    }
  }
  return closestVert;
}
int isInsideObject(MatrixXi objects, MatrixXf vertices, MatrixXi indices,
		   MatrixXf rot, MatrixXf scale, MatrixXf translate,
		   Vector3f w_rot, float w_scale,
		   Vector2f point, int ifPerspective){
  //(objs, V, E, etc)
  //returns the obj number
  //need list of objects, list of vertices, list of indices of the elements, object transformation matrices (rotation scale translation, world transformations (rotation, and scale), click point)
  int objNumber = -1; float maxZ = -3;
  //rot *= -1; w_rot *= -1;
  MatrixXf w_rotX(3,3), w_rotY(3,3), w_rotZ(3,3);
  w_rotX <<
    1,0,           0,
    0,cos(w_rot(0)),-sin(w_rot(0)),
    0,sin(w_rot(0)), cos(w_rot(0));
  w_rotY <<
    cos(w_rot(1)),0, sin(w_rot(1)),
    0,1,0,
    -sin(w_rot(1)),0,cos(w_rot(0)) ;
  w_rotZ <<
    cos(w_rot(2)), -sin(w_rot(2)), 0,
    sin(w_rot(2)), cos(w_rot(2)), 0,
    0, 0, 1;

  MatrixXf w_rotM = w_rotX * w_rotY * w_rotZ;
  
  //for each object, get each face
  //transform face based on transformations
  ////ie (pos*scale*rotation+translation)*world_rotation*world_scale
  //check whether is inside triangle; 
  //check if is closest hit triangle, if so update the objNumber and maxZ
  for(int i = 0; i < objects.cols(); i++){
    //make rotation matrices and then multiply them
    MatrixXf rotX(3,3), rotY(3,3), rotZ(3,3);
    rotX <<
      1, 0,             0,
      0, cos(rot(0,i)),-sin(rot(0,i)),
      0, sin(rot(0,i)), cos(rot(0,i));
    rotY <<
       cos(rot(1,i)), 0, sin(rot(1,i)),
       0,             1, 0,
      -sin(rot(1,i)), 0, cos(rot(1,i));
    rotZ <<
      cos(rot(2,i)),  -sin(rot(2,i)),0,
      sin(rot(2,i)),   cos(rot(2,i)),0,
      0,               0            ,1;
    MatrixXf rotM = rotX*rotY*rotZ;
    
    for(int j = objects(0,i); j < objects(1,i); j+=3){
      //cout << vertices.col(indices(0,objects(0,i))) <<endl;
      //Vector3f v1 = vertices.col(indices(0,j));
      //Vector3f v2 = vertices.col(indices(0,j+1));
      //Vector3f v3 = vertices.col(indices(0,j+2));
      Vector3f v1 = vertices.col(j);
      Vector3f v2 = vertices.col(j+1);
      Vector3f v3 = vertices.col(j+2);

      Vector3f translation = translate.col(i);
      	v1 = (v1.transpose() * rotM * scale(0,i) + translation.transpose()) * w_rotM * w_scale;
      	v2 = (v2.transpose() * rotM * scale(0,i) + translation.transpose()) * w_rotM * w_scale;
      	v3 = (v3.transpose() * rotM * scale(0,i) + translation.transpose()) * w_rotM * w_scale;
      if(ifPerspective == 1){ //more complicated
      	v1 = v1*(v1(2) / 0.5 + 1.0);
      	v2 = v2*(v2(2) / 0.5 + 1.0);
      	v3 = v3*(v3(2) / 0.5 + 1.0);
      }
      //apply transformations
      
      //check vertices

      float a,b,c;
      a = ( ((v2(1)+v3(1))*(point(0)-v3(0)))+((v3(0)-v2(0))*(point(1)-v3(1))) )/
	  ( ((v2(1)+v3(1))*(v1(0)   -v3(0)))+((v3(0)-v2(0))*(v1(1)-v3(1))) );
      b = ( ((v3(1)+v1(1))*(point(0)-v3(0)))+((v1(0)-v3(0))*(point(1)-v3(1))) )/
	  ( ((v2(1)+v3(1))*(v1(0)   -v3(0)))+((v3(0)-v2(0))*(v1(1)-v3(1))) );
      c = 1 - a - b;

      //if hit
      if(a > 0 && b > 0 && c > 0){
	//check z value to maxZ if bigger then update Obj num and max Z
	if(v1(2) > maxZ || v2(2) > maxZ || v3(2) > maxZ){
	  if(objNumber != i){objNumber = i;}
	  maxZ = (v1(2) < v2(2)) ? v2(2) : v1(2);
	  maxZ = (maxZ  < v3(2)) ? v3(2) : maxZ;
	}
      }
      //cout <<objNumber<<endl;
      //get closest object
      
    }    
  }
  return objNumber;
}
Vector3f getTriangleNormal(Vector3f v1, Vector3f v2, Vector3f v3){
  Vector3f normal = ((v2-v1).cross(v3-v1)).normalized();
  return normal;
}

MatrixXf getVertexNormals(MatrixXf vertices, MatrixXi indices, MatrixXf faceNormals, int offset){
  //list of vertices, list of faces, list of normals, offset (optional for if faceindices are shifted from vertex value by an offset)
  vector<int> faceMembership[vertices.cols()];
  /*I was thinking adjacency list but this is more computationally effective because we build the list and have the face normals so don't have to recompute it everytime. It's like memoization!*/
  //for every face add the corresponding face index to that vertex's list of corresponding faces
  
  for(int i = 0; i < faceNormals.cols(); i++){ //for every face  
    for(int j = 0; j < 3; j++){ //for every vertex in the face
      faceMembership[indices(0,i*3+j)-offset].push_back(i); 
    }
  } 

  //now for every vertex, get the average of the face normals of all the faces it is a member of
  
  MatrixXf normals(3,vertices.cols()); 
  for(int i = 0; i < vertices.cols(); i++){
    Vector3f sum(0,0,0); 
    for(int j = 0; j<faceMembership[i].size(); j++){
      sum += faceNormals.col(faceMembership[i][j]);
    }
    normals(0,i) = sum(0)/faceMembership[i].size();
    normals(1,i) = sum(1)/faceMembership[i].size();
    normals(2,i) = sum(2)/faceMembership[i].size();
  }

  return normals;
}

Vector2d getTriangleBarycenter(double x1, double x2, double x3, double y1, double y2, double y3){
  Vector2d barycenter;

  barycenter(0) = (x1+x2+x3)/3;
  barycenter(1) = (y1+y2+y3)/3;
  
  return barycenter;
}

Vector2d rotateAroundPoint(double x, double y, double rotX, double rotY, double angle){

  //translate x and y to origin
  x = x-rotX; y = y-rotY;

  Vector2d newPoint;
  
  newPoint(0) = x * cos(angle) - y * sin(angle) + rotX;
  newPoint(1) = x * sin(angle) + y * cos(angle) + rotY;
  
  return newPoint;
  
}

Vector2d scaleWith1Direction(double x, double y, double rotX, double rotY, double scale){
  //as opposed to scale from center in 2 directions or centered in 3d with 3
  x = x-rotX; y = y-rotY;

  Vector2d newPoint;

  newPoint(0) = x*scale + rotX;
  newPoint(1) = y*scale + rotY;
  
  return newPoint;
}

Vector3d getMeshBarycenter(Vector3f vertices[], int numVertices){
  Vector3d center;
  double sumX, sumY, sumZ;
  sumX = sumY = sumZ = 0;

  for(int i = 0; i<numVertices; i++){
    sumX += vertices[i](0);
    sumY += vertices[i](1);
    sumZ += vertices[i](2);
  }
  center(0) = sumX/numVertices; center(1) = sumY/numVertices; center(2) = sumZ/numVertices;
  return center;
}

MatrixXf centerOnOrigin(MatrixXf vertices, int numVertices, Vector3d centroid){
  for(int i = 0; i < numVertices; i++){
    for(int k = 0; k < 3; k++){ vertices(k,i) -= centroid(k); }
  }
  return vertices;
}
MatrixXf scaleToUnit(MatrixXf vertices, int numVertices, Vector3d centroid){
  //get max and then scale everything so that the max = 0.5
  double max = 0; //is centered on origin we have to ahve something bigger than 0 and less than 0
  for(int i = 0; i < numVertices; i++){
    for(int k = 0; k < 3; k++){
      if(abs(vertices(k,i)) > max){ max = abs(vertices(k,i)); }
    }
  }

  double scale = 0.5/max;
  for(int i = 0; i < numVertices; i++){
    for(int k = 0; k < 3; k++){ vertices(k,i) *= scale; }
  }
  return vertices;
}

/*IBO Related Functions*/
void IndexBufferObject::init(){
  glGenBuffers(1,&id);
  check_gl_error();
}

void IndexBufferObject::bind(){
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, id);
  //cout << id <<endl;
  check_gl_error();
}

void IndexBufferObject::free(){
  glDeleteBuffers(1,&id);
  check_gl_error();
}

void IndexBufferObject::update(const Eigen::MatrixXi& M){
  assert(id != 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, id);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int)*M.size(), M.data(), GL_STATIC_DRAW);
  check_gl_error();
}
