// This example is heavily based on the tutorial at https://open.gl

// OpenGL Helpers to reduce the clutter
#include "Helpers.h"
#include "functions.h"

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
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <SOIL.h>
// Timer
#include <chrono>

//Strings for Modes
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <cstdlib>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

//pi
#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>


using namespace std;
using namespace Eigen;

// VertexBufferObject wrapper
VertexBufferObject VBO; //vertices
VertexBufferObject VBO_C; //colors
VertexBufferObject NormalBuffer; //normals of vertices
VertexBufferObject U;
IndexBufferObject IBO;

VertexBufferObject VBO_Quad;
VertexBufferObject U_Quad;

//Element Buffer
MatrixXi E(1,3);
MatrixXi Objs = Eigen::MatrixXi::Zero(4,1); //objs stores start and stop index of each object in the E matrix and the item type and what kind of shading
Eigen::MatrixXf texType = Eigen::MatrixXf::Zero(1,1);
// Contains the vertex positions
Eigen::MatrixXf V = Eigen::MatrixXf::Zero(3,3); Eigen::MatrixXf V_Quad = Eigen::MatrixXf::Zero(2,6);
// Contains the per-vertex color
MatrixXf C(3,3);
MatrixXf C_Black = Eigen::MatrixXf::Zero(3,3);
MatrixXf C_Orange = Eigen::MatrixXf::Zero(3,3);

// Contains the view transformation
Eigen::Matrix4f view(4,4);
Eigen::MatrixXf N = Eigen::MatrixXf::Zero(3,4);

Eigen::MatrixXf UV = Eigen::MatrixXf::Zero(2,4); Eigen::MatrixXf UV_Quad = Eigen::MatrixXf::Zero(2,6);

//Elements
Eigen::MatrixXi bumpyEl = Eigen::MatrixXi::Zero(1,1);
Eigen::MatrixXi bunnyEl = Eigen::MatrixXi::Zero(1,1);
Eigen::MatrixXi sphereEl = Eigen::MatrixXi::Zero(1,1);
Eigen::MatrixXi cubeEl = Eigen::MatrixXi::Zero(1,36);
Eigen::MatrixXi cuboidEl = Eigen::MatrixXi::Zero(1,36);

Vector3f ORIGIN(0,0,0); double xOrigin, yOrigin;

//int rotation = 0; //can be 0, -10, or 10
string MODE;

//transformation values
MatrixXf centers = Eigen::MatrixXf::Zero(3,1);
MatrixXf rotations = Eigen::MatrixXf::Zero(3,1); //angle for each axis
MatrixXf scale = Eigen::MatrixXf::Zero(1,1);
MatrixXf translations = Eigen::MatrixXf::Zero(3,1);

float powerupScale = 0.45;

Vector3f worldRotation(0,0,0); float worldScale = 1;

int selectedObj = -1; //dont need this

int ifPerspective = 1; //don't need this

int cubeOffset = 0, cuboidOffset = 0, bumpyOffset = 0, bunnyOffset = 0, sphereOffset = 0;
MatrixXf bunnyFaceNormals, bumpyFaceNormals, cubeFaceNormals, cuboidFaceNormals, sphereFaceNormals;

Vector3f ballDirection(0,0,0);

Vector3f lightPos(0,0.3,0.5);

/*****************************************************
					IMPORT OBJECTS
 *****************************************************/
int importBumpyCubeVertex(){
  std::ifstream bumpyCube ("../data/bumpy_cube.off");
  string line;
  bumpyCube >> line; 
  int numCubeVertices;
  int numCubeFaces;

  bumpyCube >> line; 
  numCubeVertices = stoi(line); 
  Vector3f cubeVertices[numCubeVertices];
  MatrixXf cubeVerts(3,numCubeVertices);
  bumpyCube >> line; 
  numCubeFaces = stoi(line);
  bumpyCube >> line; //get rid of the edges
  
  int offset = V.cols();
  V.conservativeResize(3, V.cols()+numCubeFaces*3);
  N.conservativeResize(3, V.cols());
  C.conservativeResize(3, V.cols());
  UV.conservativeResize(2, V.cols());

  //get vertices
  for(int i = 0; i < numCubeVertices; i++){
    for(int k = 0; k < 3; k++){
      bumpyCube >> line;
      cubeVertices[i](k) = stof(line);
      cubeVerts(k,i) = cubeVertices[i](k);
    }
  }

  bumpyEl.resize(1,numCubeFaces*3);
  MatrixXf faceNormals(3,numCubeFaces); 
  for(int i = 0; i < bumpyEl.cols(); i+=3){  
    bumpyCube >> line; //get the first col
    for(int k = 0; k < 3; k++){
      bumpyCube >> line;
      bumpyEl(0,i+k) = stof(line);
    }
    //999
    Vector3f normal = getTriangleNormal(cubeVertices[bumpyEl(0,i)],
					cubeVertices[bumpyEl(0,i+1)], cubeVertices[bumpyEl(0,i+2)]);
    faceNormals.col(i/3) << normal(0), normal(1), normal(2);  
  }
  bumpyFaceNormals = faceNormals;
  //get centroid
  Vector3d center = getMeshBarycenter(cubeVertices, numCubeVertices);
  //translate vertices so that bary is at origin. ie subtract center with every vertex;
  cubeVerts = centerOnOrigin(cubeVerts, numCubeVertices, center);
  //scale so that max is 0.5 and -0.5 and at least 1 vertex is at 0.5 or -0.5
  cubeVerts = scaleToUnit(cubeVerts, numCubeVertices, center);

  MatrixXf normals = getVertexNormals(cubeVerts, bumpyEl, faceNormals, 0);

  for(int i = 0; i < numCubeFaces*3; i++){
    V(0, offset + i) = cubeVerts(0, bumpyEl(0,i)); 
    V(1, offset + i) = cubeVerts(1, bumpyEl(0,i));
    V(2, offset + i) = cubeVerts(2, bumpyEl(0,i));
    N.col(i+offset) = normals.col(bumpyEl(0,i));
    C.col(offset+i) << 0.53,0.38,0.79;
  }
  return offset;
}
int importBunnyVertex(){
  std::ifstream bunny ("../data/bunny.off");
  string line;
  //gets rid of first line
  bunny >> line;

  int numBunnyVertices;
  int numBunnyFaces;
  bunny >> line; numBunnyVertices = stoi(line);
  Vector3f bunnyVertices[numBunnyVertices];
  MatrixXf bunnyVerts(3,numBunnyVertices); //this is better but I realize it too late so we have this matrix and the array
  bunny >> line;
  numBunnyFaces = stoi(line);
  bunny >> line; //get rid of the edges

  int offset = V.cols();
  
  V.conservativeResize(3, V.cols()+numBunnyFaces*3);
  N.conservativeResize(3, V.cols());
  C.conservativeResize(3, V.cols());
  UV.conservativeResize(2, V.cols());
  bunnyEl.resize(1, numBunnyFaces*3);
  
  //get vertices
  for(int i = 0; i < numBunnyVertices; i++){
    for(int k = 0; k < 3; k++){
      bunny >> line;
      bunnyVertices[i](k) = stof(line);
      bunnyVerts(k,i) = bunnyVertices[i](k);
    }
  }
  
  //get faces
  MatrixXf faceNormals(3,bunnyEl.cols()/3);
  for(int i = 0; i < bunnyEl.cols(); i+=3){  
    bunny >> line; //get the first col
    for(int k = 0; k < 3; k++){
      bunny >> line;
      bunnyEl(0,i+k) = stof(line);
    }
    Vector3f normal = getTriangleNormal(bunnyVertices[bunnyEl(0,i)], bunnyVertices[bunnyEl(0,i+1)], bunnyVertices[bunnyEl(0,i+2)]);
    faceNormals.col(i/3) << normal(0), normal(1), normal(2);
  }
  bunnyFaceNormals = faceNormals;
  //get centroid
  Vector3d center = getMeshBarycenter(bunnyVertices, numBunnyVertices);
  //translate bary to origin
  bunnyVerts = centerOnOrigin(bunnyVerts, numBunnyVertices, center);
  //scale so that max is 0.5 and -0.5 and at least 1 vertex is at 0.5 or -0.5
  bunnyVerts = scaleToUnit(bunnyVerts, numBunnyVertices, center);

  //get normals
  MatrixXf normals = getVertexNormals(bunnyVerts, bunnyEl, faceNormals, 0);
  //upload into N
  //upload vertices to V
  
  for(int i = 0; i < numBunnyFaces*3; i++){ 
    V(0,offset+i) = bunnyVerts(0, bunnyEl(0,i));  //x
    V(1,offset+i) = bunnyVerts(1, bunnyEl(0,i));  //y
    V(2,offset+i) = bunnyVerts(2, bunnyEl(0,i));  //z
    N.col(i+offset) = normals.col(bunnyEl(0,i)); 
    C.col(offset+i) << 0.9686,0.7916,0.7882;
  }
  
  return offset;
}
int importCubeVertex(){
  int offset = V.cols();

  V.conservativeResize(3, V.cols()+36);
  N.conservativeResize(3, V.cols());
  C.conservativeResize(3, V.cols());
  UV.conservativeResize(2, V.cols());

  MatrixXf colors(3,8);
  MatrixXf cubeV(3,8); 

  cubeV.col(0) << -0.5, -0.5, -0.5;  colors.col(0) << 0,1,1;
  cubeV.col(1) << -0.5, -0.5,  0.5;  colors.col(1) << 1,0,1;
  cubeV.col(2) << -0.5,  0.5, -0.5;  colors.col(2) << 1,1,0;
  cubeV.col(3) << -0.5,  0.5,  0.5;  colors.col(3) << 1,0.6,1;
  cubeV.col(4) <<  0.5, -0.5, -0.5;  colors.col(4) << 0.9686,0.7916,0.7882; 
  cubeV.col(5) <<  0.5, -0.5,  0.5;  colors.col(5) << 0.568627,0.6588,0.8196;
  cubeV.col(6) <<  0.5,  0.5, -0.5;  colors.col(6) << 0,0,0;
  cubeV.col(7) <<  0.5,  0.5,  0.5;  colors.col(7) << 1,1,1;

  cubeEl << 
    0,1,2, 0,1,4, 0,2,4, 1,2,3, 1,5,4, 3,6,7,
    2,3,6, 0,2,4, 4,5,6, 5,6,7, 1,3,5, 3,5,7;

  MatrixXf faceNormals(3,12);
  for(int i = 0; i < cubeEl.cols(); i+=3){
    Vector3f normal = getTriangleNormal(cubeV.col(cubeEl(0,i)), cubeV.col(cubeEl(0,i+1)), cubeV.col(cubeEl(0,i+2)));   
    faceNormals.col(i/3) << normal(0), normal(1), normal(2); 
  }
  cubeFaceNormals = faceNormals;
  //cout << faceNormals;
  MatrixXf normals = getVertexNormals(cubeV, cubeEl, faceNormals, 0);
  //cout << normals <<endl;

  Vector3f magenta(1, 0/*float(106)/255*/, float(213)/255);
  for(int i = 0; i < cubeEl.cols(); i++){ 
    V.col(offset + i) = cubeV.col(cubeEl(0,i));  
    N.col(offset + i) = -normals.col(cubeEl(0,i));
    C.col(offset + i) = colors.col(cubeEl(0,i));
    //C.col(offset + i) = magenta.transpose();
  }

  return offset;
}

int importCuboidVertex(){
  int offset = V.cols();

  V.conservativeResize(3, V.cols()+36);
  //V.conservativeResize(3, V.cols()+24);
  N.conservativeResize(3, V.cols());
  C.conservativeResize(3, V.cols());
  UV.conservativeResize(2, V.cols());

  MatrixXf colors(3,8);
  MatrixXf cubeV(3,8); 
  MatrixXf uvs(2,8);
  
  cubeV.col(0) << -1, -0.4, -1.5;  uvs.col(0) << 0,0;
  cubeV.col(1) << -1, -0.4,  0.5;  uvs.col(1) << 1,0;
  cubeV.col(2) << -1,    1, -1.5;  uvs.col(2) << 0,1;
  cubeV.col(3) << -1,    1,  0.5;  uvs.col(3) << 1,1;
  cubeV.col(4) <<  1, -0.4, -1.5;  uvs.col(4) << 1,1; 
  cubeV.col(5) <<  1, -0.4,  0.5;  uvs.col(5) << 0,1;
  cubeV.col(6) <<  1,    1, -1.5;  uvs.col(6) << 1,0;
  cubeV.col(7) <<  1,    1,  0.5;  uvs.col(7) << 1,1;

  cuboidEl<< 
    0,1,2, 0,1,4, 0,2,4, 1,2,3, 1,5,4, 3,6,7,
    2,3,6, 2,4,6, 4,5,6, 5,6,7, 1,3,5, 3,5,7;

  UV.col(offset) << 0,0;       UV.col(offset + 1) << 1,0;     UV.col(offset + 2) << 0,1; //good 012
  UV.col(offset + 3) << 0,0;   UV.col(offset + 4) << 1,0;     UV.col(offset + 5) << 1,1; //kinda ugly but good  014
  UV.col(offset + 6) << 0,0;   UV.col(offset + 7) << 0,1;     UV.col(offset + 8) << 1,1;     //024
  UV.col(offset + 9) << 1,0;   UV.col(offset + 10) << 0,1;    UV.col(offset + 11) << 1,1; //good //123
  UV.col(offset + 12) << 1,0;  UV.col(offset + 13) << 0,1;    UV.col(offset + 14) << 1,1; //kinda ugly but good 154
  UV.col(offset + 15) << 0,0;  UV.col(offset + 16) << 1,0;    UV.col(offset + 17) << 0,1; //367
  UV.col(offset + 18) << 0,1;  UV.col(offset + 19) << 0,0;    UV.col(offset + 20) << 1,0; //236
  UV.col(offset + 21) << 0,1;  UV.col(offset + 22) << 1,1;    UV.col(offset + 23) << 1,0; //246
  UV.col(offset + 24) << 1,1;  UV.col(offset + 25) << 0,0;    UV.col(offset + 26) << 1,0; //456
  UV.col(offset + 27) << 0,0;  UV.col(offset + 28) << 1,0;    UV.col(offset + 29) << 0,1; //567
  UV.col(offset + 30) << 0,1;  UV.col(offset + 31) << 1,1;    UV.col(offset + 32) << 0,0; //135
  UV.col(offset + 33) << 1,1;  UV.col(offset + 34) << 0,0;    UV.col(offset + 35) << 1,0; //357

  MatrixXf faceNormals(3,12);
  for(int i = 0; i < cuboidEl.cols(); i+=3){
    Vector3f normal = getTriangleNormal(cubeV.col(cuboidEl(0,i)), cubeV.col(cuboidEl(0,i+1)), cubeV.col(cuboidEl(0,i+2)));   
    faceNormals.col(i/3) << normal(0), normal(1), normal(2); 
  }
  cuboidFaceNormals = faceNormals;
  //cout << faceNormals;
  MatrixXf normals = getVertexNormals(cubeV, cuboidEl, faceNormals, 0);
  //cout << normals <<endl;

  Vector3f magenta(1, 0/*float(106)/255*/, float(213)/255);
  for(int i = 0; i < cuboidEl.cols(); i++){ 
    V.col(offset + i) = cubeV.col(cuboidEl(0,i));  
    N.col(offset + i) = -normals.col(cuboidEl(0,i));
    //UV.col(offset+i) =  uvs.col(cuboidEl(0,i));
    //C.col(offset + i) = colors.col(cuboidEl(0,i));
    C.col(offset + i) = magenta.transpose();
  }

  return offset;
}
int importSphere(){
	std::ifstream sphere ("../data/sphere.off");
	string line;

	sphere >> line;
	int numSphereVertices, numSphereFaces;
	sphere >> line; numSphereVertices = stoi(line);
	Vector3f sphereVertices[numSphereVertices];
	MatrixXf sphereVerts(3, numSphereVertices);
	sphere >> line; numSphereFaces = stoi(line); sphere >> line;

	int offset = V.cols();

	V.conservativeResize(3, V.cols()+numSphereFaces*3);
	N.conservativeResize(3, V.cols());
	C.conservativeResize(3, V.cols());
	UV.conservativeResize(2, V.cols());
	sphereEl.resize(1, numSphereFaces*3);

	//get vertices
	for(int i = 0; i < numSphereVertices; i++){
    	for(int k = 0; k < 3; k++){
      		sphere >> line;
      		sphereVerts(k,i) = stof(line);
      		sphereVertices[i](k) = sphereVerts(k,i);
    	}
  	}
  	//get faces
  	MatrixXf faceNormals(3,sphereEl.cols()/3);
	for(int i = 0; i < sphereEl.cols(); i+=3){  
    	sphere >> line; //get the first col
    	for(int k = 0; k < 3; k++){
      		sphere >> line;
      		sphereEl(0,i+k) = stof(line);
    	}
    	Vector3f normal = getTriangleNormal(sphereVertices[sphereEl(0,i)], sphereVertices[sphereEl(0,i+1)], sphereVertices[sphereEl(0,i+2)]);
    	faceNormals.col(i/3) << normal(0), normal(1), normal(2);
	}
	sphereFaceNormals = faceNormals;

	//get centroid
	Vector3d center = getMeshBarycenter(sphereVertices, numSphereVertices);
	//translate bary to origin
	sphereVerts = centerOnOrigin(sphereVerts, numSphereVertices, center);
 	//scale so that max is 0.5 and -0.5 and at least 1 vertex is at 0.5 or -0.5
	sphereVerts = scaleToUnit(sphereVerts, numSphereVertices, center);

  	//get normals
  	MatrixXf normals = getVertexNormals(sphereVerts, sphereEl, faceNormals, 0);
  	//upload into N
  	//upload vertices to V
  
	for(int i = 0; i < numSphereFaces*3; i++){ 
    	V(0,offset+i) = sphereVerts(0, sphereEl(0,i));  //x
    	V(1,offset+i) = sphereVerts(1, sphereEl(0,i));  //y
    	V(2,offset+i) = sphereVerts(2, sphereEl(0,i));  //z
    	N.col(i+offset) = normals.col(sphereEl(0,i)); 
    	C.col(offset+i) << 1.0,0.8274,0.098;
    	if(i%3 == 0){UV.col(offset+i) << 0.,0.;}
    	else if(i%3 == 1){UV.col(offset+i) << 0.,1.;}
    	else{UV.col(offset+i) << 0.5, sqrt(3)/2;}
	}
  
  return offset;
}
/*****************************************************
					  ADD OBJECTS
 *****************************************************/
//the add functions just add another of that object into the element buffer
//we can streamline this into just 1 function by having offset and element variables and depending on what item we are adding
//we would just make offset = objOffset elements = objectEl so we dont need a function for each item
void addBumpyCube(){
  float temp = E.cols(); 
  Objs.conservativeResize(4, Objs.cols()+1); //cout << bumpyOffset + bumpyEl.cols()<<endl;
  texType.conservativeResize(1, Objs.cols()); texType(0,texType.cols()-1) = 1;
  //Objs(0, Objs.cols()-1) = temp; //start index
  Objs(0, Objs.cols()-1) = bumpyOffset; 
  //Objs(1, Objs.cols()-1) = temp + bumpyEl.cols();
  Objs(1, Objs.cols()-1) = bumpyOffset + bumpyEl.cols();//end index
  Objs(2, Objs.cols()-1) = bumpyEl.cols(); //# of indices
  Objs(3, Objs.cols()-1) = 3; //type of shading

  scale.conservativeResize(1, Objs.cols());        scale.col(Objs.cols()-1) << 1;
  rotations.conservativeResize(3, Objs.cols());    rotations.col(Objs.cols()-1) << 0,0,0;
  translations.conservativeResize(3, Objs.cols());  translations.col(Objs.cols()-1) << 0,0,0;

  E.conservativeResize(1, E.cols() + bumpyEl.cols());
		       
  for(int i = 0; i < bumpyEl.cols(); i++){
    E.col(temp + i) = bumpyEl.col(i);
  }
  //cout << "Number of Indices:" << E.cols() << " number of bumpy faces: "<<bumpyEl.cols()<<endl;
}
void addBunny(){
  float temp = E.cols();
  Objs.conservativeResize(4, Objs.cols()+1); //begin, end
  texType.conservativeResize(1, Objs.cols()); texType(0,texType.cols()-1) = 0;
  //Objs(0, Objs.cols()-1) = temp;
  //Objs(1, Objs.cols()-1) = temp + bunnyEl.cols();
  Objs(0, Objs.cols()-1) = bunnyOffset;
  Objs(1, Objs.cols()-1) = bunnyOffset + bunnyEl.cols();
  Objs(2, Objs.cols()-1) = bunnyEl.cols();
  Objs(3, Objs.cols()-1) = 3;

  centers.conservativeResize(3, Objs.cols());      centers.col(Objs.cols()-1) << 0,0,0;
  scale.conservativeResize(1, Objs.cols());        scale.col(Objs.cols()-1) << 1;
  rotations.conservativeResize(3, Objs.cols());    rotations.col(Objs.cols()-1) << 0,0,0;
  translations.conservativeResize(3, Objs.cols());  translations.col(Objs.cols()-1) << 0,0,0;

  E.conservativeResize(1, E.cols()+bunnyEl.cols());

  for(int i = 0; i < bunnyEl.cols(); i++){
    E.col(temp + i) = bunnyEl.col(i);
  }
  //cout << "Number of Indices:" << E.cols() << " number of bunny faces: "<<bunnyEl.cols()<<endl;
}
void addCube(){
  
  int temp = E.cols();
  Objs.conservativeResize(4, Objs.cols()+1);
  texType.conservativeResize(1, Objs.cols()); texType(0,texType.cols()-1) = 0;
  //Objs(0, Objs.cols()-1) = temp; Objs(1, Objs.cols()-1) = temp + 36;
  Objs(0, Objs.cols()-1) = cubeOffset; Objs(1, Objs.cols()-1) = cubeOffset + 36;
  Objs(2, Objs.cols()-1) = 36; Objs(3, Objs.cols()-1) = 1;
  centers.conservativeResize(3, Objs.cols());      centers.col(Objs.cols()-1) << 0,0,0;
  scale.conservativeResize(1, Objs.cols());        scale.col(Objs.cols()-1) << 1;
  rotations.conservativeResize(3, Objs.cols());    rotations.col(Objs.cols()-1) << 0,0,0;
  translations.conservativeResize(3, Objs.cols());  translations.col(Objs.cols()-1) << 0,0,0;

  E.conservativeResize(1, E.cols()+36);
  for(int i = 0; i < 36; i++){
    E.col(temp + i) = cubeEl.col(i);
    int index = E(0,(temp+i));
    //cout << "Index:" << E.col(temp+i) << " Vertex 1: "<< V.col(index) <<endl;
  }
  //cout << endl;
  //cout << "Number of Indices:" << E.cols() <<endl;
  //cout << "This is after I add the cube" << E <<endl;
}
void addCuboid(){
  
  int temp = E.cols();
  Objs.conservativeResize(4, Objs.cols()+1);
  texType.conservativeResize(1, Objs.cols()); texType(0,texType.cols()-1) = 1;
  //Objs(0, Objs.cols()-1) = temp; Objs(1, Objs.cols()-1) = temp + 36;
  Objs(0, Objs.cols()-1) = cuboidOffset; Objs(1, Objs.cols()-1) = cuboidOffset + cuboidEl.cols();
  Objs(2, Objs.cols()-1) = cuboidEl.cols(); Objs(3, Objs.cols()-1) = 1;
  centers.conservativeResize(3, Objs.cols());      centers.col(Objs.cols()-1) << 0,0,0;
  scale.conservativeResize(1, Objs.cols());        scale.col(Objs.cols()-1) << 1;
  rotations.conservativeResize(3, Objs.cols());    rotations.col(Objs.cols()-1) << 0,0,0;
  translations.conservativeResize(3, Objs.cols());  translations.col(Objs.cols()-1) << 0,0,0;

  E.conservativeResize(1, E.cols()+cuboidEl.cols());
  for(int i = 0; i < cuboidEl.cols(); i++){
    E.col(temp + i) = cuboidEl.col(i);
    int index = E(0,(temp+i));
    //cout << "Index:" << E.col(temp+i) << " Vertex 1: "<< V.col(index) <<endl;
  }
  //cout << endl;
  //cout << "Number of Indices:" << E.cols() <<endl;
  //cout << "This is after I add the cube" << E <<endl;
}
void addSphere(){
	float temp = E.cols();
	Objs.conservativeResize(4, Objs.cols()+1);
	texType.conservativeResize(1, Objs.cols()); texType(0,texType.cols()-1) = 3; //maybe 3
	Objs(0, Objs.cols()-1) = sphereOffset;
	Objs(1, Objs.cols()-1) = sphereOffset + sphereEl.cols();
	Objs(2, Objs.cols()-1) = sphereEl.cols();
	Objs(3, Objs.cols()-1) = 2;

	centers.conservativeResize(3, Objs.cols());      centers.col(Objs.cols()-1) << 0,0,0;
	scale.conservativeResize(1, Objs.cols());        scale.col(Objs.cols()-1) << 1;
	rotations.conservativeResize(3, Objs.cols());    rotations.col(Objs.cols()-1) << 0,0,0;
	translations.conservativeResize(3, Objs.cols());  translations.col(Objs.cols()-1) << 0,0,0;

	E.conservativeResize(1, E.cols()+sphereEl.cols());

	for(int i = 0; i < sphereEl.cols(); i++){
    	E.col(temp + i) = sphereEl.col(i);
	}
}

void deleteObj(int objectNum){
  //delete from E, and all the transformation matrices columns starting from O
  
  for(int i = Objs(0,objectNum); i+Objs(2,objectNum) < E.cols(); i++){
    //E, rotations, scale, translations
    E.col(i) = E.col(i+Objs(2,+objectNum));
  }
  E.conservativeResize(1,E.cols()-Objs(2,objectNum));

  int numObjects = Objs.cols();
  for(int i = objectNum; i < numObjects-1; i++){
    Objs.col(i) = Objs.col(i+1);
    rotations.col(i) = rotations.col(i+1);
    scale.col(i) = scale.col(i+1);
    translations.col(i) = translations.col(i+1);
    texType.col(i) = texType.col(i+1);
  }
  Objs.conservativeResize(4, numObjects-1);
  rotations.conservativeResize(3, numObjects-1);
  scale.conservativeResize(1, numObjects-1);
  translations.conservativeResize(3, numObjects-1);
  texType.conservativeResize(1, numObjects-1);
  //delete from Objs matrix.
}

void sortFacesColors(MatrixXf &faces, MatrixXf &centers, MatrixXf &colors, int start, int end){
  //quicksort
  if(start < end){
    
    float pivot = centers(2,end); //last element so can just swap backward
    
    int lowInd = start;
    for(int i = start; i < end+1; i++){
      
      if(centers(2,i) > pivot){
	//swap arr[low] and arr[i] for all arr
	//increment low
	if(i!= lowInd){
	  
	  int tempColors = colors(0,lowInd);
	  colors(0,lowInd) = colors(0,i);
	  colors(0,i) = tempColors;
	
	  Vector3f tempCenters(centers(0,lowInd),centers(1,lowInd),centers(2,lowInd));
	  centers.col(lowInd) = centers.col(i);
	  centers.col(i) = tempCenters;

	  VectorXf tempFace(9); tempFace << faces.col(lowInd);
	  faces.col(lowInd) = faces.col(i);
	  faces.col(i) = tempFace;
	}
	lowInd++;
      }
    }
    int tempColors = colors(0,lowInd);
    colors(0,lowInd) = colors(0,end);
    colors(0,end) = tempColors;
    Vector3f tempCenters(centers(0,lowInd),centers(1,lowInd),centers(2,lowInd));
    centers.col(lowInd) = centers.col(end);
    centers.col(end) = tempCenters;
    VectorXf tempFace(9); tempFace << faces.col(lowInd);
    faces.col(lowInd) = faces.col(end);
    faces.col(end) = tempFace;
    
    sortFacesColors(faces, centers, colors, start, lowInd-1);
    sortFacesColors(faces, centers, colors, lowInd+1, end); 
  }
}

void exportToSVG(std::string filename, int w_width, int w_height){
  //for every object get the translated vertex with a corresponding index number
  //for every 3 indices sort by barry and render from back to front
  ofstream svgfile(filename);
  if(svgfile.is_open()){
    //begin the svg file
    svgfile << "<svg height=\"" << w_height <<"\" width=\"" << w_width << "\">" << endl;
    
    MatrixXf rot = rotations;
    //rot(0) -= toRadian(5); rot(1) -= toRadian(5);  rot(1) -= toRadian(5);
    rot *= -1;
    Vector3f w_rot = worldRotation;
    w_rot *= -1;
    w_rot(2) *= -1;
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

    MatrixXf V_1(9,1); MatrixXf centers(3,1); MatrixXf ind(1,1);
    V_1(0,0) = -100;
    
    for(int i = 0; i < Objs.cols(); i++){
      rot(2,i) *= -1;
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
      for(int j = Objs(0,i); j < Objs(1,i); j+=3){
	//if(ifRendered(i,j/3) == 0){
	Vector3f v1 = V.col(j);
	Vector3f v2 = V.col(j+1);
	Vector3f v3 = V.col(j+2);
	  
	Vector3f translation = translations.col(i);
	v1 = (v1.transpose() * rotM * scale(0,i) + translation.transpose()) * w_rotM * worldScale;
	v2 = (v2.transpose() * rotM * scale(0,i) + translation.transpose()) * w_rotM * worldScale;
	v3 = (v3.transpose() * rotM * scale(0,i) + translation.transpose()) * w_rotM * worldScale;
	
	//if world rotation around y axis is between 270 and 90 so 1/2pi to 1.5pi
	//so > 1.5pi or less than .5pi
	if(w_rot(1) >= -4/*(-1.5*M_PI)*/ && w_rot(1) <= -2.1/*(-0.5*M_PI)*/){
	  v1(2) *= -1; v2(2)*=-1; v3(2)*=-1;
	}

	if(ifPerspective == 1){ //more complicated
	  v1 = v1*(v1(2) * 0.5 + 1.0);
	  v2 = v2*(v2(2) * 0.5 + 1.0);
	  v3 = v3*(v3(2) * 0.5 + 1.0);
	}

	Vector3f center((v1(0)+v2(0)+v3(0))/3,
		      (v1(1)+v2(1)+v3(1))/3,
			(v1(2)+v2(2)+v3(2))/3);

	if(V_1(0,0) == -100){
	  V_1.col(0) << v1(0),v1(1),v1(2), v2(0),v2(1),v2(2), v3(0),v3(1),v3(2);
	  centers.col(0) << center(0), center(1), center(2);
	  ind(0,0) = j;
	}
	else{
	  V_1.conservativeResize(9, V_1.cols()+1);
	  V_1.col(V_1.cols()-1) << v1(0),v1(1),v1(2), v2(0),v2(1),v2(2), v3(0),v3(1),v3(2);
	  centers.conservativeResize(3,V_1.cols());
	  centers.col(V_1.cols()-1) << center(0), center(1), center(2);
	  ind.conservativeResize(1, V_1.cols());
	  ind(0, V_1.cols()-1) = j;
	}
      }
    }
    //sort all indices from back to front
    //cout << ind <<endl;
    sortFacesColors(V_1, centers, ind,  0, V_1.cols()-1); 
    //for(int i = 0; i < ind.cols(); i++){cout << centers(2,i) <<endl;}
    
    for(int i = 0; i < V_1.cols(); i++){
      Vector4f v1_1(V_1(0,i),V_1(1,i),V_1(2,i),1); v1_1 = v1_1.transpose()*view;
      Vector4f v2_1(V_1(3,i),V_1(4,i),V_1(5,i),1); v2_1 = v2_1.transpose()*view;
      Vector4f v3_1(V_1(6,i),V_1(7,i),V_1(8,i),1); v3_1 = v3_1.transpose()*view;
      
      Vector3f v1(v1_1(0),v1_1(1),v1_1(2)); Vector3f v2(v2_1(0),v2_1(1),v2_1(2)); Vector3f v3(v3_1(0),v3_1(1),v3_1(2));
      Vector3f normal = getTriangleNormal(v1,v2,v3).normalized()/*.cwiseAbs()*/; //normal vector
      Vector3f center((v1(0)+v2(0)+v3(0))/3,
		      (v1(1)+v2(1)+v3(1))/3,
		      (v1(2)+v2(2)+v3(2))/3);

      float normalAngle = normal.dot(v1);
      //if(normalAngle < 0){normal = normal.cwiseAbs();}

      Vector2f point1(v1_1(0),v1_1(1)); Vector2f point2(v2_1(0),v2_1(1)); Vector2f point3(v3_1(0),v3_1(1));
      Vector2f center1(center(0),center(1));

      Vector3f lightP = lightPos.transpose() * w_rotM * worldScale;
      //cout << lightP <<endl;
	    
      Vector3f eye = (-center).normalized();
      float dcolor = ((lightP - v1).normalized().transpose().dot(normal));
      if(normalAngle < 0){dcolor *= -1;}
      if(dcolor < 0){dcolor = 0;}
      
      //cout <<ind(0,i)<<endl;
      Vector3f color = C.col(ind(0,i))*(1+dcolor)*255; 
      if(color(0) > 255){color(0) = 255;}if(color(1) > 255){color(1) = 255;}if(color(2) > 255){color(2) = 255;}
      
      //do the aspect ratio thing before drawing by multiplying by view
      point1(0) = (point1(0)+1)*(w_width/2); point3(0) = (point3(0)+1)*(w_width/2); point2(0) = (point2(0)+1)*(w_width/2); 
      point1(1) = (point1(1)*-1+1)*(w_height/2); point3(1) = (point3(1)*-1+1)*(w_height/2); point2(1) = (point2(1)*-1+1)*(w_height/2);
      svgfile << "<polygon points=\""<<point1(0)<<","<<point1(1)<<" "<<point2(0)<<","<<point2(1)<<" "
	      <<point3(0)<<","<<point3(1)<<"\" fill = \"rgba("
	      <<color(0)<<","<<color(1)<<","<<color(2)<<",1)\" />" <<endl;
      /*"{"
                        "vec3 norm = normalize(f_normal);"
                        "vec3 eye = normalize(-f_position);"
                        "vec3 lightdir = normalize(f_lightPos - f_position);"
                        "vec3 reflectdir = reflect(-lightdir,norm);"
                        "float diffuse = clamp(dot(norm,lightdir),0,1);" //cosTheta
                        "vec3 specularStrength = vec3(1,1,1);"
                        "vec3 specFactor = vec3(1,1,1);"
                        "if(f_ifSpecular == 1){"
                            "specFactor = pow(max(dot(eye,reflectdir),0.0),32)*specularStrength;"
                            "outColor = vec4(f_color*(1+diffuse+specFactor), 1.0);" //1 is the ambient light str"
                        "}"
                        "else{"
                        "   outColor = vec4(f_color*(1+diffuse), 1.0);" //1 is the ambient light str
                        "}"
                    "}*/
    }
    //end the svg file
    svgfile << "</svg>";
    svgfile.close();

  }
  else{
    cout << "File Not Created" <<endl;
  }
}

int ifSleep = 0;
void setUpBall(int server){

	translations(0,4) = (server == 2) ? -0.815 : 0.815;
    ballDirection(1) = 0; ballDirection(2) = 0;
    ballDirection(0)= (translations(0,4) < 0) ? 0.005 : -0.005;
    Vector3f angles; 
    angles(0) = (rand()%2 == 0) ? toRadian(45) : 0; //angles(0) = (rand()%2 == 0) ? angles(0) *= -1: angles(0);
    angles(1) = (rand()%2 == 0) ? toRadian(45) : 0; //angles(1) = (rand()%2 == 0) ? angles(1) *= -1: angles(1);
    angles(2) = (rand()%2 == 0) ? toRadian(45) : 0; //angles(2) = (rand()%2 == 0) ? angles(2) *= -1: angles(2);
    ballDirection = rotate3D(ballDirection, ORIGIN, angles); 
    ifSleep = 100;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height){
	//if attempt to resize frame change it back to original size 800x800
	//glutReshapeWindow(800,800);
  //glfwGetWindowSize(window, &width, &height);
  //view(0,0) = float(height)/float(width);
  
  //glViewport(0, 0, width, height);
}

string level, ifSinglePlayer; 
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods){

	//Mouse is only used for picking "level" and 1 Player vs 2 Player

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS){
    	// Get the position of the mouse in the window
	    double xpos, ypos;
	    glfwGetCursorPos(window, &xpos, &ypos);

	    // Get the size of the window
	    int width, height;
	    glfwGetWindowSize(window, &width, &height);

	    // Convert screen position to world coordinates
	    Eigen::Vector4f p_screen(xpos,height-1-ypos,0,1);
	    Eigen::Vector4f p_canonical(( (p_screen[0])/width)*2-1,( (p_screen[1])/height )*2-1,0,1);
	    //add the xview y view and maybe any other translation here
	    Eigen::Vector4f p_world = view.inverse()*p_canonical;

        Vector2f point(p_world[0],p_world[1]); 

    }

    // Upload the change to the GPU
    VBO.update(V);
    VBO_C.update(C);
}
int ifBloom = 0;
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods){

  //Player 1 Move Paddle
	if(key == GLFW_KEY_W){
		if(translations(1,2) + (scale(2)/2)+ 0.02 < 1){
			translations(1,2) += 0.02; 
			if(ifSleep > 0 && translations(0,4) > 0){translations(1,4) += 0.02;}
		}
	}
    if(key == GLFW_KEY_A){
    	if(translations(2,2) + (scale(2)/2)+ 0.02 < 0.025){
    		translations(2,2) += 0.02; 
    		if(ifSleep > 0 && translations(0,4) > 0){translations(2,4) += 0.02;}
    	}
    }
    if(key == GLFW_KEY_S){
    	if(translations(1,2) - (scale(2)/2) - 0.02 > -0.4){
    		translations(1,2) -= 0.02; 
    		if(ifSleep > 0 && translations(0,4) > 0){translations(1,4) -= 0.02;}
    	}
    }
    if(key == GLFW_KEY_D){
    	if(translations(2,2) - (scale(2)/2) - 0.02 > -1.975){
    		translations(2,2) -= 0.02; 
    		if(ifSleep > 0 && translations(0,4) > 0){translations(2,4) -= 0.02;}
    	}
    }

  //Player 2 Move Paddle
    if(key == GLFW_KEY_UP)   {
    	if(translations(1,3) + (scale(3)/2)+ 0.02 < 1){
    		translations(1,3) += 0.02;
    		if(ifSleep > 0 && translations(0,4) < 0){translations(1,4) += 0.02;}
    	}
    }
    if(key == GLFW_KEY_DOWN) {
    	if(translations(1,3) - (scale(3)/2) - 0.02 > -0.4){
    		translations(1,3) -= 0.02;
    		if(ifSleep > 0 && translations(0,4) < 0){translations(1,4) -= 0.02;}
    	}
    }
    if(key == GLFW_KEY_LEFT) {
    	if(translations(2,3) - (scale(3)/2) - 0.02 > -1.975){
    		translations(2,3) -= 0.02;
    		if(ifSleep > 0 && translations(0,4) < 0){translations(2,4) -= 0.02;}
    	}
    }
    if(key == GLFW_KEY_RIGHT){
    	if(translations(2,3) + (scale(3)/2) + 0.02 < 0.025){
    		translations(2,3) += 0.02;
    		if(ifSleep > 0 && translations(0,4) < 0){translations(2,4) += 0.02;}
    	}
    }

    if(key == GLFW_KEY_TAB && action == GLFW_RELEASE){
    	if(ifBloom == 0){
    		ifBloom = 1;
    		cout << "Blur On" <<endl;
    	}
    	else{
    		ifBloom = 0;
    		cout << "Blur Off" <<endl;
    	}
    }


  // Upload the change to the GPU
  IBO.update(E);
}

int main(void)
{
  	scale << 1;
  	worldRotation(1) += toRadian(180); //fix this and flip everything. I dont know why I flipped my x axis
  
    GLFWwindow* window;

    //Init stuff
    
    // Initialize the library
    if (!glfwInit())
        return -1;

    // Activate supersampling
    glfwWindowHint(GLFW_SAMPLES, 8);

    // Ensure that we get at least a 3.2 context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);

    // On apple we have to load a core profile with forward compatibility
    #ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	#endif

    // Create a windowed mode window and its OpenGL context and disable resize
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    window = glfwCreateWindow(800, 800, ":SPong:", NULL, NULL);
    if (!window){glfwTerminate(); return -1;}

    // Make the window's context current
    glfwMakeContextCurrent(window);

    //something to do with apple
    #ifndef __APPLE__
      glewExperimental = true;
      GLenum err = glewInit();
      if(GLEW_OK != err)
      {
        /* Problem: glewInit failed, something is seriously wrong. */
       fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
      }
      glGetError(); // pull and savely ignonre unhandled errors like GL_INVALID_ENUM
      fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
    #endif

    // print version info
    int major, minor, rev;
    major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
    minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
    rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
    printf("OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
    printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
    printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

    
    /*************
      Create VAOs
     *************/
    VertexArrayObject VAO_Quad;
    VAO_Quad.init();

    // Initialize the VAO
    VertexArrayObject VAO;
    VAO.init();
    VAO.bind();

    // Initialize the VBOs with the vertices data
    // A VBO is a data container that lives in the GPU memory
    VBO.init();
    VBO_Quad.init();

    //add the bottom bar we should put fun windows buttons on it and like win xp and stuff
    V.resize(3,4);
    V <<
       0.8  ,  0.8,   -0.8, -0.8,//x
      -0.245, -0.7, -0.245, -0.7, //y
       0.5  ,  0.5,    0.5,  0.5;//z
    VBO.update(V);

    V_Quad <<
    	-1.0, 1.0,  1.0,  1.0, -1.0, -1.0, //x
    	 1.0, 1.0, -1.0, -1.0, -1.0,  1.0, //y
    VBO_Quad.update(V_Quad);

    U_Quad.init();
    UV_Quad << 
    	0.0, 1.0, 1.0, 1.0, 0.0, 0.0,
    	1.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    U_Quad.update(UV_Quad);

    /************************************
      LOAD INITIAL DATA INTO RENDER VBOS
        ELEMENTS, COLORS, NORMALS, UVs
     ************************************/
    IBO.init();
    IBO.bind();
    E << 0,1,2;
    IBO.update(E);
    
    // Second VBO for colors
    VBO_C.init();

    C.resize(3,4);
    C <<
      0,  1, 1, 1,
      1,  0, 1, 1,
      1,  1, 0, 1;

    NormalBuffer.init();
    NormalBuffer.bind();

    U.init();
    U.bind();
    
    Vector3f a = getTriangleNormal(V.col(0), V.col(1), V.col(2));
    for(int i = 0; i < 3; i++){
      N.col(i) = a;
    }

    //import shape
    cuboidOffset = importCuboidVertex();
    cubeOffset = importCubeVertex(); //cube offset 3 num vertex 8 (indexes 3 to 10)
    bunnyOffset = importBunnyVertex();
    bumpyOffset = importBumpyCubeVertex(); 
    sphereOffset = importSphere();

	/*******************************
	      set up the background 
	  add the paddles and the ball
	********************************/
    //background is wireframe 

    srand (time(NULL));

    addCuboid(); //the background //obj 1
    translations(2,1) = -0.475;

    addSphere(); //obj 2 paddle 1
    translations(0,2) = 1;
    translations(1,2) = 0.3;
    translations(2,2) = -1;
    scale(2) = 0.25;
    addSphere(); //obj 3 paddle 2
    translations(0,3) = -1;
    translations(1,3) = 0.3;
    translations(2,3) = -1;
    scale(3) = 0.25;

    addSphere(); //obj 4, pong ball
    scale(4) = 0.1;
    translations(1,4) = 0.3;
    translations(2,4) = -1;
    //put ball randomly one player 1 or player 2's side
    //size of the ball is scaled to 1 then .25 of it so -.75 or _.75 IF the ball was the same size but it is not
    translations(0,4) = (rand()%2 == 0) ? -0.815 : 0.815;

    ballDirection(0)= (translations(0,4) < 0) ? 0.005 : -0.005;
    Vector3f angles; 
    angles(0) = (rand()%2 == 0) ? toRadian(45) : 0; //angles(0) = (rand()%2 == 0) ? angles(0) *= -1: angles(0);
    angles(1) = (rand()%2 == 0) ? toRadian(45) : 0; //angles(1) = (rand()%2 == 0) ? angles(1) *= -1: angles(1);
    angles(2) = (rand()%2 == 0) ? toRadian(45) : 0; //angles(2) = (rand()%2 == 0) ? angles(2) *= -1: angles(2);
    ballDirection = rotate3D(ballDirection, ORIGIN, angles); 
    //if ball is on negative x then it will move to positive x and vice versa in 45 degree angle
    //randomly if 45 degrees up or down though
    //so ball direction is 1,0,0 or -1,0,0 rotated 45 degrees then scaled down to 5% of thatmaybe 10

    C_Orange.conservativeResize(3, C.cols()); //this doesnt need to be bigger than the 2 paddles
    for(int i = 0; i < C_Orange.cols(); i++){
    	C_Orange.col(i) << 1, 0.5647, 0.1216;
    }

    VBO.update(V);
    VBO_C.update(C);
    NormalBuffer.update(N);
    U.update(UV);

    
    /*******************************************************
    				  CREATE SHADER PROGRAMS
     *******************************************************/
    Program program;
    const GLchar* vertex_shader =   
            "#version 150 core\n"
            		"in vec2 texcoord;"			"out vec2 f_texcoord;"
                    "in vec3 position;"        	"out vec3 f_position;" 
                    "in vec3 color;" 			"out vec3 f_color;"
                    "in vec3 normal;"			"out vec3 f_normal;"
                    "uniform vec3 lightPos;"	"out vec3 f_lightPos;" 
                    //transformation matrices and vectors
                    "uniform vec3 rotation;" //rotation matrix the degrees of rotation for x y and z axis
                                    //access vec 3 via vec3.x vec3.y vec3.z respectively for (0) (1) and (2)
                    "uniform vec3 translation;" //translation matrix
                    "uniform float scale;" //scaling matrix
                    "uniform vec3 w_rotation;" 	"uniform float w_scale;"
                    "uniform float ambient;" 	"flat out float f_ambient;"
                            	                   
                    "uniform mat4 view;"
                    "uniform int ifSpecular;"  	"flat out int f_ifSpecular;"
                    "uniform int ifPerspective;"
                    //"uniform int ifTextured;"  	"flat out int f_ifTextured;" // 0 if not 1 if texture 2 if bump map 3 if normal
                    "void main()"
                    "{"
                        "mat3 rotX = mat3("
                          "vec3( 1,   0.0,               0.0),"
                          "vec3(0.0,  cos(rotation.x),  -sin(rotation.x)),"
                          "vec3(0.0,  sin(rotation.x),   cos(rotation.x))"
                        ");"
                        "mat3 rotY = mat3("
                          "vec3( cos(rotation.y),  0.0,  sin(rotation.y)),"
                          "vec3( 0.0,              1.0,  0.0),"
                          "vec3(-sin(rotation.y),  0.0,  cos(rotation.y))"
                        ");"
                        "mat3 rotZ = mat3("
                          "vec3( cos(rotation.z),  sin(rotation.z),  0.0),"
                          "vec3(-sin(rotation.z),  cos(rotation.z),  0.0),"
                          "vec3(        0.0,         0.0,            1.0)"
                        ");" //obj rotation matrices
                        "mat3 w_rotX = mat3("
                          "vec3( 1,   0.0,               0.0),"
                          "vec3(0.0,  cos(w_rotation.x),  -sin(w_rotation.x)),"
                          "vec3(0.0,  sin(w_rotation.x),   cos(w_rotation.x))"
                        ");"
                        "mat3 w_rotY = mat3("
                          "vec3( cos(w_rotation.y),  0.0,  sin(w_rotation.y)),"
                          "vec3( 0.0,              1.0,  0.0),"
                          "vec3(-sin(w_rotation.y),  0.0,  cos(w_rotation.y))"
                        ");"
                        "mat3 w_rotZ = mat3("
                          "vec3( cos(w_rotation.z),  sin(w_rotation.z),  0.0),"
                          "vec3(-sin(w_rotation.z),  cos(w_rotation.z),  0.0),"
                          "vec3(        0.0,         0.0,            1.0)"
                        ");" //world rotation matrices
                        "mat3 rotMat = rotX * rotY *rotZ;"
                        "mat3 w_rotMat = w_rotX * w_rotY * w_rotZ;"
                        "vec3 projectedPos = (position * rotMat * scale + translation) * w_rotMat* w_scale;"
                        "float perspectiveFactor = projectedPos.z * 0.5 + 1.0;"
      
                        "if(ifPerspective == 0){gl_Position = view * vec4(projectedPos, 1.0);}"
                        "if(ifPerspective == 1){"
                            "projectedPos.y = projectedPos.y - 0.25;" //move the eye to the top half
                            "projectedPos = projectedPos/perspectiveFactor;"
                            "projectedPos.y = projectedPos.y + 0.25;"
                            "gl_Position = view * vec4(projectedPos, 1.0);"
                        "}"
      					"projectedPos.x = projectedPos.x * -1;"
      //rotation and scale first since they need to be done at bary, then translate
                        "f_color = color;"
                        "f_normal = normal*rotMat;"
                        "f_position = projectedPos;"
                        "f_lightPos = lightPos;"
                        "f_ifSpecular = ifSpecular;"
						"f_ambient = ambient;"
						"f_texcoord = texcoord;"
                    "}";
    const GLchar* fragment_shader = 
            "#version 150 core\n"
                    "in vec3 f_color;"
                    "in vec3 f_position;"
                    "in vec3 f_lightPos;"
                    "in vec3 f_normal;"
                    "in vec2 f_texcoord;"
                    //"flat in int f_ifTextured;"
                    "uniform int ifTextured;" 
      				"flat in float f_ambient;"
                    "flat in int f_ifSpecular;"
                    "out vec4 outColor;"
                    "uniform sampler2D grid;" "uniform sampler2D spongeNormal;" "uniform sampler2D spongeBump;"
                    "uniform vec3 triangleColor;"
                    "void main()"
                    "{"
                    	"vec4 colimg;"
                    	"if(ifTextured == 1){"
                    		"colimg = texture(grid, f_texcoord);"
                    	"}"
                    	"if(ifTextured == 2){"
                    		"colimg = texture(spongeNormal, f_texcoord);"
                    	"}"
                    	"if(ifTextured == 3){"
                    		"colimg = texture(spongeBump, f_texcoord);"
                    	"}"
                    	"vec3 colimg3d = colimg.xyz;"
                        "vec3 norm = normalize(f_normal);"
                        "vec3 eye = normalize(-f_position);"
                        "vec3 lightdir = normalize(f_lightPos - f_position);"
                        "vec3 reflectdir = reflect(-lightdir,norm);"
                        "float diffuse = clamp(dot(norm,lightdir),0,1);" //cosTheta
                        //"diffuse = 1 - diffuse;"
                        "vec3 specularStrength = vec3(1,1,1);"
                        "vec3 specFactor = vec3(1,1,1);"
                        "specFactor = pow(max(dot(eye,reflectdir),0.0),32)*specularStrength;"
                        "if(ifTextured == 0){"
	                        "if(f_ifSpecular == 1){"
	                            "outColor = vec4(f_color*((1*f_ambient)+diffuse+specFactor), 1.0);" //1 is the ambient light str"
	                        "}"
	                        "else{"
	                        "   outColor = vec4(f_color*(1+diffuse), 1.0);" //1 is the ambient light str
	                        "}"
	                    "}"
	                    "if(ifTextured != 0){"
	                    	"outColor = vec4(colimg3d*(1+diffuse), 1.0);"
	                    "}"
                    "}";

    // Compile the two shaders and upload the binary to the GPU
    // Note that we have to explicitly specify that the output "slot" called outColor
    // is the one that we want in the fragment buffer (and thus on screen)
    program.init(vertex_shader,fragment_shader,"outColor");
    program.bind();

    program.bindVertexAttribArray("position",VBO);
    program.bindVertexAttribArray("color",VBO_C);
    program.bindVertexAttribArray("normal",NormalBuffer);
    program.bindVertexAttribArray("texcoord",U);

    // Get size of the window + add view matrix
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    float aspect_ratio = float(height)/float(width); // corresponds to the necessary width scaling
    view <<
      aspect_ratio,0, 0, 0,
      0,           1, 0, 0,
      0,           0, 1, 0,
      0,           0, 0, 1;
    glUniformMatrix4fv(program.uniform("view"), 1, GL_FALSE, view.data());

    glUniform3f(program.uniform("lightPos"), lightPos(0), lightPos(1), lightPos(2));

    Program program2;

    const GLchar* vertex_shader2 = 
    	"#version 150 core\n"
    		"in vec2 position;"
    		"in vec2 texCoord;"	"out vec2 f_texCoord;"
    		"void main(){"
    			"f_texCoord = texCoord;"
    			"gl_Position = vec4(position,0.0,1.0);"
    		"}";

    const GLchar* fragment_shader2 = 
        "#version 150 core\n"
        	"in vec2 f_texCoord;"
        	"out vec4 outColor;"
        	"uniform sampler2D texFramebuffer;" //rendered image
        	"uniform vec2 offset;" //determines horizontal or vertical
        	"const float weights[9] = float[](0.00481007202f, 0.0286864862f, 0.102712765f,"
        										"0.220796734f,  0.284958780f, 0.220796734f,"
        										"0.102712765f, 0.0286864862f, 0.00481007202f);"
        	"void main(){"
        		"vec4 sum = vec4(0.0);"
        		"for(int i = -4; i < 4; i++){" 
        			"sum += texture(texFramebuffer, f_texCoord + i * offset) * weights[i+4];"
        		"}"
        		"outColor = sum;"
        	"}";

    program2.init(vertex_shader2, fragment_shader2, "outColor"); 
    VAO_Quad.bind(); //What wizardry makes me need this here? Why can I not use the same VAO as the other program????
    program2.bind();
    program2.bindVertexAttribArray("position",VBO_Quad);
    program2.bindVertexAttribArray("texCoord",U_Quad);


    /**************************
     *  Add Textures to Blur  *
     **************************/
    glUniform1i(glGetUniformLocation(program2.program_shader, "texFramebuffer"), 0);
    GLint uniTexOffset = glGetUniformLocation(program2.program_shader, "offset");


    VAO.bind();
    program.bind(); //Bind the first program again just in case

    /**************************
     * Add Textures to Render *
     **************************/
    GLuint textures[3];
    glGenTextures(3, textures);
    int imgWidth, imgHeight;
	unsigned char* image;

	glActiveTexture(GL_TEXTURE0); 
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glBindTexture(GL_TEXTURE_2D, textures[0]);
	image = SOIL_load_image("../data/vaporwavegrid.png", &imgWidth, &imgHeight, 0, SOIL_LOAD_RGB);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imgWidth, imgHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
	glUniform1i(program.uniform("grid"), 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, textures[1]);
	image = SOIL_load_image("../data/spongebumpmap.jpg", &imgWidth, &imgHeight, 0, SOIL_LOAD_RGB);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imgWidth, imgHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
	glUniform1i(program.uniform("spongeBump"), 1);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glActiveTexture(GL_TEXTURE2); 
	glBindTexture(GL_TEXTURE_2D, textures[2]);
	image = SOIL_load_image("../data/spongenormalmap.jpg", &imgWidth, &imgHeight, 0, SOIL_LOAD_RGB);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imgWidth, imgHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
	glUniform1i(program.uniform("spongeNormal"), 2);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    /************************
        Register Callbacks
     ************************/
    // keyboard mouse framebuffer callbacks
    // Register the keyboard callback
    glfwSetKeyCallback(window, key_callback);

    // Register the mouse callback
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    // Update viewport
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    //glClearColor(1.0f, 0.0f, 1.0f, 1.0f); 
    //glClearColor(0.9f, 0.8f, 0.8f, 1.0f); //red = 0.9686;  green = 0.79216; blue = 0.7882;
    //glClearColor(0.6f, 0.7f, 0.8f, 1.0f);//red = 0.568627; green = 0.6588; blue = 0.8196;
    glClearColor(0.5f, 0.0f, 0.3f, 1.0f); //red = 0.529; blue = 0.29;
        
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    //Some variables
    int ifBallFast = 0;
    int howFast = 1;
    int ifGrabbed = 0;
    int grabbedInd = -1;
    //scores
    int player1 = 0; int player2 = 0;
    int server = (translations(0,4) < 0) ? 2 : 1;
    int numPowerups = 0;
    // Save the current time --- it will be used to dynamically change the triangle color
    auto t_start = std::chrono::high_resolution_clock::now();
	
	/***********************
	  Generate Framebuffers
	 ***********************/
	//generate frame bufers
	//VAO_Quad.bind();
	program2.bind();
	GLuint frameBuffer[2];
    glGenFramebuffers(2, frameBuffer);

    //generate colorbuffer for normal rendering and brightness
    GLuint colorBuffer[2];
    glGenTextures(2, colorBuffer);

	glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer[1]);
    glBindTexture(GL_TEXTURE_2D, colorBuffer[1]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 800, 800, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL); //800x800 window
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);  
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorBuffer[1], 0); //attach to framebuffer

    glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer[0]);
    glBindTexture(GL_TEXTURE_2D, colorBuffer[0]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 800, 800, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL); //800x800 window
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);  
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorBuffer[0], 0);

    //currently framebuffer[0] should be bound rn
    GLuint rbo;
    glGenRenderbuffers(1, &rbo); 
    glBindRenderbuffer(GL_RENDERBUFFER, rbo); 
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, 800, 800); //800x800 window amd only depth test no stencil

    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo); //attach the renderbuffer
    // use a single renderbuffer object for both a depth AND stencil buffer.
 	check_gl_error(); 	
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE){
        cout << glCheckFramebufferStatus(GL_FRAMEBUFFER) << " No Framebuffer" << endl;
        return 1;
    }
    
    // Bind your VAO (not necessary if you have only one)
   	VAO.bind();
    program.bind();
    cout <<"Game Start!" <<endl;
    while (!glfwWindowShouldClose(window)){ 

      	//bind first framebuffer to render
      	int ifBloomLocal = ifBloom;
      	if(ifBloomLocal == 1){
      		glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer[0]); check_gl_error(); 	
      	}
      	else{
      		glBindFramebuffer(GL_FRAMEBUFFER, 0); check_gl_error(); 
      	}
      	// Clear the framebuffer
      	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      	glEnable(GL_DEPTH_TEST); 
    	glDepthFunc(GL_LESS);
      	
    	// Bind your VAO (not necessary if you have only one)
   		VAO.bind();
      	// Bind your program
    	program.bind();

    	glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, textures[0]);
		glActiveTexture(GL_TEXTURE1);
    	glBindTexture(GL_TEXTURE_2D, textures[1]);
		glActiveTexture(GL_TEXTURE2); 
		glBindTexture(GL_TEXTURE_2D, textures[2]);

    	
    	//will be disabled when quad render
      //update the matrix incase frame buffer has changed.
      //we do perspective/ortho switching here instead of to key because frame buffer changes
      	glUniformMatrix4fv(program.uniform("view"), 1, GL_FALSE, view.data());
      	glUniform3f(program.uniform("w_rotation"), worldRotation(0), worldRotation(1), worldRotation(2));
      	glUniform1f(program.uniform("w_scale"), worldScale);
      	glUniform1i(program.uniform("ifPerspective"),ifPerspective);
      //void glUniformMatrix4fv(GLint location,GLsizei count,GLboolean transpose,const GLfloat *value);
      
      // Set the uniform value depending on the time difference
      	auto t_now = std::chrono::high_resolution_clock::now();
      	float time = std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start).count();
      //glUniform3f(program.uniform("triangleColor"), (float)(sin(time * 4.0f) + 1.0f) / 2.0f, 0.0f, 0.0f);

      	//put while loop here to continue only while game is going on.
      	//this makes it wait one frame before it pauses when the ball is set up again so it shows the ball set up and not the ball falling into the void why does this cause problems? 
      	//this causes it to fail with some block error during runtime
      	if(ifSleep > 0){
      		ifSleep--;
      	}
      	if(ifGrabbed > 0){
      		if(ifGrabbed == 200){
      			translations(0,grabbedInd) +=0.005;
      		}
      		else{
      			if(ifGrabbed%2 == 0){translations(0,grabbedInd) +=0.01;}
      			else{translations(0,grabbedInd) -=0.01;}
      		}
      		ifGrabbed--;
      		if(ifGrabbed == 0){
      			Vector3f rangles;
      			for(int i = 0; i < 3; i++){ rangles(i) = toRadian(rand()%360);}
      			ballDirection = rotate3D(ballDirection, ORIGIN, rangles); 
      			if(grabbedInd != -1){deleteObj(grabbedInd); numPowerups --;}
      		}
      	}
      	if(ifBallFast > 0){
      		ifBallFast--;
      		if(ifBallFast == 0){howFast = 1;}
      	}
      	if(ifSleep == 0 && ifGrabbed == 0){
      		translations.col(4) += ballDirection*howFast;
      	}
      	//glUniform3f(program.uniform("lightPos"), translations(0,4), translations(1,4),translations(2,4));
      	//translations.col(4) += ballDirection;
		//get cursor coords
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);
		glfwGetWindowSize(window, &width, &height);

		// Convert screen position to world coordinates
    	Eigen::Vector4f p_screen(xpos,height-1-ypos,0,1);
		Eigen::Vector4f p_canonical(( (p_screen[0])/width)*2-1,( (p_screen[1])/height )*2-1,0,1);
		Eigen::Vector4f p_world = view.inverse()*p_canonical;

		// Draw triangles
		C_Black.conservativeResize(3, C.cols());
		C_Black.setZero();

		/*************************
      	 * Check Collisions Here *
      	 *************************/
      	//check if ball hit anything

      	//1. Check if ball hit obj 1-3 ie cube and paddles

      	////  Check if ball is in range of obj 1-3
      	//// if ball translationX > 1 or < -1 then is past wall and point scored
      	int ifCollision = 0;
      	float distancePaddle1 = (translations.col(4)-translations.col(2)).norm();
      	float distancePaddle2 = (translations.col(4)-translations.col(3)).norm();
      	//cout << distanceClosestPaddle <<":"<< (translations.col(4)-translations.col(2)).norm() << ":" << (translations.col(4)-translations.col(3)).norm() <<endl;
      	if(translations(0,4) > 1 || translations(0,4) < -1){
      		if(translations(0,4) > 0){ //obj 2
      			//left side player (player 1) loses or player 2 scores
      			player2++;
      			//set up ball back on scoring person's side
      			//translations(0,4) = -0.815;
      			//switch servers after 2 points unless after 10/10 split
      			
      		}
      		else{ //obj 3
      			//player 1 scores
      			player1++;
      		}
      		if((player1+player2) > 19){if(server == 2){server = 1;} else{server = 2;}}
      		if((player1+player2) % 2 == 0){if(server == 2){server = 1;} else{server = 2;}}
      		setUpBall(server);
      		translations(1,4) = translations(1,server+1);
      		translations(2,4) = translations(2,server+1);
      		cout << "Player 1: " << player1 <<" Player 2: " << player2 << endl;
      		//if one player reaches eat least 10 points and has a 2 point lead win game and game ends
      	}
      	//// Check if ball hit both the wall and the paddle if so reverse direction
      	else if( (distancePaddle1 < (scale(2)+scale(4))/2 || distancePaddle2 < (scale(3)+scale(4))/2) &&
      			 ( translations(1,4) + scale(4)/2 > 1 || (translations(1,4) - scale(4)/2) < -0.4  || 
      			  (translations(2,4) + scale(4)/2) > 0.025 || (translations(2,4) - scale(4)/2) < -1.5 ) ){ 
      		ifCollision = 1;
      		//both if it hit either paddle AND walls. Check whether hit z plane or y plane and or
      		//if hit z and y plane then fully reverse, otherwise reverse the x and whichever plane it did not hit
      		if(  ( (translations(1,4) + scale(4)/2) > 1 || (translations(1,4) - scale(4)/2) < -0.4 )  &&
      			 ( (translations(2,4) + scale(4)/2) > 0.025 || (translations(2,4) - scale(4)/2) < -1.5 ) ){
      			//hit corner corner
      			ballDirection *= -1;
      		} //hit corner
      		else{
      			//hit edge
      			if( (translations(1,4) + scale(4)/2) > 1 || (translations(1,4) - scale(4)/2) < -0.4 ){
      				ballDirection(0) *= -1; ballDirection(1) *= -1;
      			}
      			if( (translations(2,4) + scale(4)/2) > 0.5 || (translations(2,4) - scale(4)/2) < -1.5 ){
      				ballDirection(0) *= -1; ballDirection(2) *= -1;
      			}
      		} //hit edge
      	}
      	else{ //check for these 3 clauses

      		//if hit paddle
      		if(distancePaddle1 < (scale(2)+scale(4))/2 || distancePaddle2 < (scale(3)+scale(4))/2){
      			//cout <<"Haha Gottem" <<endl;
      			Vector3f origin1;
      			//cout << Objs(1,4) <<endl;
	      		if(distancePaddle1 < (scale(2)+scale(4))/2){cout << "Ping" <<endl;
	      			origin1 = translations.col(2);
	      		}
	      		else if(distancePaddle2 < (scale(3)+scale(4))/2){ cout << "Pong" <<endl;
	      				origin1 = translations.col(3);
	      		}
	      		ifCollision = 1;
		      	Vector3f colNormal(translations(0,4), translations(1,4), translations(2,4)); 
		      	ballDirection = ballDirection - (2*(ballDirection.dot(colNormal))*colNormal);
		      	ballDirection.normalize();
		      	ballDirection = ballDirection * 0.02;
	      		//reflection = direction - 2 (direction.dot(normal)) * normal
      		}
      		//if hit wall
      		else if( ( (translations(1,4) + scale(4)/2) > 1 || (translations(1,4) - scale(4)/2) < -0.4 )  &&
      			( (translations(2,4) + scale(4)/2) > 0.025 || (translations(2,4) - scale(4)/2) < -1.5 )){
      			//if hit 2 walls
      			ballDirection(1) *= -1; ballDirection(2) *= -1;
      		}
      		else if((translations(1,4) + scale(4)/2) > 1 || (translations(1,4) - scale(4)/2) < -0.4 ){
      			//if hit floor normal is 0,0,+-1
      			ballDirection(1) *= -1;
      		}
      		else if((translations(2,4) + scale(4)/2) > 0.025 || (translations(2,4) - scale(4)/2) < -1.5){
      			//if hit wall normal is 0,+-1,0
      			ballDirection(2) *= -1;
      		}
	      	//// if ball translationY + 0.5*ballScale > 1 or ball translationY - 0.5*ballscale < -0.4
	      	//// if ball translationZ + 0.5*ballScale > 0.5 or ball translationZ - 0.5*ballscale < -1.5
      	}
      	//2. Check if ball hit powerups ie objs 5-6
      	//if bunny then 1.05x faster with a max of 1.25x faster
      	if(Objs.cols() > 5){ 
      		for(int i = 5; i < Objs.cols(); i++){
      			//check to see if in range of any of the powerups' bary
      			//if so check if they touch
      			if((translations.col(4) - translations.col(i)).norm() <  (scale(4)+scale(i))/2){
      				//hit if bumpycube or cube
      				if(scale(i) > powerupScale){
      					//check if hit bunny
      					//if hit bunny break
      					cout << "Weeeeeeeeeee!" << endl;
      					ifBallFast = 200;
      					howFast *= 1.5;
      					deleteObj(i);
      					break;
      				}
      				else{
      					
      					//determine what kind of powerup it collided with
      					//if bumpycube destroy ball and start it again
      					if(Objs(2,i) == bumpyEl.cols()){ //only the bunny has 3000 cols too
      						if(ballDirection(0) > 0){player1++; cout << "Ball popped by Player 2!" <<endl;} //if going towards player 1 side then player 1 scores
      						else{player2++; cout << "Ball popped by Player 2!" <<endl;}
      						if((player1+player2) > 19){if(server == 2){server = 1;} else{server = 2;}}
			      			if((player1+player2) % 2 == 0){if(server == 2){server = 1;} else{server = 2;}}
			      			setUpBall(server);
			      			cout << "Player 1: " << player1 <<" Player 2: " << player2 << endl;
			      			deleteObj(i);
			      			numPowerups --;
      					}
      					else if(Objs(2,i) == cubeEl.cols() && ifGrabbed == 0){
      						grabbedInd = i;
      						ifGrabbed = 200;
      						translations.col(4) = translations.col(i);
      						//grab and hold before firing it in some random dir
      						cout << "Snared!" << endl;
      					}
      					break;
      				}
      			}
      		}
      	}

      	/*************
      	   Rendering
      	 *************/
		/*Draw the bottom black bar first*/
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		//ifPerspective = 0;
		//
		glUniform3f(program.uniform("rotation"), rotations(0,0), rotations(1,0), rotations(2,0));
		glUniform3f(program.uniform("translation"), translations(0,0),translations(1,0),translations(2,0));
		glUniform1f(program.uniform("scale"), scale(0));
		glUniform1i(program.uniform("ifTextured"), 0);
		glDrawArrays(GL_TRIANGLE_STRIP,0,4);
		//glDrawArrays(GL_TRIANGLES,3,3);
		//ifPerspective = 1;
		//glUniform1i(program.uniform("ifPerspective"),ifPerspective);
		float ambient = 0.90;
		//Draw Cuboid
		glUniform3f(program.uniform("rotation"), rotations(0,1), rotations(1,1), rotations(2,1));
		glUniform3f(program.uniform("translation"), translations(0,1),translations(1,1),translations(2,1));
		glUniform1f(program.uniform("scale"), scale(1));
		glUniform1i(program.uniform("ifTextured"), 2);
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL);
		for(int j = Objs(0,1); j < Objs(1,1)-6; j+=3){
		    glDrawArrays(GL_TRIANGLES, j, 3);
		}
		
		for(int i = 2; i < 5; i++){
			ambient = (ifCollision == 1) ? 0.75:0.9;

		    glUniform1f(program.uniform("ambient"), ambient);
		  
		    glUniform3f(program.uniform("rotation"), rotations(0,i), rotations(1,i), rotations(2,i));
		    glUniform3f(program.uniform("translation"), translations(0,i),translations(1,i),translations(2,i));
		    glUniform1f(program.uniform("scale"), scale(i));
		    glUniform1i(program.uniform("ifSpecular"), 0);
		    if(i == 4){glUniform1i(program.uniform("ifTextured"), 3);} else{glUniform1i(program.uniform("ifTextured"), 1);}
		    
		    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		    for(int j = Objs(0,i); j < Objs(1,i); j+=3){
		      glDrawArrays(GL_TRIANGLES, j, 3);
		    }
		    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		    VBO_C.update(C_Orange);
		    for(int j = Objs(0,i); j < Objs(1,i); j+=3){glDrawArrays(GL_TRIANGLES, j, 3);}
		      //glDrawElements(GL_TRIANGLES, Objs(2,i), GL_UNSIGNED_INT, (void*)Objs(0,i));
		    VBO_C.update(C);

		}
		ambient = 0.90;
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		//draw cube except the last 2 faces which are for collusion only

		if(Objs.cols() > 5){
		  for(int i = 4; i < Objs.cols(); i++){ 
		    //for each object upload its rotation translation and scale matrices
		    //then draw objects
		    glUniform1f(program.uniform("ambient"), ambient);
		  
		    glUniform3f(program.uniform("rotation"), rotations(0,i), rotations(1,i), rotations(2,i));
		    glUniform3f(program.uniform("translation"), translations(0,i),translations(1,i),translations(2,i));
		    glUniform1f(program.uniform("scale"), scale(i));
		    glUniform1i(program.uniform("ifSpecular"), 0);
		    glUniform1i(program.uniform("ifTextured"), 0);
		    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		    
		    if(Objs(3,i) == 1){glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );}
		    if(Objs(3,i) == 3){glUniform1i(program.uniform("ifSpecular"), 1);}//phong
		    //cout << Objs(0,i) <<"-" <<Objs(1,i)<<endl;
		    for(int j = Objs(0,i); j < Objs(1,i); j+=3){
		      glDrawArrays(GL_TRIANGLES, j, 3);
		    }
		    //glDrawElements(GL_TRIANGLES, Objs(2,i), GL_UNSIGNED_INT, (void*)Objs(0,i));
		    if(Objs(3,i) == 2){
		      VBO_C.update(C_Black);
		      glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		      for(int j = Objs(0,i); j < Objs(1,i); j+=3){glDrawArrays(GL_TRIANGLES, j, 3);}
		      //glDrawElements(GL_TRIANGLES, Objs(2,i), GL_UNSIGNED_INT, (void*)Objs(0,i));
		      VBO_C.update(C);
		    }//flat + wireframe
		  }
		}

		/********************************
		 *     Try to spawn powerup     *
      	 ********************************/
      	if(numPowerups < 3){
      		if(rand() % 1000 == 0){ 
      			numPowerups++;
      			//cout << numPowerups <<endl;
      			int powerupType = rand()%3;
      			if(powerupType == 0){
      				addBunny();
      				scale(scale.cols()-1) = powerupScale+0.05;
      			}
      			else if(powerupType == 1){
      				addBumpyCube();
      				scale(scale.cols()-1) = powerupScale;
      			}
      			else{
      				addCube();
      				scale(scale.cols()-1) = 0.15;
      			}
      			translations(0,scale.cols()-1) = (float)((rand()%200)-100)/(float)100;
      			translations(1,scale.cols()-1) = (float)((rand()%130)-30)/(float)100;
      			translations(2,scale.cols()-1) = (float)((rand()%190)+1)/(float)-100;
      			rotations(0, scale.cols()-1) = toRadian(rand()%360);
      			rotations(1, scale.cols()-1) = toRadian(rand()%360);
      			rotations(2, scale.cols()-1) = toRadian(rand()%360);
      		}
      	}

      	if(ifBloomLocal == 1){
			//bind framebuffer
			glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer[1]);
			VAO_Quad.bind();
			glDisable(GL_DEPTH_TEST);
			program2.bind();

			glActiveTexture(GL_TEXTURE0);
	        glBindTexture(GL_TEXTURE_2D, colorBuffer[0]);
	        glUniform2f(uniTexOffset, 1.0f / 300.0f, 0.0f);

	        glDrawArrays(GL_TRIANGLES, 0, 6);

	       	//bind framebuffer
			glBindFramebuffer(GL_FRAMEBUFFER, 0);

			glBindTexture(GL_TEXTURE_2D, colorBuffer[1]);
	        glUniform2f(uniTexOffset, 0.0f, 1.0f / 200.0f);
	        glDrawArrays(GL_TRIANGLES, 0, 6);
	    }

        // Swap front and back buffers
        glfwSwapBuffers(window);
        // Poll for and process events
        glfwPollEvents();

    } //while window still open

    // Deallocate opengl memory
    glDeleteFramebuffers(2, frameBuffer);
    program.free();
    VAO.free();
    VBO.free();
    IBO.free();
    VBO_C.free();

    // Deallocate glfw internals
    glfwTerminate();
    return 0;
}
