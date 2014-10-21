/*
 OM3D allows users to manipulate objects in photographs in 3D
 by using stock 3D models.
 Copyright (C) 2014 Natasha Kholgade and Tomas Simon
 
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2
 of the License, or (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 
 For further details, please contact the authors at nkholgad@cs.cmu.edu
 */


#ifndef __matrixutils_h__
#define __matrixutils_h__

#include <OpenGL/gl.h>
#include <OpenGL/glu.h>


#ifdef __cplusplus
extern "C" {
#endif
	
	typedef struct {
		GLfloat* xarray;
		GLfloat* yarray;
		GLfloat* zarray;
		int size;
		int increment;
		int maxelements;
	} ptsArray;
	
	void insertElementIntoArray(ptsArray* theArray, GLfloat x, GLfloat y, GLfloat z);
	void initArray(ptsArray* theArray);
	void initArrayWithMax(ptsArray* theArray, int mymax);
	void getElementFromArray(ptsArray* theArray, int pos, GLfloat* x, GLfloat* y, GLfloat* z);
	void deleteArray(ptsArray* theArray);
	void clearArray(ptsArray* theArray);
	int testLinePlaneIntersection(GLfloat* la, GLfloat* lb, GLfloat* p0, GLfloat* p1, GLfloat* p2);
	int geq(GLfloat a, GLfloat b); 
	int leq(GLfloat a,GLfloat b);
	
	int findAllElementsInArray(GLfloat* array, GLfloat element, GLfloat* indices, int numArray);
	
	void vertexFaceIntersection(GLfloat* verts, GLfloat* faces, int* output, int nverts, int nfaces, int numperface);
	
	void interp01(GLfloat* facevertices, GLfloat* output, int discnum, int vertstep);
	void interp02(GLfloat* facevertices, GLfloat* output, int discnum, int vertstep);
	
	GLfloat vDotProduct(GLfloat* v1, GLfloat* v2);
	
	void vCrossProduct(GLfloat *v1, GLfloat *v2, GLfloat *r);
	
	GLfloat vNorm(GLfloat* v);
	void vNormalize(GLfloat* v);
	void vScaleByLast(GLfloat* v);
	void vScaleVector(GLfloat* v, GLfloat scale);
	void vScaleVectorToAnother(GLfloat* vin, GLfloat* vout, GLfloat scale);
	
	void printMatrix4(GLfloat* M);
	void printVector(GLfloat* v);
	
	int pointInTriangle(GLfloat* pt, GLfloat* tri);
	int InsidePolygon(GLfloat x, GLfloat y, int N, GLfloat* polygon);
	
	void multMatrix4Vector4(GLfloat* M, GLfloat* x, GLfloat* y);
	
	void transformPoints3(GLfloat* M, GLfloat *X, GLfloat *Y, GLfloat *Z);
	void transformPointsv3(GLfloat* M, GLfloat *v);
	
	void transformPoints4(GLfloat* M, GLfloat *X, GLfloat *Y, GLfloat *Z);
	void transformPointsv4(GLfloat* M, GLfloat *v);
	
	void transformPointToOther4(GLfloat* M, GLfloat *X, GLfloat *Y, GLfloat *Z, GLfloat *Xo, GLfloat *Yo, GLfloat *Zo);

	void multMatrix4Vector3(GLfloat* M, GLfloat* x, GLfloat* y);
	
	void multMatrix4SeveralVerticesDivw(GLfloat* M, GLfloat* Vin, GLfloat* Vout, int numElements);
	
	void multMatrix4SeveralVertices(GLfloat* M, GLfloat* Vin, GLfloat* Vout, int numElements);
	void multMatrix43SeveralVertices(GLfloat* M, GLfloat* Vin, GLfloat* Vout, int numElements);
	
	void multMatrix4Vector4Div(GLfloat* M, GLfloat* x, GLfloat* y);
	
	void subVectors(GLfloat *v1, GLfloat *v2, GLfloat *r);
	
	void addVectors(GLfloat *v1, GLfloat *v2, GLfloat *r);
	// calculate 3D point on a sphere centered at object center given 2D point and projection matrix
	void project2Donto3Dsphere(GLfloat x, GLfloat y, GLfloat* PM, GLfloat * M, GLfloat Xc, GLfloat Yc, GLfloat Zc, GLfloat R, GLfloat* X, GLfloat *Y, GLfloat *Z);
	
	void resetToIdentity(GLfloat *M);
	
	void calculateDistancesFromPointToPointsInVector(GLfloat* Vin, GLfloat x, GLfloat y, GLfloat* distances, int numElements);
	
	void distanceNDim(GLfloat* Vin, GLfloat* vcomp, GLfloat* distances, int numElements, int numfeatures);
	
	void minWithHighReplacement(GLfloat* vals, GLfloat* minval, int* iminval, int numElements); 
	void findMinVal(GLfloat* vals, GLfloat* minval, int* iminval, int numElements); 
		
#ifdef __cplusplus
}
#endif

#endif