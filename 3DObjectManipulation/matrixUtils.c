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

#include "matrixUtils.h"
#include <math.h>
//#include <cv.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MINN(x,y) (x < y ? x : y)
#define MAXX(x,y) (x > y ? x : y)
#define INSIDE 1
#define OUTSIDE 0

void initArray(ptsArray* theArray)
{
	initArrayWithMax(theArray, 100);
}

void initArrayWithMax(ptsArray* theArray, int mymax)
{
	theArray->size=0; theArray->increment=100; theArray->maxelements=mymax;
	theArray->xarray=malloc(theArray->maxelements*sizeof(GLfloat));
	theArray->yarray=malloc(theArray->maxelements*sizeof(GLfloat));
	theArray->zarray=malloc(theArray->maxelements*sizeof(GLfloat));
}

void insertElementIntoArray(ptsArray* theArray, GLfloat x, GLfloat y, GLfloat z)
{
	if (theArray->size==theArray->maxelements)
	{
		theArray->maxelements+=theArray->increment;
		GLfloat* tmpXarray=malloc(theArray->maxelements*sizeof(GLfloat));
		GLfloat* tmpYarray=malloc(theArray->maxelements*sizeof(GLfloat));
		GLfloat* tmpZarray=malloc(theArray->maxelements*sizeof(GLfloat));
		memcpy(tmpXarray,theArray->xarray,theArray->size*sizeof(GLfloat));
		memcpy(tmpYarray,theArray->yarray,theArray->size*sizeof(GLfloat));
		memcpy(tmpZarray,theArray->zarray,theArray->size*sizeof(GLfloat));
		
		free(theArray->xarray); free(theArray->yarray); 
		free(theArray->zarray); 
		theArray->xarray=tmpXarray;
		theArray->yarray=tmpYarray;
		theArray->zarray=tmpZarray;
	}
	theArray->xarray[theArray->size]=x; 
	theArray->yarray[theArray->size]=y; 
	theArray->zarray[theArray->size]=z;
	theArray->size+=1; 
	/*CvArr* src1;
	CvArr* src2; 
	CvArr* dst1;
	cvAdd(src1, src2, dst1);*/
	
}

void clearArray(ptsArray* theArray)
{
	theArray->size=0;
}

void getElementFromArray(ptsArray* theArray, int pos, GLfloat *x, GLfloat *y, GLfloat* z)
{
	*x=theArray->xarray[pos]; *y=theArray->yarray[pos];
	*z=theArray->zarray[pos]; 
}

void deleteArray(ptsArray* theArray)
{
	free(theArray->xarray); free(theArray->yarray); free(theArray->zarray);
}

int findAllElementsInArray(GLfloat* array, GLfloat element, GLfloat* indices, int numArray)
{
	int i=0; int count=0; 
	
	for (i=0; i<numArray; i++)
	{
		if ( fabs(array[i]-element)<.001)
		{
			indices[count]=i; count++;
		}
	}
	return count;
}


// returns true if there is an intersection with the face, else returns false
int testLinePlaneIntersection(GLfloat* la, GLfloat* lb, GLfloat* p0, GLfloat* p1, GLfloat* p2)
{
	// returns true for indices where intersection occurs
	GLfloat a1=la[0]-lb[0], b1=la[1]-lb[1], c1=la[2]-lb[2];
	GLfloat a2=p1[0]-p0[0], b2=p1[1]-p0[1], c2=p1[2]-p0[2];
	GLfloat a3=p2[0]-p0[0], b3=p2[1]-p0[1], c3=p2[2]-p0[2];
	GLfloat l1=la[0]-p0[0], l2=la[1]-p0[1], l3=la[2]-p0[2];
	
	GLfloat denom=(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
	
	if (fabs(denom)<1e-6) return 0; 
	// if you can't find an intersection because there is no solution, return notFacesSeen=0 ==> you are never blocked
	
	GLfloat num1= (a2*b3*l3 - a3*b2*l3 - a2*c3*l2 + a3*c2*l2 + b2*c3*l1 - b3*c2*l1);
	GLfloat num2=-(a1*b3*l3 - a3*b1*l3 - a1*c3*l2 + a3*c1*l2 + b1*c3*l1 - b3*c1*l1);
	GLfloat num3= (a1*b2*l3 - a2*b1*l3 - a1*c2*l2 + a2*c1*l2 + b1*c2*l1 - b2*c1*l1);
	
	GLfloat t1=num1/denom, t2=num2/denom, t3=num3/denom;
	
	return geq(t1,0) & leq(t1,1) & geq(t2,0) & geq(t3,0) & leq(t3,1) & leq(t2,1) & leq(t2+t3,1);
}

int geq(GLfloat a,GLfloat b) {
	return a>b | fabs(a-b)<1e-6;
}

int leq(GLfloat a,GLfloat b) {
	return a<b | fabs(a-b)<1e-6;
}

// change this to reflect triangles
void vertexFaceIntersection(GLfloat* verts, GLfloat* faces, int* output, int nverts, int nfaces, int numperface)
{
	GLfloat la[3]={.0f, .0f, .0f};
	int i=0; int j=0;
	for (i=0; i<nverts; i++) {
		GLfloat lb[3]; lb[0]=verts[3*i]; lb[1]=verts[3*i+1]; lb[2]=verts[3*i+2];
		int done=0;
		for (j=0; j<nfaces && !done; j++) {
			int f0=faces[j]-1, f1=faces[j+nfaces]-1, f2=faces[j+2*nfaces]-1;
			if (f0!=i && f1 !=i && f2 !=i) {
				GLfloat v0[3], v1[3], v2[3];
				int k=0; for (k=0; k<3; k++) {
					v0[k]=verts[3*f0+k]; v1[k]=verts[3*f1+k]; v2[k]=verts[3*f2+k];
				}
				done=testLinePlaneIntersection(la, lb, v0, v1, v2);
				if (!done)
				{
					if (numperface>3)
					{
						int f3=faces[j+3*nfaces]-1;
						if (f3 != i)
						{
							GLfloat v3[3]; for (k=0; k<3; k++) v3[k]=verts[3*f3+k];
							done=testLinePlaneIntersection(la, lb, v0, v2, v3);
						}
					}
				}
			}
		}
		output[i]=done;
	}
}

// returns true if point is inside triangle, else returns false
int pointInTriangle(GLfloat* pt, GLfloat* tri)
{
	int i=0, j=0;
	//int isin=1;
	for (i=0; i<3; i++)
	{
		int ip1=i+1; if (ip1==3) ip1=0; 
		GLfloat a0=tri[2*ip1]-tri[2*i];
		GLfloat a1=tri[2*ip1+1]-tri[2*i+1];
		GLfloat b0=pt[0]-tri[2*i];
		GLfloat b1=pt[1]-tri[2*i+1];
		GLfloat t=a0*b1-a1*b0;
		//isin = isin && geq(t,0);
		if (t<0) return 0;
	}
	//return isin;
	return 1;
}


int InsidePolygon(GLfloat x, GLfloat y, int N, GLfloat* polygon)
{
	int counter = 0;
	int i;
	GLfloat xinters;
	//Point p1,p2;
	GLfloat x1, y1, x2, y2;
	
	x1 = polygon[0];
	y1 = polygon[1];
	for (i=1;i<=N;i++) {
		x2 = polygon[2*(i % N)];
		y2 = polygon[2*(i % N)+1];
		if (y > MINN(y1,y2)) {
			if (y <= MAXX(y1,y2)) {
				if (x <= MAXX(x1,x2)) {
					if ( fabs(y1 - y2) > 1e-8 ) {
						xinters = (y-y1)*(x2-x1)/(y2-y1)+x1;
						if ( fabs(x1 - x2)<1e-8 || x <= xinters)
							counter++;
					}
				}
			}
		}
		x1 = x2;
		y1 = y2;
	}
	
	return (counter %2 != 0);
	
	/*
	if (counter % 2 == 0)
		return(OUTSIDE);
	else
		return(INSIDE);*/
}

// change this to reflect triangles
void interp01(GLfloat* facevertices, GLfloat* output, int discnum, int vertstep)
{
	int i=0; int j=0; int k=0;
	GLfloat h=1/((GLfloat)(discnum-1));
	GLfloat alpha=.0f, beta=.0f;
	for (i=0; i<discnum; i++)
	{
		for (j=0; j<discnum; j++)
		{
			for (k=0; k<vertstep; k++)
			{
			GLfloat ans;
			GLfloat t0=facevertices[k], t1=facevertices[k+vertstep], t2=facevertices[k+2*vertstep], t3=facevertices[k+3*vertstep];
			GLfloat ta=t0+alpha*(t3-t0);
			GLfloat tb=t1+alpha*(t2-t1);
			ans=ta+beta*(tb-ta);
			
			output[vertstep*(i*discnum+j)+k]=ans;
				//printf("%f\n",ans);
			}
			beta=beta+h;
			
		}
		alpha=alpha+h;
		beta=.0f;
	}
	
}

void interp02(GLfloat* facevertices, GLfloat* output, int discnum, int vertstep)
{
	int i=0; int j=0; int k=0;
	GLfloat h=1/((GLfloat)(discnum-1));
	GLfloat alpha=.0f, beta=.0f;
	int dcount=0;
	for (i=0; i<discnum; i++)
	{
		for (j=0; j<(discnum-i); j++)
		{
			for (k=0; k<vertstep; k++)
			{
				GLfloat ans;
				GLfloat t0=facevertices[k], t1=facevertices[k+vertstep], t2=facevertices[k+2*vertstep];
				ans=t0*(1-alpha-beta)+t1*beta+t2*alpha; // alpha=1 ==> t2, beta==1 => t1, otherwise t0
				
				output[vertstep*(dcount)+k]=ans;
				//printf("%f\n",ans);
			}
			dcount=dcount+1;
			beta=beta+h;
			
		}
		alpha=alpha+h;
		beta=.0f;
	}
	
}

GLfloat vDotProduct(GLfloat* v1, GLfloat* v2)
{
	return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
}

void vCrossProduct(GLfloat* v1, GLfloat* v2, GLfloat* r)
{
	r[0]= v1[1]*v2[2]-v1[2]*v2[1];
	r[1]=-v1[0]*v2[2]+v1[2]*v2[0];
	r[2]= v1[0]*v2[1]-v1[1]*v2[0];
}


void printMatrix4(GLfloat* M)
{
	int i=0;
	for (i=0; i<4; i++)
		printf("%f,%f,%f,%f\n",M[0+i],M[4+i],M[8+i],M[12+i]);
}

void printVector(GLfloat* v)
{
	printf("[%f,%f,%f]",v[0],v[1],v[2]);
	
}

void calculateDistancesFromPointToPointsInVector(GLfloat* Vin, GLfloat x, GLfloat y, GLfloat* distances, int numElements)
{
	int i=0;
	for (i=0; i<numElements; i++) {
		GLfloat xa=Vin[3*i]; GLfloat ya=Vin[3*i+1];
		distances[i]=xa*xa+ya*ya-2.0f*(xa*x+ya*y);
	}
}

void distanceNDim(GLfloat* Vin, GLfloat* vcomp, GLfloat* distances, int numElements, int numfeatures)
{
	int i=0,j=0;
	for (i=0; i<numElements; i++)
	{
		distances[i]=.0f;
		if isinf(Vin[numfeatures*i])
		{
			distances[i]=INFINITY;
			//printf("%f\n",distances[i]);
		}
		else {
		for (j=0; j<numfeatures; j++)
		{
			GLfloat xa=Vin[numfeatures*i+j], xc=vcomp[j];
			distances[i]+=xc*xc+xa*xa-2.0f*xa*xc;
		}
		}
	}
}

void multMatrix4SeveralVerticesDivw(GLfloat* M, GLfloat* Vin, GLfloat* Vout, int numElements)
{
	int i=0; 
	for (i=0; i<numElements; i++) {
		GLfloat vin[3]; GLfloat vout[3];
		vin[0]=Vin[3*i]; vin[1]=Vin[3*i+1]; vin[2]=Vin[3*i+2];
		multMatrix4Vector4Div(M, vin, vout);
		Vout[3*i]=vout[0]; Vout[3*i+1]=vout[1]; Vout[3*i+2]=vout[2];
	}
}

void multMatrix4SeveralVertices(GLfloat* M, GLfloat* Vin, GLfloat* Vout, int numElements)
{
	int i=0; 
	for (i=0; i<numElements; i++) {
		GLfloat vin[3]; GLfloat vout[3];
		vin[0]=Vin[3*i]; vin[1]=Vin[3*i+1]; vin[2]=Vin[3*i+2];
		multMatrix4Vector4(M, vin, vout);
		Vout[3*i]=vout[0]; Vout[3*i+1]=vout[1]; Vout[3*i+2]=vout[2];
	}
}

void multMatrix43SeveralVertices(GLfloat* M, GLfloat* Vin, GLfloat* Vout, int numElements)
{
	int i=0;
	for (i=0; i<numElements; i++) {
		GLfloat vin[3]; GLfloat vout[3];
		vin[0]=Vin[3*i]; vin[1]=Vin[3*i+1]; vin[2]=Vin[3*i+2];
		multMatrix4Vector3(M, vin, vout);
		Vout[3*i]=vout[0]; Vout[3*i+1]=vout[1]; Vout[3*i+2]=vout[2];
	}
}


void multMatrix4Vector4(GLfloat* M, GLfloat* x, GLfloat* y)
{
    float x0=x[0], x1=x[1], x2=x[2];
	y[0]=M[0]*x0+M[4]*x1+M[8]*x2+M[12];
	y[1]=M[1]*x0+M[5]*x1+M[9]*x2+M[13];
	y[2]=M[2]*x0+M[6]*x1+M[10]*x2+M[14];
}

void multMatrix4Vector4Div(GLfloat* M, GLfloat* x, GLfloat* y)
{
	float w=M[3]*x[0]+M[7]*x[1]+M[11]*x[2]+M[15];
	  y[0]=(M[0]*x[0]+M[4]*x[1]+M[8]*x[2]+M[12])/w;
	  y[1]=(M[1]*x[0]+M[5]*x[1]+M[9]*x[2]+M[13])/w;
	  y[2]=(M[2]*x[0]+M[6]*x[1]+M[10]*x[2]+M[14]);   // you may not need to divide.
	
}

void transformPoints3(GLfloat* M, GLfloat *X, GLfloat *Y, GLfloat *Z)
{
	GLfloat in[3], out[3];
	in[0]=*X; in[1]=*Y, in[2]=*Z;
	multMatrix4Vector3(M, in, out);
	*X=out[0]; *Y=out[1]; *Z=out[2];
}

void transformPointsv3(GLfloat* M, GLfloat* v)
{
	transformPoints3(M, &v[0], &v[1], &v[2]);
}


void transformPoints4(GLfloat* M, GLfloat *X, GLfloat *Y, GLfloat *Z)
{
	GLfloat in[3], out[3];
	in[0]=*X; in[1]=*Y, in[2]=*Z;
	multMatrix4Vector4(M, in, out);
	*X=out[0]; *Y=out[1]; *Z=out[2];	
}

void transformPointsv4(GLfloat* M, GLfloat* v) {
	transformPoints4(M, &v[0], &v[1], &v[2]);
}

void transformPointToOther4(GLfloat* M, GLfloat *X, GLfloat *Y, GLfloat *Z, GLfloat *Xo, GLfloat *Yo, GLfloat *Zo)
{
	GLfloat in[3], out[3];
	in[0]=*X; in[1]=*Y, in[2]=*Z;
	multMatrix4Vector4(M, in, out);
	*Xo=out[0]; *Yo=out[1]; *Zo=out[2];	
}

GLfloat vNorm(GLfloat* v) {
	return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}

void vNormalize(GLfloat* v) {
	float nn=vNorm(v);
	v[0]=v[0]/nn; v[1]=v[1]/nn; v[2]=v[2]/nn;
}

void vScaleByLast(GLfloat* v) {
	v[0]=v[0]/v[2]; v[1]=v[1]/v[2]; v[2]=1.0f;
}

void vScaleVector(GLfloat* v, GLfloat scale) {
	v[0]=v[0]*scale; v[1]=v[1]*scale; v[2]=v[2]*scale;
}

void vScaleVectorToAnother(GLfloat* vin, GLfloat* vout, GLfloat scale) {
	vout[0]=vin[0]*scale; vout[1]=vin[1]*scale; vout[2]=vin[2]*scale;
}

void multMatrix4Vector3(GLfloat* M, GLfloat* x, GLfloat* y)
{
    float x0=x[0], x1=x[1], x2=x[2];
	y[0]=M[0]*x0+M[4]*x1+M[8]*x2;
	y[1]=M[1]*x0+M[5]*x1+M[9]*x2;
	y[2]=M[2]*x0+M[6]*x1+M[10]*x2;
}

void subVectors(GLfloat *v1, GLfloat *v2, GLfloat *r)
{
	r[0]=v1[0]-v2[0]; r[1]=v1[1]-v2[1]; r[2]=v1[2]-v2[2];
}

void addVectors(GLfloat *v1, GLfloat *v2, GLfloat *r)
{
	r[0]=v1[0]+v2[0]; r[1]=v1[1]+v2[1]; r[2]=v1[2]+v2[2];
}

void resetToIdentity(GLfloat *M){
	int i=0; 
	for (i=0; i<16; i++)
		M[i]=0.0f;
	for (i=0; i<16; i+=5)
		M[i]=1.0f;
}

void minWithHighReplacement(GLfloat* vals, GLfloat* minval, int* iminval, int numElements)
{
	int i=0;
	*minval=vals[0]; 
	*iminval=0;
	for (i=1; i<numElements; i++) {
		if (vals[i]<*minval)
		{
			*minval=vals[i];
			*iminval=i;
		}
	}
	vals[*iminval]=1e6f; 
}

void findMinVal(GLfloat* vals, GLfloat* minval, int* iminval, int numElements)
{
	int i=0;
	*minval=vals[0]; 
	*iminval=0;
	for (i=1; i<numElements; i++) {
		if (vals[i]<*minval)
		{
			*minval=vals[i];
			*iminval=i;
		}
	}
	
}

// calculate 3D point on a sphere centered at object center given 2D point and projection matrix
void project2Donto3Dsphere(GLfloat x, GLfloat y, GLfloat* PM, GLfloat * M, GLfloat Xc, GLfloat Yc, GLfloat Zc, GLfloat R, GLfloat* X, GLfloat *Y, GLfloat *Z)
{
	GLfloat BB11=PM[0], BB21=PM[1], BB31=PM[3], 
	BB12=PM[4], BB22=PM[5], BB32=PM[7],
	BB13=PM[8], BB23=PM[9], BB33=PM[11],
	BB14=PM[12], BB24=PM[13], BB34=PM[15];
	
	GLfloat B11=BB11-x*BB31, B12=BB12-x*BB32, B13=BB13-x*BB33;
	GLfloat B21=BB21-y*BB31, B22=BB22-y*BB32, B23=BB23-y*BB33;
	GLfloat C1=-(BB11-x*BB31)*Xc - (BB12-x*BB32)*Yc - (BB13-x*BB33)*Zc-(BB14-x*BB34);
	GLfloat C2=-(BB21-y*BB31)*Xc - (BB22-y*BB32)*Yc - (BB23-y*BB33)*Zc-(BB24-y*BB34);
	
	GLfloat Xs1, Xs2, Ys1, Ys2, Zs1, Zs2;
	GLfloat R2=R*R;
	GLfloat ans=R2*B11*B11*B22*B22 + R2*B11*B11*B23*B23 - B11*B11*C2*C2 - 2*R2*B11*B12*B21*B22 - 2*R2*B11*B13*B21*B23 + 2*B11*B21*C1*C2 + 
	R2*B12*B12*B21*B21 + R2*B12*B12*B23*B23 - B12*B12*C2*C2 - 2*R2*B12*B13*B22*B23 + 2*B12*B22*C1*C2 + R2*B13*B13*B21*B21 + 
	R2*B13*B13*B22*B22 - B13*B13*C2*C2 + 2*B13*B23*C1*C2 - B21*B21*C1*C1 - B22*B22*C1*C1 - B23*B23*C1*C1;
	
	if (ans<1e-6) ans=1e-6; // projection -- doesn't work for aligned-axis rotations (maybe some zero related thingy)
	
	GLfloat sqrt1=sqrt(ans);
	
	GLfloat Xs_disc=( B12*B23 - B13*B22)*sqrt1;	
	GLfloat Ys_disc=(-B11*B23 + B13*B21)*sqrt1;
	GLfloat Zs_disc=( B11*B22 - B12*B21)*sqrt1;
	
	GLfloat denom=B11*B11*B22*B22 + B11*B11*B23*B23 - 2*B11*B12*B21*B22 - 2*B11*B13*B21*B23 + B12*B12*B21*B21 + B12*B12*B23*B23 - 2*B12*B13*B22*B23 + 
	B13*B13*B21*B21 + B13*B13*B22*B22;
	GLfloat numerX1=C2*B12*B12 - B22*C1*B12 + C2*B13*B13 - B23*C1*B13;
	GLfloat numerX2=C1*B22*B22 - B12*C2*B22 + C1*B23*B23 - B13*C2*B23;	
	
	GLfloat numerY1=C2*B11*B11 - B21*C1*B11 + C2*B13*B13 - B23*C1*B13;
	GLfloat numerY2=C1*B21*B21 - B11*C2*B21 + C1*B23*B23 - B13*C2*B23;
	
	GLfloat numerZ1=C2*B11*B11 - B21*C1*B11 + C2*B12*B12 - B22*C1*B12;
	GLfloat numerZ2=C1*B21*B21 - B11*C2*B21 + C1*B22*B22 - B12*C2*B22;
	
	Xs1=(B21*numerX1 + B11*numerX2 + Xs_disc)/denom;
	Xs2=(B21*numerX1 + B11*numerX2 - Xs_disc)/denom;
	
	Ys1=(B22*numerY1 + B12*numerY2 + Ys_disc)/denom;
	Ys2=(B22*numerY1 + B12*numerY2 - Ys_disc)/denom;
	
	
	Zs1=(B23*numerZ1 + B13*numerZ2 + Zs_disc)/denom;
	Zs2=(B23*numerZ1 + B13*numerZ2 - Zs_disc)/denom;
	
	GLfloat X1=Xs1+Xc, Y1=Ys1+Yc, Z1=Zs1+Zc, X2=Xs2+Xc, Y2=Ys2+Yc, Z2=Zs2+Zc;
	
	//GLfloat Xproj1=M[0]*X1+M[4]*Y1+M[8]*Z1+M[12];
	//GLfloat Xproj2=M[0]*X2+M[4]*Y2+M[8]*Z2+M[12];
	//GLfloat Yproj1=M[1]*X1+M[5]*Y1+M[9]*Z1+M[13];
	//GLfloat Yproj2=M[1]*X2+M[5]*Y2+M[9]*Z2+M[13];
	GLfloat Zproj1=M[2]*X1+M[6]*Y1+M[10]*Z1+M[14];
	GLfloat Zproj2=M[2]*X2+M[6]*Y2+M[10]*Z2+M[14];
	
	if (Zproj2 > Zproj1)
	{
		*X=X2; *Y=Y2; *Z=Z2;
	} else {
		*X=X1; *Y=Y1; *Z=Z1;
	}
	
	
}
