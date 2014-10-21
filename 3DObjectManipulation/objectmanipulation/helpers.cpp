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

#include "helpers.h"
#include "image.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"


void writeFile(string direc, void* ptr, int size, const char* name) {
	string path=direc;
	FILE* fid=fopen(direc.append(name).c_str(), "w");
	fwrite(ptr, size, 1, fid);
	fclose(fid);
}

void readFile(string direc, void* ptr, int size, const char* name) {
	string path=direc;
	FILE* fid=fopen(direc.append(name).c_str(), "r");
	fread(ptr, size, 1, fid);
	fclose(fid);
}

void readImageInfo( string direc, const char* name, int& width, int& height) {
    string path=direc; path.append(name);
    png::image< png::rgb_pixel > image( path.c_str() );
    width=image.get_width();
    height=image.get_height();
}

void readImage( string direc, const char* name, float* ptr)
{
    string path=direc; path.append(name);
    png::image< png::rgb_pixel > image( path.c_str() );
    int width=image.get_width();
    int height=image.get_height();
    int index=0;
    for (size_t h=0; h<height; h++) {
        for (size_t w=0; w<width; w++) {
            png::rgb_pixel rgb=image[h][w];
            index=w*height+h;
            ptr[3*index  ]=((float)(rgb.red))/255;
            ptr[3*index+1]=((float)(rgb.green))/255;
            ptr[3*index+2]=((float)(rgb.blue))/255;
        }
    }
}

void readImageOpenGL( string direc, const char* name, float* ptr)
{
    string path=direc; path.append(name);
    
    png::image< png::rgb_pixel > image( path.c_str() );
    int width=image.get_width();
    int height=image.get_height();
    int index=0;
    for (size_t h=0; h<height; h++) {
        for (size_t w=0; w<width; w++) {
            png::rgb_pixel rgb=image[h][w];
            index=h*width+w;
            ptr[3*index  ]=((float)(rgb.red))/255;
            ptr[3*index+1]=((float)(rgb.green))/255;
            ptr[3*index+2]=((float)(rgb.blue))/255;
        }
    }
}

void readBinaryImage( string direc, const char* name, bool* ptr)
{
    string path=direc; path.append(name);
    png::image< png::rgb_pixel > image( path.c_str() );
    int width=image.get_width();
    int height=image.get_height();
    int index=0;
    float r, g, b;
    float thresh=.5;
    for (size_t h=0; h<height; h++) {
        for (size_t w=0; w<width; w++) {
            png::rgb_pixel rgb=image[h][w];
            index=w*height+h;
            
            r=((float)(rgb.red))/255;
            g=((float)(rgb.green))/255;
            b=((float)(rgb.blue))/255;
            
            ptr[index]=( r>thresh && g>thresh && b>thresh);
        }
    }
}

void readBinaryImage( string direc, const char* name, float* ptr)
{
    string path=direc; path.append(name);
    png::image< png::rgb_pixel > image( path.c_str() );
    int width=image.get_width();
    int height=image.get_height();
    int index=0;
    float r, g, b;
    float thresh=.5;
    for (size_t h=0; h<height; h++) {
        for (size_t w=0; w<width; w++) {
            png::rgb_pixel rgb=image[h][w];
            index=w*height+h;
            
            r=((float)(rgb.red))/255;
            g=((float)(rgb.green))/255;
            b=((float)(rgb.blue))/255;
            
            ptr[index]=( r>thresh && g>thresh && b>thresh) ? 1.0f : 0.0f;
        }
    }
}

void writeImage( string direc, float* ptr, int width, int height, const char* name)
{
    string path=direc; path.append(name);
    png::image< png::rgb_pixel > image( width, height );
    int index=0;
    for (size_t h=0; h<height; h++) {
        for (size_t w=0; w<width; w++) {
            png::rgb_pixel rgb=image[h][w];
            index=w*height+h;            
            rgb.red  =(unsigned char)(roundf(255*ptr[3*index  ]));
            rgb.green=(unsigned char)(roundf(255*ptr[3*index+1]));
            rgb.blue =(unsigned char)(roundf(255*ptr[3*index+2]));
            image[h][w]=rgb;
        }
    }
    image.write(path.c_str());
}


bool comparator(std::pair<int,int> i, std::pair<int,int> j) {
	return (i.first<j.first);
}

float computeBoundingBox(PbrtPoint* Vertices, int nverts, PbrtPoint* bbox)
{
	int i=0;
	
	// allocate bbox outside
	bbox[0].x=INFINITY;
	bbox[0].y=INFINITY;
	bbox[0].z=INFINITY;
	bbox[1].x=-INFINITY;
	bbox[1].y=-INFINITY;
	bbox[1].z=-INFINITY;
	
	for (i=0; i<nverts; i++) {
		if (Vertices[i].x<bbox[0].x) bbox[0].x=Vertices[i].x;
		if (Vertices[i].y<bbox[0].y) bbox[0].y=Vertices[i].y;
		if (Vertices[i].z<bbox[0].z) bbox[0].z=Vertices[i].z;
		if (Vertices[i].x>bbox[1].x) bbox[1].x=Vertices[i].x;
		if (Vertices[i].y>bbox[1].y) bbox[1].y=Vertices[i].y;
		if (Vertices[i].z>bbox[1].z) bbox[1].z=Vertices[i].z;
	}
	float dx=bbox[1].x-bbox[0].x;
	float dy=bbox[1].y-bbox[0].y;
	float dz=bbox[1].z-bbox[0].z;
	return sqrt(dx*dx+dy*dy+dz*dz);
}

float estimateLightSphereRadius(PbrtPoint* Vertices, int nverts, float o2w[4][4])
{
	// assuming that the center of the sphere is at the camera center, set the radius of the sphere to be 4 times
	// as big as the distance from the camera center to the furthest point on the bounding box of the object
	// to give space for movement
	PbrtPoint* bbox=new PbrtPoint[2];
	
	computeBoundingBox(Vertices, nverts, bbox);
	
	Vector* boxvt=new Vector[8];
	int i=0;
	int ix, iy, iz;
	for (i=0; i<8; i++) {
		iz=i % 2; iy=(i/2) % 2; ix=((i/2)/2) % 2;
		boxvt[i].x=o2w[0][0]*Vertices[ix].x+o2w[0][1]*Vertices[iy].y+o2w[0][2]*Vertices[iz].z+o2w[0][3];
		boxvt[i].y=o2w[1][0]*Vertices[ix].x+o2w[1][1]*Vertices[iy].y+o2w[1][2]*Vertices[iz].z+o2w[1][3];
		boxvt[i].z=o2w[2][0]*Vertices[ix].x+o2w[2][1]*Vertices[iy].y+o2w[2][2]*Vertices[iz].z+o2w[2][3];
	}
	
	
	float maxd=0;
	float d=0;
	for (i=0; i<8; i++) {
		d=boxvt[i].x*boxvt[i].x+boxvt[i].y*boxvt[i].y+boxvt[i].z*boxvt[i].z;
		if (d>maxd) maxd=d;
	}
	
	delete[] boxvt;
	delete[] bbox;
    return 800;
	//return 3*sqrtf(maxd); // for patrik
	//return 10*sqrt(maxd); // for fruits
	//return 20*sqrt(maxd); // for pen
	
}



void computeFaceAdjacency(int* faces, int* Tfaces, int* fadjacencies, int* tadjacencies, int nt) {
	// for every face, there will be three adjacent faces
	//
	int j1, j2;
	int i, j, k;
	for (i=0; i<nt; i++) {
		for (j=0; j<3; j++) {
			if (j==0) { j1=1; j2=2; }
			else if (j==1) {j1=2; j2=0; }
			else if (j==2) {j1=0; j2=1; }
			for (k=0; k<nt; k++) {
				if (i!=k) {
					if (faces[3*i+j1]==faces[3*k] && faces[3*i+j2]==faces[3*k+1]) {
						tadjacencies[9*i+3*j+j1]=Tfaces[3*k]; tadjacencies[9*i+3*j+j2]=Tfaces[3*k+1]; tadjacencies[9*i+3*j+j]=Tfaces[3*k+2];
						if (fadjacencies) { fadjacencies[9*i+3*j+j1]=faces[3*k]; fadjacencies[9*i+3*j+j2]=faces[3*k+1]; fadjacencies[9*i+3*j+j]=faces[3*k+2]; }
						break;
					} else if (faces[3*i+j2]==faces[3*k] && faces[3*i+j1]==faces[3*k+1]) {
						tadjacencies[9*i+3*j+j2]=Tfaces[3*k]; tadjacencies[9*i+3*j+j1]=Tfaces[3*k+1]; tadjacencies[9*i+3*j+j]=Tfaces[3*k+2];
						if (fadjacencies) { fadjacencies[9*i+3*j+j2]=faces[3*k]; fadjacencies[9*i+3*j+j1]=faces[3*k+1]; fadjacencies[9*i+3*j+j]=faces[3*k+2]; }
						break;
					} else if (faces[3*i+j1]==faces[3*k+1] && faces[3*i+j2]==faces[3*k+2]) {
						tadjacencies[9*i+3*j+j1]=Tfaces[3*k+1]; tadjacencies[9*i+3*j+j2]=Tfaces[3*k+2]; tadjacencies[9*i+3*j+j]=Tfaces[3*k];
						if (fadjacencies) { fadjacencies[9*i+3*j+j1]=faces[3*k+1]; fadjacencies[9*i+3*j+j2]=faces[3*k+2]; fadjacencies[9*i+3*j+j]=faces[3*k]; }
						break;
					} else if (faces[3*i+j2]==faces[3*k+1] && faces[3*i+j1]==faces[3*k+2]) {
						tadjacencies[9*i+3*j+j2]=Tfaces[3*k+1]; tadjacencies[9*i+3*j+j1]=Tfaces[3*k+2]; tadjacencies[9*i+3*j+j]=Tfaces[3*k];
						if (fadjacencies) { fadjacencies[9*i+3*j+j2]=faces[3*k+1]; fadjacencies[9*i+3*j+j1]=faces[3*k+2]; fadjacencies[9*i+3*j+j]=faces[3*k]; }
						break;
					} else if (faces[3*i+j1]==faces[3*k+2] && faces[3*i+j2]==faces[3*k]) {
						tadjacencies[9*i+3*j+j1]=Tfaces[3*k+2]; tadjacencies[9*i+3*j+j2]=Tfaces[3*k]; tadjacencies[9*i+3*j+j]=Tfaces[3*k+1];
						if (fadjacencies) { fadjacencies[9*i+3*j+j1]=faces[3*k+2]; fadjacencies[9*i+3*j+j2]=faces[3*k]; fadjacencies[9*i+3*j+j]=faces[3*k+1]; }
						break;
					} else if (faces[3*i+j2]==faces[3*k+2] && faces[3*i+j1]==faces[3*k]) {
						tadjacencies[9*i+3*j+j2]=Tfaces[3*k+2]; tadjacencies[9*i+3*j+j1]=Tfaces[3*k]; tadjacencies[9*i+3*j+j]=Tfaces[3*k+1];
						if (fadjacencies) { fadjacencies[9*i+3*j+j2]=faces[3*k+2]; fadjacencies[9*i+3*j+j1]=faces[3*k]; fadjacencies[9*i+3*j+j]=faces[3*k+1]; }
						break;
					}
				}
			}
		}
	}
}

void determineSeams(int* present_iround, bool* isseam, int iround, int nrounds, int ntexpixel, int height)
{
	int xpix, ypix;
	
	// there is a seam if I'm seen and one of my 8-connected neighbors is not seen in the current set
	for (int i=0; i<ntexpixel; i++) {
		xpix=i/height; ypix=i%height;
		isseam[i]=(xpix>0 && xpix<ntexpixel/height-1 && ypix>0 && ypix<height-1 ) &&
		
		(present_iround[i]==1 || present_iround[i]==0) &&
		
		(present_iround[(xpix+1)*height+ypix]>1 || present_iround[(xpix-1)*height+ypix]>1 ||
		 present_iround[(xpix)*height+ypix+1]>1 || present_iround[(xpix)*height+ypix-1]>1 ||
		 present_iround[(xpix+1)*height+ypix+1]>1 || present_iround[(xpix-1)*height+ypix+1]>1 ||
		 present_iround[(xpix+1)*height+ypix-1]>1 || present_iround[(xpix-1)*height+ypix-1]>1);
	}
}


/*float smoothSeams(float* Itex, bool* isseam, int ntexpixel, int height) {
 int xpix, ypix;
 
 float* Itexsmoothed=new float[
 
 for (int i=0; i<ntexpixel; i++) {
 xpix=i/height; ypix=i%height;
 
 }
 }*/

float getWeight(float* x, float* mu, float oneover2sigmasq)
{
	float delta=(x[0]-mu[0])*(x[0]-mu[0])+(x[1]-mu[1])*(x[1]-mu[1])+(x[2]-mu[2])*(x[2]-mu[2]);
	return expf(-delta*oneover2sigmasq);
}

float getWeight(float d, float oneover2sigmasq) {
	return expf(-d*oneover2sigmasq);
}

float getDistanceSq(float* x, float* mu)
{
	return (x[0]-mu[0])*(x[0]-mu[0])+(x[1]-mu[1])*(x[1]-mu[1])+(x[2]-mu[2])*(x[2]-mu[2]);
}

void texturescale(PbrtPoint vdtex, int dmapheight, int dmapwidth, float* xout, float* yout ) {
    float x, y, x_, y_;
    x=vdtex.x; y=vdtex.y;
    x_=x*dmapwidth;y_=y*dmapheight;
    *xout=min(max(0.f,x_),(float)dmapwidth-2);
    *yout=min(max(0.f,y_),(float)dmapheight-2);
}


bool checkIntersection(float x, float y, float x1, float y1, float x2, float y2, float x3, float y3, float* bcs) {
	float denomm=(x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2);
	if (fabs(denomm)<1e-12)
		return false;
	float denomminv=1./denomm;
	float a=(x*y2 - x2*y - x*y3 + x3*y + x2*y3 - x3*y2)*denomminv;
	if (a<0. || a>1.)
		return false;
	float b=-(x*y1 - x1*y - x*y3 + x3*y + x1*y3 - x3*y1)*denomminv;
	if (b<0. || b>1.)
		return false;
	if (bcs) {bcs[0]=a; bcs[1]=b; bcs[2]=1.-a-b; }
	return true;
}

