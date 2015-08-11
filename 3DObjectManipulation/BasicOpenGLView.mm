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
#import "BasicOpenGLView.h"
#import "stdio.h"
#import "stdlib.h"
#import "time.h"
#import "math.h"
//#import "pbrtstarter.h"
#import "string.h"
//#import "engine.h"
#import "trace.h"
#import "helpers.h"
//#import "triangle.h"

// ==================================

GLfloat colormap[9][3]={{1.0f,0.0f,.0f},
	{0.0,1.0,0.0},{1.0,1.0,.0},{0.0,1.0,1.0},
	{1.0,.0,1.0},{1.0,1.0,1.0},{.5,.1,.5},
	{.1,.5,.5},{.5,.5,.1}};

GLfloat tol=.5;
GLfloat oneover2sigmasq=20;  // sigma=.05f

BOOL isXaxis, isYaxis, isZaxis;

GLfloat xAxis[3]={1.0f,0.0f,0.0f};
GLfloat yAxis[3]={0.0f,1.0f,0.0f};
GLfloat zAxis[3]={0.0f,0.0f,1.0f};
GLfloat xAxisObject[3]={1.0f,.0f,.0f};
GLfloat yAxisObject[3]={.0f,1.0f,0.0f};
GLfloat zAxisObject[3]={0.0f,0.0f,1.0f};

int hasColor;
int hasTexture;

// single set of interaction flags and states
GLint gDollyPanStartPoint[2] = {0, 0};
GLfloat gTrackBallRotation [4] = {0.0f, 0.0f, 0.0f, 0.0f};
GLboolean gDolly = GL_FALSE;
GLboolean gPan = GL_FALSE;
GLboolean gTrackball = GL_FALSE;
BasicOpenGLView * gTrackingViewInfo = NULL;

#pragma mark --- TODO ----
// Recenter the vertices in the object view
// maintain indices to vertices,

// or when you add vertices to list, add back the center

// pbrt: do the whole image, not just the ground mask

#pragma mark ---- OpenGL C drawing ----

static void drawSphere(GLfloat X, GLfloat Y, GLfloat Z, GLfloat sz, GLfloat cx, GLfloat cy, GLfloat cz)
{
	glColor3f(cx,cy,cz);
	GLUquadric *quad=gluNewQuadric();
	glTranslatef(X,Y,Z);
	gluSphere(quad, sz, 10, 10);
	glTranslatef(-X,-Y,-Z);
	gluDeleteQuadric(quad);
}

static void rodrigues(float* v, float* k, float theta, float* v_rot) {
    float kcrossv[3];
    vCrossProduct(k, v, kcrossv);
    float kdotv=vDotProduct(k, v);
    float ctheta=cosf(theta);
    float stheta=sinf(theta);
    float kdotvoneminusctheta=kdotv*(1.0-ctheta);
    for (int j=0; j<3; j++) {
        v_rot[j]=v[j]*ctheta+kcrossv[j]*stheta+k[j]*kdotvoneminusctheta;
    }
}

static void drawRotationAxes(GLfloat Xcentreo, GLfloat Ycentreo, GLfloat Zcentreo, GLfloat R)
{
	glPointSize(3);
	GLfloat stepsize=M_PI/1800.0;
	GLfloat angi;
	glBegin(GL_POINTS);
	glColor3f(.5,.5,.0);
	glVertex3f(Xcentreo, Ycentreo, Zcentreo);
	for (angi=0; angi<2*M_PI; angi+=stepsize)
	{
		glColor3f(0.0f, 1.0f, 1.0f);
		glVertex3f(Xcentreo, Ycentreo+R*cos(angi), Zcentreo+R*sin(angi));
		glColor3f(1.0f, 1.0f, 0.0f);
		glVertex3f(Xcentreo+R*sin(angi), Ycentreo, Zcentreo+R*cos(angi));
		glColor3f(1.0f, 0.0f, 1.0f);
		glVertex3f(Xcentreo+R*cos(angi), Ycentreo+R*sin(angi), Zcentreo);
		glColor3f(1.0f, 1.0f, 1.0f);
	}
	glEnd();
}

static void drawTranslationAxes(float Xcentreo, float Ycentreo, float Zcentreo, float* xaxis, float* yaxis, float* zaxis, float tscale)
{
    glLineWidth(3);
    glBegin(GL_LINES);
    glColor3f(0.0, 1.0, 1.0);
    glVertex3f(Xcentreo, Ycentreo, Zcentreo);
    glVertex3f(Xcentreo+tscale*xaxis[0], Ycentreo+tscale*xaxis[1], Zcentreo+tscale*xaxis[2]);
    glColor3f(1.0, .0, 1.0);
    glVertex3f(Xcentreo, Ycentreo, Zcentreo);
    glVertex3f(Xcentreo+tscale*yaxis[0], Ycentreo+tscale*yaxis[1], Zcentreo+tscale*yaxis[2]);
    glColor3f(1.0, 1.0, 0.0);
    glVertex3f(Xcentreo, Ycentreo, Zcentreo);
    glVertex3f(Xcentreo+tscale*zaxis[0], Ycentreo+tscale*zaxis[1], Zcentreo+tscale*zaxis[2]);
    glEnd();
}

static void cp(float* axis, float* startvec)
{
    // provide a start vector that is perpendicular to the axis
    float a1[3]={1,0,0};
    float a2[3]={0,1,0};
    float vd1=vDotProduct(axis, a1);
    float vd2=vDotProduct(axis, a2);
    if (vd1<vd2) {
        vCrossProduct(axis, a1, startvec);
    } else {
        vCrossProduct(axis, a2, startvec);
    }
    vNormalize(startvec);
    
}

static void drawRotationAxes(GLfloat Xcentreo, GLfloat Ycentreo, GLfloat Zcentreo, GLfloat* xaxis, GLfloat* yaxis, GLfloat* zaxis, GLfloat R)
{
	glPointSize(3);
	GLfloat stepsize=M_PI/1800.0;
	GLfloat angi;
	glBegin(GL_POINTS);
	glVertex3f(Xcentreo, Ycentreo, Zcentreo);
    float startvec[3];
    float endvec[3];
    for (int j=0; j<3; j++) {
        if (j==0) cp(xaxis,startvec);
        else if (j==1) cp(yaxis,startvec);
        else if (j==2) cp(zaxis,startvec);
        for (angi=0; angi<2*M_PI; angi+=stepsize) {
            glColor3f(1.0f-float( j==0 ), 1.0f-float( j==1 ), 1.0f-float( j==2 )); // xaxis is cyan, yaxis is magenta, zaxis is yellow
            if (j==0) rodrigues(startvec, xaxis, angi, endvec);
            else if (j==1) rodrigues(startvec, yaxis, angi, endvec);
            else if (j==2) rodrigues(startvec, zaxis, angi, endvec);
            glVertex3f(Xcentreo+R*endvec[0], Ycentreo+R*endvec[1], Zcentreo+R*endvec[2]);
        }
    }
	glEnd();
}

// ===================================
@implementation BasicOpenGLView

using namespace std;

- (void)dealloc
{
	//printf("Program end");
	//deleteArray(&arrayOfPoints);
	delete[] lassoPolygon;
	delete[] objectModel_faces;
	delete[] objectModel_vertices;
	delete[] objectModel_texcoords;
	delete[] objectModel_texfaces;
	delete[] objectModel_image;
	delete[] objectModel_image1;
    delete[] verticesInMotion;
    delete[] mapwidths;
    delete[] mapheights;
    
	//delete[] Vertices;
	//delete[] vertexNormals;
	//delete[] TextureVertices;
	delete[] objectProjections;
	delete[] objectCentroidProjections;
	delete[] clickPointToObjectProjDistances;
	delete[] clickPointToCentroidProjDistances;
    
    for (int j=0; j<nmaps; j++)
	{
		delete[] objectModel_teximages[j];
	}
	delete[] objectModel_teximages;
	if (groups) { delete[] groups; delete[] groupsLocked;}
    
    // arap
    delete[] rayconstrained_idxs;
    delete[] rigid_idxs;
    // delete[] anchored_idxs;
    // delete[] lassodeformed_idxs;
    delete[] islassodeformed;
    if (deformgroupsLocked) delete[] deformgroupsLocked;
    delete[] isanchored;
    delete[] arapRotations;
    delete[] Reflection;
    delete[] objectModel_vertexSol;
    delete[] neighboring_idxs;
    delete[] nneighbors;
    delete[] pts2Drayconstrained;
    delete[] pts2Drigid;
    delete[] lassomovelist;
    delete[] duplicatedVertices;
    delete[] duplicated;
    
    if (symmetric) delete[] symmetric;
    if (verticesMemory) delete[] verticesMemory;
    if (groups) delete[] groups;
    if (groupsLocked) delete[] groupsLocked;
    if (lassoPolygon) delete[] lassoPolygon;
    
	delete[] texbindings;
	[super dealloc];
}

# pragma mark ---- setup ----

-(int)readVertices;
{
	// do all the initiating, reading, opening, etc etc in here.
	// switch over to using pbrt instead of ModelData
	// switch over to strings instead of char*
    string path;
	string direcc=direc;
	direcc.append(name).append("/");
	int i, j, nt, nv, ntexv, ndivs, npixels, xmin, xmax, ymin, ymax, n_transforms, i1, i2, i3;
	float tgroundAxis[3];
	float x, y, z;
	int height;
	char* mapnameholder=new char[30];
	FILE* fid, *fidmap;
	//Load info
	path=direcc;
	fid=fopen(path.append("info.txt").c_str(), "r");
	if (!fid) { printf("Invalid info file!\n"); return -1; }
	fscanf(fid, "numtri=%d\nnumverts=%d\nnumtexverts=%d\nfocallength=%f\nsensorwidth=%f",
		   &nt,&nv,&ntexv,&focallength,&sensorwidth);
	
	nmaps=1;
	objectModel_teximages=new GLfloat*[nmaps];
	mapwidths=new GLint[nmaps];
	mapheights=new GLint[nmaps];
    readImageInfo(direcc, "texture.png", mapwidths[0], mapheights[0]);
    objectModel_teximages[0]=new GLfloat[ mapwidths[0]*mapheights[0]*3];
    readImageOpenGL(direcc, "texture.png", objectModel_teximages[0]);
    
	
	fclose(fid);
	
	nvperface=3;
	nfaces=nt;
	nvertices=nv;
	ntexturevertices=ntexv;
	
	hasColor=0;
	hasTexture=0;
	//Vector v1;
	
	objectModel_vertices=new GLfloat[3*nv];
	verticesInMotion=new GLfloat[3*nv];
	objectModel_texcoords=new GLfloat[2*ntexv];
	//TextureVertices=new PbrtPoint[ntexv];
	objectModel_faces=new GLint[nvperface*nt];
	objectModel_texfaces=new GLint[nvperface*nt];
	//vertexNormals=new Vector[nv];
	//Vertices=new PbrtPoint[nv];
	
	
	
	//Load vertices
	path=direcc;
	fid=fopen(path.append("verts_modified.txt").c_str(), "r");
	if (!fid) { fclose(fid); path=direcc; fid=fopen(path.append("verts_original.txt").c_str(), "r");	}
	if (!fid) { fclose(fid); path=direcc;  fid=fopen(path.append("verts.txt").c_str(), "r");	}
	if (!fid) { printf("Invalid vertices file!\n"); return -1; }
	for (i=0; i<nv; i++) {
		fscanf(fid, "%f %f %f\n", &x, &y, &z);
		objectModel_vertices[3*i]=x;
		objectModel_vertices[3*i+1]=y;
		objectModel_vertices[3*i+2]=z;
		//Vertices[i].x=x; Vertices[i].y=y; Vertices[i].z=z;
	}
	fclose(fid);
	
	// Load faces (assume triangles for now)
	path=direcc;
	fid=fopen(path.append("faces.txt").c_str(), "r");
	if (!fid) { printf("Invalid faces file!\n"); return -1; }
	for (i=0; i<nt; i++) {
		fscanf(fid, "%d %d %d\n", &i1, &i2, &i3);
		objectModel_faces[3*i]=i1;
		objectModel_faces[3*i+1]=i2;
		objectModel_faces[3*i+2]=i3;
	}
	fclose(fid);
	
	//Load texture vertices
	path=direcc;
	fid=fopen(path.append("textureverts.txt").c_str(), "r");
	if (!fid) { printf("Invalid texture vertices file!\n"); return -1; }
	for (i=0; i<ntexv; i++) {
		fscanf(fid, "%f %f\n", &x, &y);
		objectModel_texcoords[2*i]=x;
		objectModel_texcoords[2*i+1]=y;
		//TextureVertices[i].x=x*TEXW; TextureVertices[i].y=y*TEXH; TextureVertices[i].z=100.0f;
		//printf("%d, %d\n",i, objectModel_faces[1]);
	}
	fclose(fid);
	
	// Load texture faces
	path=direcc;
	fid=fopen(path.append("texturefaces.txt").c_str(), "r");
	if (!fid) { printf("Invalid texture faces file!\n"); return -1; }
	for (i=0; i<nt; i++) {
		fscanf(fid, "%d %d %d\n", &i1, &i2, &i3);
		objectModel_texfaces[3*i]=i1;
		objectModel_texfaces[3*i+1]=i2;
		objectModel_texfaces[3*i+2]=i3;
	}
	fclose(fid);
	
	// Setup normals
	int idx=0, idxprev=0, idxnext=0, jprev=0, jnext=0;
	
	/*Vector v1, v2, nn;
     for (i=0; i<nt; i++) {
     for (j=0; j<3; j++) {
     jprev=j-1; if (jprev<0) jprev=j+2;
     jnext=j+1; if (jnext>2) jnext=j-2;
     idx=3*i+j;
     idxprev=3*i+jprev;
     idxnext=3*i+jnext;
     v1=Vertices[objectModel_faces[idxnext]]-Vertices[objectModel_faces[idx]];
     v2=Vertices[objectModel_faces[idxprev]]-Vertices[objectModel_faces[idx]];
     nn=Normalize(Cross(v1, v2));
     vertexNormals[objectModel_faces[idx]]+=nn;
     }
     }
     for (i=0; i<nv; i++) vertexNormals[i]=Normalize(vertexNormals[i]);*/
	
	// open image
	path=direcc;
    
    int w, h;
    readImageInfo(direcc, "image.png", w, h);
    imwidth=(GLfloat)w; imheight=(GLfloat)h;
    reswidth=(int)(imwidth/2.0);
    resheight=(int)(imheight/2.0);
    npixels=imwidth*imheight;
    objectModel_image=new float[npixels*3];
	objectModel_image1=new float[npixels*3];
    objectModel_renderedImage=new float[reswidth*resheight*3];

    readImageOpenGL(direcc, "image.png", objectModel_image);
    readImageOpenGL(direcc, "background.png", objectModel_image1);
	uc=imwidth/2.0f;
	vc=imheight/2.0f;
	//fu=500.0f; fv=500.0f;
    fu=imwidth*focallength/sensorwidth;
    fv=fu;

    /*
	fid=fopen(path.append("image_inverted.dat").c_str(),"r");
	fread(objectModel_image,npixels*3,sizeof(float),fid);
	printf("Populated Image\n");
	fclose(fid);
    
	// open ground texture
	path=direcc;
	fid=fopen(path.append("groundtex_inverted.dat").c_str(), "r");
	fread(objectModel_image1,npixels*3,sizeof(float),fid);
	fclose(fid);
     */
	
	objectProjections=new GLfloat[nv*3];
	clickPointToObjectProjDistances=new GLfloat[nv];
	objectCentroidProjections=new GLfloat[nt*3];
	clickPointToCentroidProjDistances=new GLfloat[nt];
	
	return 0;
}

-(void) processArguments
{
	NSArray *arguments=[[NSProcessInfo processInfo] arguments];
	int argc=[arguments count];
	
	// defaults
	name=""; outputfilename="";
	direc="../examples/chair/";
	sigma=1.0f;
	filterhsize=3;
	filtertype=GAUSS_FILTER;
	nsamp=4;
	if (argc>1)
        direc=string([[arguments objectAtIndex:1] UTF8String]);
    
    /*
	if (argc>1) name=string([[arguments objectAtIndex:1] UTF8String]);
	if (argc>2) direc=string([[arguments objectAtIndex:2] UTF8String]);
	if (argc>3) outputfilename=string([[arguments objectAtIndex:3] UTF8String]);
	if (argc>4) nsamp=atoi([[arguments objectAtIndex:4] UTF8String]);
	if (argc>5) filtertype=atoi([[arguments objectAtIndex:5] UTF8String]);
	if (filtertype==GAUSS_FILTER) {
		if (argc>6) sigma=atof([[arguments objectAtIndex:6] UTF8String]);
		filterhsize=(int)round(3*sigma);
	} else if (filtertype==BOX_FILTER) {
		if (argc>6) filterhsize=atoi([[arguments objectAtIndex:6] UTF8String]);
		else filterhsize=3;
	}
     */
}

-(id) initWithFrame: (NSRect) frameRect
{
	// Program specific
	NSOpenGLPixelFormat * pf = [BasicOpenGLView basicPixelFormat];
	self = [super initWithFrame: frameRect pixelFormat: pf];
    ////printf("my view number is %d\n",openglViewNumber);
	NSRect brect=[self frame];
	//printf("%f %f %f %f\n",brect.origin.x, brect.origin.y, brect.size.width, brect.size.height);
	if ((brect.origin.x)<100) openglViewNumber=2;
	else openglViewNumber=1;
	resetToIdentity(trackTranslation);
	resetToIdentity(trackBallRotation);
	
	if (openglViewNumber==1) {
		glClearColor(.7f, .7f, .7f, 1.0f);
		theOperationMode=MODE_ROTATE;
	}
	else if (openglViewNumber==2) {
		glClearColor(.7f, .7f, .7f, 1.0f);
		theOperationMode=MODE_TRANSLATE;
	}
	
	[[self openGLContext] makeCurrentContext];
	
	[self initObject];
    for (int i=0; i<500; i++) {
        writeX[i]=.0f;
        writeY[i]=.0f;
        writeZ[i]=.0f;
    }
    nwrite=0;
    doWrite=false;
    
	/*vanilla = [[Shader alloc] initWithShadersInAppBundle:@"Vanilla"];
	 chocolate = [[Shader alloc] initWithShadersInAppBundle:@"Chocolate"];
	 if( vanilla ) vanillaProgramObject = [vanilla programObject];
	 if (chocolate) chocolateProgramObject=[chocolate programObject];*/
    
	return self;
}

- (void) initObject {
    
    doDrawMesh=true;
    n_previous_primitives=0;
    
	// Object specific
	[self processArguments];
	hasColor=0;
	hasTexture=0;
	[self readVertices];
	/*
     string path=direc;
     path.append(name).append("/transforms_demo1.dat");
     FILE* fid=fopen(path.c_str(), "r");
     path=direc;
     path.append(name).append("/initTransformReal.dat");
     FILE* fout=fopen(path.c_str(), "w");
     
     float f;
     for (int i=0; i<4; i++) {
     for (int j=0; j<4; j++) {
     fread(&f,1,sizeof(float),fid);
     fwrite(&f,1,sizeof(float),fout);
     }
     }
     fclose(fid);
     fclose(fout);
     */
	anchorWeight=1.0f;
	symmetryWeight=1.0f;
	deformationWeight=.0f;
	rayConstraintsWeight=1.0f;
    symmetryConstraintsWeight=.01f; // not used
    planarityWeight=4.0f;
    laplacianWeight=.1f;
    augweight=.5f;
    numiterations=20;
    
	arapRotations=new GLfloat[9*nvertices];
	Reflection=new GLfloat[12];
	objectModel_vertexSol=new GLfloat[3*nvertices];
	neighboring_idxs=new int[N_CONNMAX*nvertices];
    for (int i=0; i<N_CONNMAX*nvertices; i++) {
        neighboring_idxs[i]=-1;
    }
	nneighbors=new int[nvertices];
	pts2Drayconstrained=new GLfloat[2*50];
	rayconstrained_idxs=new int[50]; nrayconstrained=0;
	//anchored_idxs=new int[nvertices*10]; nanchored=0;
    isanchored=new bool[nvertices];
    for (int i=0; i<nvertices; i++) isanchored[i]=false;
	//lassodeformed_idxs=new int[nvertices*10]; nlassodeformed=0;
    islassodeformed=new bool[nvertices];
    for (int i=0; i<nvertices; i++) islassodeformed[i]=false;
    rigid_idxs=new int[nvertices*2]; nrigid=0;
	pts2Drigid=new GLfloat[2*nvertices];
    drawMeshOnly=YES;
    smoothBetweenRigidMotions=NO;
    renderIsDone=NO;
    numrender=0;
	
    lassomovelist=new float[40*nvertices];
    nlassomovelist=0;
    nlassomovelistprevert=0;
    
	if (openglViewNumber==2) {
		verticesMemory=new GLfloat[3*nvertices*40];
	} else {
        //		verticesMemory=0;
		verticesMemory=new GLfloat[3*nvertices*40];
	}
	nverticesmemory=0;  nstores=0;
	
	toggleState=true;
	
	string path1=direc; path1.append(name).append("/groups.dat");
	FILE* fid1=fopen(path1.c_str(),"r");
	if (fid1 != NULL) {
		groups=new int[nvertices];
		fread(groups,nvertices*sizeof(int),1,fid1);
        ngroups=0;
        for (int i=0; i<nvertices; i++) {
            ngroups=groups[i]>ngroups ? groups[i] : ngroups;
        }
		
        ngroups=groups[nvertices-1];
        //ngroups=3;
		groupsLocked=new bool[ngroups];
        deformgroupsLocked=new bool[ngroups];
		for (int j=0; j<ngroups; j++) {
			groupsLocked[j]=false;
            deformgroupsLocked[j]=false;
		}
        numComponentsSelected=0;
        isComponentSelected=new bool[ngroups];
        isVertexSelected=new bool[nvertices];
        for (int j=0; j<ngroups; j++) {
            isComponentSelected[j]=false;
        }
        for (int j=0; j<nvertices; j++) {
            isVertexSelected[j]=false;
        }
        selectionAxesArePCs=false;
        resetToIdentity(switcherMatrix);
        
	} else {
		groupsLocked=NULL;
        deformgroupsLocked=NULL;
		groups=NULL;
	}
	
	path1=direc; path1.append(name).append("/symmetric.dat");
	fid1=fopen(path1.c_str(),"r");
	if (fid1 != NULL) {
		symmetric=new int[nvertices];
		fread(symmetric,nvertices*sizeof(int),1,fid1); fclose(fid1);
	} else {
		symmetric=NULL;
	}
    
    path1=direc; path1.append(name).append("/coplanar_idx.dat");
    fid1=fopen(path1.c_str(),"r");
    if (fid1 !=NULL) {
        planar=new int[nvertices];
        fread(planar,nvertices*sizeof(int),1,fid1); fclose(fid1);
        path1=direc; path1.append(name).append("/plane_modified.dat");
        fid1=fopen(path1.c_str(),"r");
        if (!fid1) {
            path1=direc;path1.append(name).append("/plane.dat");
            fid1=fopen(path1.c_str(),"r");
        }
        fread(plane, 4, sizeof(float), fid1);
        printf("plane=[%f,%f,%f,%f]\n",plane[0],plane[1],plane[2],plane[3]);
        fclose(fid1);
    } else {
        planar=NULL;
    }
	
	displayGroupNum=0;
    displayGroupNum=0;
	toggleDeformUpState=YES;
	toggleDeformDownState=YES;
    toggleRigidDownState=YES;
	
    duplicatedVertices=new GLfloat[3*nvertices*100];
    duplicated=new bool[10000];
    for (int k=0; k<10000; k++) {
        duplicated[k]=false;
    }
	nduplicated=0;
	
	//initArray(&arrayOfPoints);
	lassoPolygon=new GLfloat[3000]; nlassopolygon=0;
	initArray(&clickedPoints);
	closeLoop=NO;
	isInitObjectProj=NO;
	issetInitTransform=NO;
	isBeingSaved=NO;
	justMadeEdit=NO;
	inpainted=0;
	ngroundpts=0;
	normalAxis[0]=0.0f; normalAxis[1]=0.0f; normalAxis[2]=0.0f;
	isgroundaxis=NO; isgriddr1=NO; isgriddr2=NO;
	hmark=0;
    
	manipPhase=NO;
	doZoom=NO;
    alllocked=NO;
	n_transforms=0;
	
	int i=0, j=0;
	for (i=0; i<nvertices; i++) {
		for (j=0; j<9; j++) {
			if (j==0 || j==8 || j==4) arapRotations[9*i+j]=1;
			else arapRotations[9*i+j]=0;
		}
	}
	[self computeMaxMinMean];
	R=rot_ax_val;
	Xc=Xcentre; Yc=Ycentre; Zc=Zcentre;
	Xcentreo=Xcentre; Ycentreo=Ycentre; Zcentreo=Zcentre;
	
	NSRect brect=[self frame];
	vport[0]=0; vport[1]=0;
	rvport[0]=0; rvport[1]=0;
	if (openglViewNumber==1) {
		vport[2]=brect.size.width; vport[3]=brect.size.height;
		rvport[2]=brect.size.width; rvport[3]=brect.size.height;
	} else {
		vport[2]=imwidth; vport[3]=imheight;
		float wratio=((float)(brect.size.width))/imwidth;
		float hratio=((float)(brect.size.height))/imheight;
		if (wratio<hratio) {
			rvport[2]=brect.size.width; rvport[3]=wratio*imheight;
			rvport[0]=0; rvport[1]=(brect.size.height-rvport[3])/2.0f;
		} else {
			rvport[2]=hratio*imwidth; rvport[3]=brect.size.height;
			rvport[1]=0; rvport[0]=(brect.size.width-rvport[2])/2.0f;
		}
	}
	
	[self initTextures];

    
	/*
	if (openglViewNumber==2) {
        if (!name.compare("chair") || !name.compare("patrikv3") || !name.compare("patrikv3") || !name.compare("patrikv3_srground1")) {
            fu=2*uc*14.0f/36.0f;
        } else if (!name.compare("lumixchair")) {
            fu=2*uc*5.1f/8.07f;
        } else if (!name.compare("banana")) {
            fu=3900.00;
        } else if (!name.compare("patrik1")) {
            fu=1294.275146;
        } else if (!name.compare("mango")) {
            fu=3900.00;
        } else if (!name.compare("tophat")) {
            fu=2*uc*20.88/5.744;
        } else if (!name.compare("crane")) {
            fu=2*uc*5.1f/8.07f;
        } else if (!name.compare("taxi") || !name.compare("taxi_imac")) {
            fu=7322.578982/2;
        } else if (!name.compare("watch")) {
            fu=2*uc*28.0f/36.0f;
        } else if (!name.compare("cliffcar")) {
            fu=2*uc*5.81f/4.308f;
        } else if (!name.compare("apple")) {
            fu=2*uc*24.0f/36.0f;
        } else if (!name.compare("appleproposal")) {
            fu=622.3640;
        } else if (!name.compare("person")) {
            fu=2*uc*6.6/5.744;
        } else if (!name.compare("sofa") || !name.compare("hand")) {
            fu=2*uc*23/22.3;
        }
        fv=fu;
        
        if (!name.compare("tajmahal")) {
            //fu=1029.211011;
            //fv=1106.007930;
            //uc=536.658187;
            //vc=627.137505;
            fu=1021.317686;
            fv=1100.607632;
        }
        
        if (!name.compare("avenger") || !name.compare("avengersaligned")) {
            fu=1819.055176;
            fv=1193.121925;
            //            uc=652.172537;
            //            vc=-507.240034;
        }
        if (!name.compare("tulip")) {
            fu=463.635151;
            fv=372.541871;
        }
	}
     */
    
    
    if (openglViewNumber==2) {
        [txtAnchorWeight setValue:[NSString stringWithFormat:@"%6.5f",anchorWeight]];
        [txtSymmetryWeight setValue:[NSString stringWithFormat:@"%6.5f",symmetryWeight]];
        [txtDeformationWeight setValue:[NSString stringWithFormat:@"%6.5f",deformationWeight]];
        [txtRayConstraintsWeight setValue:[NSString stringWithFormat:@"%6.5f",rayConstraintsWeight]];
        [txtNumIterations setValue:[NSString stringWithFormat:@"%d",numiterations]];
        [txtPlanarityWeight setValue:[NSString stringWithFormat:@"%6.5f",planarityWeight]];
        [txtAugWeight setValue:[NSString stringWithFormat:@"%6.5f",augweight]];
    }
    
    time(&date1);
    
}

// pixel format definition
+ (NSOpenGLPixelFormat*) basicPixelFormat
{
    NSOpenGLPixelFormatAttribute attributes [] = {
        NSOpenGLPFAWindow,
        NSOpenGLPFADoubleBuffer,	// double buffered
        NSOpenGLPFADepthSize, (NSOpenGLPixelFormatAttribute)16, // 16 bit depth buffer
		NSOpenGLPFAAccumSize, (NSOpenGLPixelFormatAttribute)64,
		NSOpenGLPFAMultisample,
		NSOpenGLPFASampleBuffers, (NSOpenGLPixelFormatAttribute)1,
		NSOpenGLPFASamples, (NSOpenGLPixelFormatAttribute)4,
        (NSOpenGLPixelFormatAttribute)nil
    };
    return [[[NSOpenGLPixelFormat alloc] initWithAttributes:attributes] autorelease];
}

// ---------------------------------

// set initial OpenGL state (current context is set)
// called after context is created

- (void) prepareOpenGL
{
    long swapInt = 1;
    [[self openGLContext] setValues:(const GLint*)&swapInt forParameter:NSOpenGLCPSwapInterval]; // set to vbl sync
	// init GL stuff here
	glEnable(GL_DEPTH_TEST);
	
	glShadeModel(GL_SMOOTH);
	glEnable(GL_CULL_FACE);
	glFrontFace(GL_CCW);
	glPolygonOffset (1.0f, 1.0f);
	
    [self.window setBackgroundColor:[NSColor colorWithDeviceRed:.2 green:.2 blue:.2 alpha:1.0]];
	glClearColor(.2, .2f, .2f, 1.0f);
	//	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	
	resetToIdentity(objectRotation);
    /*if (openglViewNumber==1) {
     objectRotation[0]=.0f; objectRotation[1]=1.0f;
     objectRotation[4]=-1.0f; objectRotation[5]=0.0f;
     }*/
	resetToIdentity(trackBallRotation);
	resetToIdentity(initTransform);
	isXaxis=NO; isYaxis=NO; isZaxis=NO;
	shapeSize = 7.0f; // max radius of of objects
}

- (void)computeMaxMinMean
{
	int i=0;
	Xcentre=0; Ycentre=0; Zcentre=0;
	Xmin=INFINITY; Ymin=INFINITY; Zmin=INFINITY;
	Xmax=-INFINITY; Ymax=-INFINITY; Zmax=-INFINITY;
	
	for (i=0; i<nvertices; i++) {
		GLfloat XX=objectModel_vertices[3*i], YY=objectModel_vertices[3*i+1], ZZ=objectModel_vertices[3*i+2];
		Xcentre+=XX;Ycentre+=YY;Zcentre+=ZZ;
		if (Xmin>XX) Xmin=XX; if (Xmax<XX) Xmax=XX;
		if (Ymin>YY) Ymin=YY; if (Ymax<YY) Ymax=YY;
		if (Zmin>ZZ) Zmin=ZZ; if (Zmax<ZZ) Zmax=ZZ;
	}
	Mmax=Xmax; Mmin=Xmin;
	if (Mmax<Ymax) Mmax=Ymax; if (Mmax<Zmax) Mmax=Zmax;
	if (Mmin>Ymin) Mmin=Ymin; if (Mmin>Zmin) Mmin=Zmin;
	
	Xcentre/=nvertices; Ycentre/=nvertices; Zcentre/=nvertices;
    BoxDiag=sqrtf((Xmax-Xmin)*(Xmax-Xmin)+(Ymax-Ymin)*(Ymax-Ymin)+(Zmax-Zmin)*(Zmax-Zmin));
    r_sphere_val=R_SPHERE_RATIO*BoxDiag;
    rot_ax_val=RADIUS_RATIO*BoxDiag;
    trans_ax_val=T_SCALE_RATIO*BoxDiag;
    join_thresh_val=JOINTHRESH_RATIO*BoxDiag;
    
}

-(void)initTextures
{
    /*
	glActiveTexture(GL_TEXTURE0);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	
	glGenTextures(1, &backgroundTex);
	glBindTexture(GL_TEXTURE_2D,backgroundTex);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, (int)(imwidth), (int)(imheight), 0, GL_RGB, GL_FLOAT, objectModel_image);
	
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    
	glActiveTexture(GL_TEXTURE1);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	
	glGenTextures(1, &backgroundTexInpainted);
	glBindTexture(GL_TEXTURE_2D,backgroundTexInpainted);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, (int)(imwidth), (int)(imheight), 0, GL_RGB, GL_FLOAT, objectModel_image1);
	
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	
     */
    
	//texbindings=new GLuint[nmaps];
    /*
	for (int i=0; i<nmaps; i++) {
		if (i==0) glActiveTexture(GL_TEXTURE2);
		else if (i==1) glActiveTexture(GL_TEXTURE3);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		
		glGenTextures(1, &texbindings[i]);
		glBindTexture(GL_TEXTURE_2D,texbindings[i]);
		glTexImage2D(GL_TEXTURE_2D, 0, 3, (int)(mapwidths[i]), (int)(mapheights[i]), 0, GL_RGB, GL_FLOAT, objectModel_teximages[i]);
		
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	}
     */
}


#pragma mark ---- OpenGL ObjectiveC drawing ----

-(void) drawGrid
{
	if (openglViewNumber==2 && issetInitTransform) {
		glColor3f(.0f, .0f, .0f);
		
		GLfloat scale=1.75;
		int ngrid=21;
		
		GLfloat X1, Y1, Z1;
		GLfloat X2, Y2, Z2;
		GLfloat X3, Y3, Z3;
		GLfloat X4, Y4, Z4;
		X1=gridCentre[0]+scale*R*gridDr1[0]+scale*R*gridDr2[0];
		Y1=gridCentre[1]+scale*R*gridDr1[1]+scale*R*gridDr2[1];
		Z1=gridCentre[2]+scale*R*gridDr1[2]+scale*R*gridDr2[2];
		
		X2=gridCentre[0]+scale*R*gridDr1[0]-scale*R*gridDr2[0];
		Y2=gridCentre[1]+scale*R*gridDr1[1]-scale*R*gridDr2[1];
		Z2=gridCentre[2]+scale*R*gridDr1[2]-scale*R*gridDr2[2];
		
		X3=gridCentre[0]-scale*R*gridDr1[0]-scale*R*gridDr2[0];
		Y3=gridCentre[1]-scale*R*gridDr1[1]-scale*R*gridDr2[1];
		Z3=gridCentre[2]-scale*R*gridDr1[2]-scale*R*gridDr2[2];
		
		X4=gridCentre[0]-scale*R*gridDr1[0]+scale*R*gridDr2[0];
		Y4=gridCentre[1]-scale*R*gridDr1[1]+scale*R*gridDr2[1];
		Z4=gridCentre[2]-scale*R*gridDr1[2]+scale*R*gridDr2[2];
		
		int i=0;
		for (i=0; i<ngrid; i++) {
			GLfloat alpha=i/(ngrid-1.0f);
			
			if (fabs(alpha-.5)<1e-3) glColor3f(.0f,.0f,.0f); else glColor3f(.8f, .8f, .8f);
			
			glBegin(GL_LINES);
			glVertex3f((1-alpha)*X1+alpha*X2, (1-alpha)*Y1+alpha*Y2, (1-alpha)*Z1+alpha*Z2);
			glVertex3f((1-alpha)*X4+alpha*X3, (1-alpha)*Y4+alpha*Y3, (1-alpha)*Z4+alpha*Z3);
			glEnd();
			
			glBegin(GL_LINES);
			glVertex3f((1-alpha)*X1+alpha*X4, (1-alpha)*Y1+alpha*Y4, (1-alpha)*Z1+alpha*Z4);
			glVertex3f((1-alpha)*X2+alpha*X3, (1-alpha)*Y2+alpha*Y3, (1-alpha)*Z2+alpha*Z3);
			glEnd();
			
		}
		
		glBegin(GL_LINES);
		
		glVertex3fv(gridCentre);
		glVertex3f(gridCentre[0]+10*gridAxis[0], gridCentre[1]+10*gridAxis[1], gridCentre[2]+10*gridAxis[2]);
		
		glEnd();
	}
}

-(void) drawObjectModel
{
    long i, j;
	
	glPointSize(3);
	///*
	
    GLuint mytex;
    glGenTextures(1, &mytex);
    glBindTexture(GL_TEXTURE_2D, mytex);
    
	glTexImage2D(GL_TEXTURE_2D, 0, 3, (int)(mapwidths[0]), (int)(mapheights[0]), 0, GL_RGB, GL_FLOAT, objectModel_teximages[0]);
    
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    
    glShadeModel(GL_SMOOTH);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, mytex);
    
    
	for (i=0; i<nfaces; i++) {
		
		glBegin(GL_POLYGON);
		for (j=0; j<nvperface; j++)
		{
			long vidx=(long)objectModel_faces[nvperface*i+j];
			glColor4f(1.0,1.0,1.0,1.0);
			{
				if (true) {
					long tidx=(long)objectModel_texfaces[nvperface*i+j]; // replace with color
					glColor4f(1.0f, 1.0f, 1.0f,1.0f);
					glTexCoord2f(objectModel_texcoords[2*tidx], 1.0f-objectModel_texcoords[2*tidx+1]);  // replace with color
				}
				if (hasColor)
				{
					glColor4f(objectModel_diffuseColor[i],objectModel_diffuseColor[i+nfaces],objectModel_diffuseColor[i+2*nfaces],1.0f);
					//glColor3f(.8,.8,.8);
				}
			}
			//glColor3f(.8,.8,.8);
			glVertex3f(objectModel_vertices[3*vidx], objectModel_vertices[3*vidx+1], objectModel_vertices[3*vidx+2]);
		}
		glEnd();
	}
	glDisable(GL_TEXTURE_2D);
    glDeleteTextures(1, &mytex);
	
	glColor4f(.0f, .0f, .0f,1.0f);
//	glUseProgramObjectARB(NULL);
	
	
	for (i=0; i<nfaces; i++) {
		glLineWidth(1.5);
		glBegin(GL_LINE_LOOP);
		for (j=0; j<nvperface; j++)
		{
			long vidx=(long)objectModel_faces[nvperface*i+j];
			glColor3f(.9f,.9f,.9f);
			glVertex3f(objectModel_vertices[3*vidx], objectModel_vertices[3*vidx+1], objectModel_vertices[3*vidx+2]);
		}
		glEnd();
	}
	
}

-(void) drawObjectModelTextured
{
	int itex;
	long i, j;
	
	glPointSize(3);
	///*
	
	
    
    glEnable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE2);
    //glShadeModel(GL_FLAT);
    glShadeModel(GL_SMOOTH);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    glBindTexture(GL_TEXTURE_2D, texbindings[0]);
    
    for (i=0; i<nfaces; i++) {
        glBegin(GL_POLYGON);
        for (j=0; j<nvperface; j++)
        {
            long vidx=(long)objectModel_faces[nvperface*i+j];
            long tidx=(long)objectModel_texfaces[nvperface*i+j]; // replace with color
            if (!groups || displayGroupNum==0 || displayGroupNum==groups[vidx]) {
                glColor4f(0.0f, 1.0f, 1.0f,1.0f);
                glTexCoord2f(objectModel_texcoords[2*tidx], 1.0f-objectModel_texcoords[2*tidx+1]);  // replace with color
                glVertex3f(objectModel_vertices[3*vidx], objectModel_vertices[3*vidx+1], objectModel_vertices[3*vidx+2]);
            }
        }
        glEnd();
        
    }
    
	glDisable(GL_TEXTURE_2D);
//	glUseProgramObjectARB(NULL);
	
	
	glLineWidth(1.5);
	for (i=0; i<nfaces; i++) {
		glLineWidth(1.5);
		glBegin(GL_LINE_LOOP);
		for (j=0; j<nvperface; j++)
		{
			long vidx=(long)objectModel_faces[nvperface*i+j];
			if (!groups || displayGroupNum==0 || displayGroupNum==groups[vidx]) {
				glColor3f(.0f,.0f,.0f);
				glVertex3f(objectModel_vertices[3*vidx], objectModel_vertices[3*vidx+1], objectModel_vertices[3*vidx+2]);
			}
		}
		
		glEnd();
	}
    
	
	glColor4f(.0f, .0f, .0f,1.0f);
	
    
}

- (void) drawDuplicatedObjects
{
    int itex;
	long i, j;
	
	glPointSize(3);
    
    GLuint mytex;
    glGenTextures(1, &mytex);
    glBindTexture(GL_TEXTURE_2D, mytex);
    
	//glActiveTexture(GL_TEXTURE2);
    //glShadeModel(GL_FLAT);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, (int)(mapwidths[0]), (int)(mapheights[0]), 0, GL_RGB, GL_FLOAT, objectModel_teximages[0]);
    
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    
    glShadeModel(GL_SMOOTH);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, mytex);
    
    for (int k=0; k<nduplicated; k++) {
        
        for (i=0; i<nfaces; i++) {
            glBegin(GL_POLYGON);
            for (j=0; j<nvperface; j++)
            {
                long vidx=(long)objectModel_faces[nvperface*i+j];
                long tidx=(long)objectModel_texfaces[nvperface*i+j]; // replace with color
                if (!groups || displayGroupNum==0 || displayGroupNum==groups[vidx]) {
                    glColor4f(0.0f, 1.0f, 1.0f,1.0f);
                    glTexCoord2f(objectModel_texcoords[2*tidx], 1.0f-objectModel_texcoords[2*tidx+1]);  // replace with color
                    glVertex3f(duplicatedVertices[3*nvertices*k+3*vidx], duplicatedVertices[3*nvertices*k+3*vidx+1], duplicatedVertices[3*nvertices*k+3*vidx+2]);
                }
            }
            glEnd();
            
        }
    }
    
	glDisable(GL_TEXTURE_2D);
    glDeleteTextures(1, &mytex);
//	glUseProgramObjectARB(NULL);
	
		
	glColor4f(.0f, .0f, .0f,1.0f);
    //	glUseProgramObjectARB(NULL);
	
    for (int k=0; k<nduplicated; k++) {
        for (i=0; i<nfaces; i++) {
            glLineWidth(1.5);
            glBegin(GL_LINE_LOOP);
            for (j=0; j<nvperface; j++)
            {
                long vidx=(long)objectModel_faces[nvperface*i+j];
                glColor3f(.9f,.9f,.9f);
                glVertex3f(duplicatedVertices[3*nvertices*k+3*vidx], duplicatedVertices[3*nvertices*k+3*vidx+1], duplicatedVertices[3*nvertices*k+3*vidx+2]);
            }
            glEnd();
        }
        
    }
}

-(void) drawObjectModelMesh
{
    //	glUseProgramObjectARB(vanillaProgramObject);
	
	long i, j;
	
	glPointSize(3);
	
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_FASTEST);
	glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_FASTEST);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
	
	glDisable(GL_TEXTURE_2D);
	glShadeModel(GL_FLAT);
    
    if (theOperationMode==MODE_LASSODEFORM) {
        //printf("here\n");
    }
	
	for (i=0; i<nfaces; i++) {
		glLineWidth(1.5);
		glBegin(GL_LINE_LOOP);
		for (j=0; j<nvperface; j++)
		{
			long vidx=(long)objectModel_faces[nvperface*i+j];
            
            if (theOperationMode==MODE_LASSODEFORM) {
                if (!groups || displayGroupNum==0 || displayGroupNum==groups[vidx]) {
                    if (isVertexSelected[vidx]) glColor3f(0.0f,0.2f,1.0f);
                    else glColor3f(1.0f,1.0f,1.0f);
                    glVertex3f(objectModel_vertices[3*vidx], objectModel_vertices[3*vidx+1], objectModel_vertices[3*vidx+2]);
                }
            } else {
                if (!groups || (displayGroupNum==0 || displayGroupNum==groups[vidx])) {
                    if (isVertexSelected[vidx]) glColor3f(0.0f,0.2f,1.0f);
                    else glColor3f(1.0f,1.0f,1.0f);
                    glVertex3f(objectModel_vertices[3*vidx], objectModel_vertices[3*vidx+1], objectModel_vertices[3*vidx+2]);
                }
			}
		}
		
		glEnd();
	}
	glDisable(GL_TEXTURE_2D);
	
//	glUseProgramObjectARB(NULL);
	
}

-(void)renderLassoStrip
{
	GLfloat f=1000.0f, n=10.0f;
	GLfloat a=-(f+n);
	GLfloat b=-2.0*f*n;
	GLfloat c=f-n;
	GLfloat m_bp[16];
	GLfloat m_bm[16]={1.0,0.0,0.0,0.0,
		0.0,1.0,0.0,0.0,
		0.0,0.0,-1.0,0.0,
		0.0,0.0,0.0,1.0};
	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	int j=0; for (j=0; j<16; j++) m_bp[j]=0.0f;
	m_bp[0]=1.0f/(imwidth*.5);
	m_bp[5]=1.0f/(imheight*.5);
	
	float lassoscale=20.0f;
	m_bp[12]=-lassoscale; m_bp[13]=-lassoscale;
	m_bp[10]=a/c; m_bp[11]=-1.0; m_bp[14]=b/c;
	glMultMatrixf(m_bp);
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glMultMatrixf(m_bm);
	
	
	if ( (theOperationMode==MODE_LASSO || theOperationMode==MODE_LASSODEFORM) && nlassopolygon >= 2)
	{
		glColor3f(1.0f, 0.0f, 1.0f);
		glPointSize(5);
		glLineWidth(3);
		glDisable(GL_TEXTURE_2D);
		
		GLenum mymode=GL_LINE_STRIP;
		
		if (closeLoop)
        {  mymode=GL_LINE_LOOP;
        } else {
            glBegin(mymode);
            for (j=0; j<nlassopolygon; j++)
            {
                //			GLfloat x, y, z;
                //			getElementFromArray(&arrayOfPoints, j, &x, &y, &z);
                if (openglViewNumber==2) { glVertex3f(lassoPolygon[2*j]*lassoscale,(vport[3]-lassoPolygon[2*j+1])*lassoscale,lassoscale); }
                else { glVertex3f(lassoPolygon[2*j]*lassoscale,lassoPolygon[2*j+1]*lassoscale,lassoscale); }
            }
            glEnd();
            
            glLineWidth(1);
        }
    } else if (theOperationMode==MODE_ZOOM && openglViewNumber==2) {
		if (doZoom) {
			glColor3f(0.0f, 1.0f, 1.0f);
			glLineWidth(2);
			glBegin(GL_LINE_LOOP);
			GLfloat x1v, y1v, x2v, y2v;
			[self viewportTransformX:x1 Y:y1 toX:&x1v toY:&y1v];
			[self viewportTransformX:x2 Y:y2 toX:&x2v toY:&y2v];
			glVertex3f(x1v*lassoscale, (vport[3]-y1v)*lassoscale, lassoscale);
			glVertex3f(x1v*lassoscale, (vport[3]-y2v)*lassoscale, lassoscale);
			glVertex3f(x2v*lassoscale, (vport[3]-y2v)*lassoscale, lassoscale);
			glVertex3f(x2v*lassoscale, (vport[3]-y1v)*lassoscale, lassoscale);
			glEnd();
			glLineWidth(1);
		}
	} else if (theOperationMode==MODE_MARKDEFORM) {
		glColor3f(1.0f, 1.0f, 1.0f);
		glPointSize(5);
		glLineWidth(1);
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_TEXTURE_2D);
		
		glBegin(GL_LINES);
		glVertex3f(dlinex*lassoscale, (vport[3]-dliney)*lassoscale, lassoscale);
		glVertex3f(pts2Drayconstrained[2*nrayconstrained-2]*lassoscale, (vport[3]-pts2Drayconstrained[2*nrayconstrained-1])*lassoscale, lassoscale);
		glEnd();
		
		glColor3f(1.0, .0, .0);
		glBegin(GL_POINTS);
		glVertex3f(dlinex*lassoscale, (vport[3]-dliney)*lassoscale, lassoscale);
		glVertex3f(pts2Drayconstrained[2*nrayconstrained-2]*lassoscale, (vport[3]-pts2Drayconstrained[2*nrayconstrained-1])*lassoscale, lassoscale);
		glEnd();
	} else if (theOperationMode==MODE_MARKRIGID) {
		glPointSize(5);
		glLineWidth(1);
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_TEXTURE_2D);
        
        for (int j=0; j<nrigid; j++) {
            glColor3f(1.0, 0.0, 1.0);
            glBegin(GL_POINTS);
            glVertex3f(pts2Drigid[2*j]*lassoscale, (vport[3]-pts2Drigid[2*j+1])*lassoscale, lassoscale);
            glEnd();
        }
		
		glColor3f(1.0f, 1.0f, 1.0f);
		glBegin(GL_LINES);
		glVertex3f(dlinex*lassoscale, (vport[3]-dliney)*lassoscale, lassoscale);
		glVertex3f(pts2Drigid[2*nrigid-2]*lassoscale, (vport[3]-pts2Drigid[2*nrigid-1])*lassoscale, lassoscale);
		glEnd();
		
		glColor3f(0.0, 1.0, 1.0);
		glBegin(GL_POINTS);
		glVertex3f(dlinex*lassoscale, (vport[3]-dliney)*lassoscale, lassoscale);
		glVertex3f(pts2Drigid[2*nrigid-2]*lassoscale, (vport[3]-pts2Drigid[2*nrigid-1])*lassoscale, lassoscale);
		glEnd();
		
	}
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

-(void)renderBackground
{
	GLfloat f=10000.0f, n=.1f;
	GLfloat a=-(f+n);
	GLfloat b=-2.0*f*n;
	GLfloat c=f-n;
	GLfloat m_bp[16];
	GLfloat m_bm[16]={1.0,0.0,0.0,0.0,
		0.0,1.0,0.0,0.0,
		0.0,0.0,-1.0,0.0,
		0.0,0.0,0.0,1.0};
	// draw the background with the correct modelview and projection matrices
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glMultMatrixf(m_bm);
	
	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	
	float backgroundscale=4000;
	
	
	glLoadIdentity();
	int j=0; for (j=0; j<16; j++) m_bp[j]=0.0f;
	m_bp[0]=1.0f/((renderIsDone ? (float)reswidth:imwidth)*.5);
	m_bp[5]=-1.0f/((renderIsDone ? (float)resheight:imheight)*.5);
	m_bp[12]=-backgroundscale; m_bp[13]=backgroundscale;
	m_bp[10]=a/c; m_bp[11]=-1.0; m_bp[14]=b/c;
	glMultMatrixf(m_bp);
	
    
	// change:
	inpainted=manipPhase;
    GLuint mytex;
    glGenTextures(1, &mytex);
    glBindTexture(GL_TEXTURE_2D, mytex);
    
    if (renderIsDone) {
        glTexImage2D(GL_TEXTURE_2D, 0, 3, (int)(reswidth), (int)(resheight), 0, GL_RGB, GL_FLOAT, objectModel_renderedImage);
        
    } else if (!renderIsDone && inpainted) {
        
        //glActiveTexture(GL_TEXTURE2);
        //glShadeModel(GL_FLAT);
        glTexImage2D(GL_TEXTURE_2D, 0, 3, (int)(imwidth), (int)(imheight), 0, GL_RGB, GL_FLOAT, objectModel_image1);
        
	}
	else {
        //glActiveTexture(GL_TEXTURE2);
        //glShadeModel(GL_FLAT);
        glTexImage2D(GL_TEXTURE_2D, 0, 3, (int)(imwidth), (int)(imheight), 0, GL_RGB, GL_FLOAT, objectModel_image);
		
	}
    
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, mytex);
    

	glShadeModel(GL_FLAT);
	glBegin(GL_QUADS);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (!renderIsDone) {
        glTexCoord2f(0.0, 0.0); glVertex3f(0.0, 0.0, backgroundscale);
        glTexCoord2f(0.0, 1.0); glVertex3f(0.0, imheight*backgroundscale, backgroundscale);
        glTexCoord2f(1.0, 1.0);	glVertex3f(imwidth*backgroundscale, imheight*backgroundscale, backgroundscale);
        glTexCoord2f(1.0, 0.0);	glVertex3f(imwidth*backgroundscale, 0.0, backgroundscale);
    } else {
        glTexCoord2f(0.0, 0.0); glVertex3f(0.0, 0.0, backgroundscale);
        glTexCoord2f(0.0, 1.0); glVertex3f(0.0, (float)(resheight)*backgroundscale, backgroundscale);
        glTexCoord2f(1.0, 1.0);	glVertex3f((float)(reswidth)*backgroundscale, (float)(resheight)*backgroundscale, backgroundscale);
        glTexCoord2f(1.0, 0.0);	glVertex3f((float)(reswidth)*backgroundscale, 0.0, backgroundscale);
    }
	glEnd();
	glDisable(GL_TEXTURE_2D);
    glDeleteTextures(1, &mytex);
	
//	glUseProgramObjectARB(NULL);
	
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	j=0; for (j=0; j<16; j++) m_bp[j]=0.0f;
	m_bp[0]=1.0f/( (renderIsDone ? (float)reswidth:imwidth)*.5);
	m_bp[5]=-1.0f/((renderIsDone ? (float)resheight:imheight)*.5);
	m_bp[12]=-50.0; m_bp[13]=50.0;
	//	m_bp[0]=1.0f; m_bp[1]=-1.0f; m_bp[12]=0; m_bp[13]=0;
	m_bp[10]=a/c; m_bp[11]=-1.0; m_bp[14]=b/c;
	glMultMatrixf(m_bp);
	
	//	glPointSize(10);
	//	glBegin(GL_POINTS);
	//	glColor3f(0.0f, 1.0f, 1.0f);
	//	glVertex3f(.072148, .582847, 1.0);
	//	glEnd();
	
	if (openglViewNumber==2 && theOperationMode==MODE_PICKPOINTS)
	{
		
		glPointSize(10);
		glDisable(GL_TEXTURE_2D);
		glBegin(GL_POINTS);
		for (j=0; j<clickedPoints.size; j++)
		{
			GLfloat X, Y, Z;
			getElementFromArray(&clickedPoints, j, &X, &Y, &Z);
			glColor3f(colormap[j%9][0], colormap[j%9][1], colormap[j%9][2]);
			glVertex3f(X*50, Y*50,Z*50);
			
		}
		glEnd();
	}
	if (openglViewNumber==2 && theOperationMode==MODE_COMPUTEINTRINSICS)
	{
		
		glPointSize(10);
		glDisable(GL_TEXTURE_2D);
		glBegin(GL_LINES);
		for (j=0; j<ngroundpts; j+=2)
		{
			if (j+1 < ngroundpts) {
				glColor3f(1.0f, 1.0f, 1.0f);
				glVertex3f(50*groundpts[j][0],50*groundpts[j][1],50);
				glVertex3f(50*groundpts[j+1][0],50*groundpts[j+1][1],50);
			}
		}
		glEnd();
		
		glBegin(GL_POINTS);
		for (j=0; j<ngroundpts; j++)
		{
			glColor3f(1.0f, 0.0f, 0.0f);
			glVertex3f(50*groundpts[j][0],50*groundpts[j][1],50);
		}
		glEnd();
	}
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

-(void)renderObject
{
	GLfloat f=1.0e12f, n=10.0f;
	GLfloat a=-(f+n);
	GLfloat b=-2.0*f*n;
	GLfloat c=f-n;
	GLfloat m_tp[16];
	GLfloat m_tm[16]={MVx,0.0,0.0,0.0,
		0.0,MVy,0.0,0.0,
		0.0,0.0,MVz,0.0,
		0.0,0.0,0.0,1.0};
	
    resetToIdentity(switcherMatrix);
    
	if (openglViewNumber==1) {
		m_tm[0]=-1.0; m_tm[5]=0.0f; m_tm[6]=1.0f; m_tm[9]=1.0f; m_tm[10]=0.0f;
        memcpy(switcherMatrix, m_tm, 16*sizeof(float));
	}
	
	int j=0;
	
	// draw the objectModel with the correct modelview and  projection matrices
	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	
	if (openglViewNumber==2)
	{
		glLoadIdentity();
		for (j=0; j<16; j++) m_tp[j]=0.0f;
		m_tp[0]=fu/(imwidth*.5);
		m_tp[5]=(-1)*(MVy)*fv/(imheight*.5);
		m_tp[10]=a/c;
		m_tp[11]=Pz;
		m_tp[14]=b/c;
	}
	else {
		for (j=0;j<16;j++) m_tp[j]=.0f;
		GLfloat scalefactor=2;
        projectionFactor=1/(scalefactor*.5*(Mmax-Mmin));
		m_tp[0]=projectionFactor;
		m_tp[5]=projectionFactor;
		m_tp[10]=-projectionFactor;
		m_tp[14]=.0f;
		m_tp[15]=1.0f;
		glLoadIdentity();
		
	}
	glMultMatrixf(m_tp);
	glGetFloatv(GL_PROJECTION_MATRIX,projectionMatrix);
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	
	if (!(openglViewNumber==2 && !issetInitTransform))
	{
		glLoadIdentity();
		glMultMatrixf(m_tm);
		[self drawGrid];
		
        if (openglViewNumber==2 && issetInitTransform && manipPhase) {
            [self drawDuplicatedObjects];
        }
        
        
        if (openglViewNumber==2 && issetInitTransform && doWrite) {
            glColor3f(0.1, 0.0, .8);
            glBegin(GL_LINE_STRIP);
            for (int i=0; i<nwrite; i++) {
                glVertex3f(writeX[i], writeY[i], writeZ[i]);
            }
            glEnd();
        }
        
        // the translation axes need to be drawn before
		glMultMatrixf(trackTranslation);
		float tscale=trans_ax_val;
        
		glMultMatrixf(trackBallRotation);
		glMultMatrixf(objectRotation);
        //		drawRotationAxes(Xcentreo, Ycentreo, Zcentreo, R);
		if (openglViewNumber==1) glGetFloatv(GL_MODELVIEW_MATRIX, modelViewMatrix);
		
		GLfloat mm[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, mm);
        
        //printf("Res:%f,%f,%f\n",mm[12],mm[13],mm[14]);
        if (openglViewNumber==2) glMultMatrixf(initTransform);
        if (numComponentsSelected==0) {
            drawRotationAxes(Xcentreo, Ycentreo, Zcentreo, xAxisObject, yAxisObject, zAxisObject, R);
            drawTranslationAxes(Xcentreo, Ycentreo, Zcentreo, xAxisObject, yAxisObject, zAxisObject, R*2);
        } else {
            // drawRotationAxes for individual guys
            drawRotationAxes(Xcompo, Ycompo, Zcompo, xAxisComp, yAxisComp, zAxisComp, R);
            drawTranslationAxes(Xcompo, Ycompo, Zcompo, xAxisComp, yAxisComp, zAxisComp, R*2);
            //printf("%f, %f, %f", Xcomp,Ycomp,Zcomp);
        }
		//
		if (openglViewNumber==1)
			if (doDrawMesh) [self drawObjectModel];
		if (openglViewNumber==2 && issetInitTransform)
			if (doDrawMesh) { (!drawMeshOnly)  ? [self drawObjectModel] : [self drawObjectModelMesh]; }
        
        if (openglViewNumber==2 && issetInitTransform && manipPhase &&
			(theOperationMode==MODE_ROTATE || theOperationMode==MODE_AXISROTATE || theOperationMode==MODE_TRANSLATE ||
			 theOperationMode==MODE_SCALE ||
			 (theOperationMode==MODE_MARKDEFORM && !toggleState) ||
             (theOperationMode==MODE_MARKH && hmark==2) )) {
                // only allow computations of light and texture for manipulation phase not alignment phase
                
                // populate the transforms list so that you can use it in pbrt.cpp
                GLfloat mytransf[16];
                [self multiplyMatrices_M1:trackTranslation M2:trackBallRotation M3:objectRotation M4:initTransform Mout:mytransf];
                for (int i=0; i<16; i++) {
                    transforms.push_back(mytransf[i]);
                }
                transforms.push_back(n_transforms==0 || deforming ? 1.0f : 0.0f);
                if (n_transforms==0 || deforming) {
                    string path=direc;
                    path.append(name).append("/vertices_ui.dat");
                    FILE* fid=fopen(path.c_str(), n_transforms==0 ? "w" : "a");
                    fwrite(objectModel_vertices, 3*sizeof(float), nvertices, fid);
                    fclose(fid);
                }
                deforming=NO;
                //hmark=0;
                n_transforms++;
//                printf("n_transforms=%d\n",n_transforms);
            }
		
			}
    
    if (openglViewNumber==2 && theOperationMode==MODE_MARKDEFORM) {
        glDisable(GL_TEXTURE_2D);
		glPointSize(5);
		glBegin(GL_POINTS);
		for (j=0; j<nrayconstrained; j++) {
			if (rayconstrained_idxs[j]!=-1) {
				glColor3f(1.0f, .0f, .0f);
				glVertex3fv(&objectModel_vertices[3*rayconstrained_idxs[j]]);
			}
		}
        glEnd();
        
    }
    
    if (openglViewNumber==2 && theOperationMode==MODE_MARKRIGID) {
        glDisable(GL_TEXTURE_2D);
		glPointSize(5);
		glBegin(GL_POINTS);
		for (j=0; j<nrigid; j++) {
			if (rigid_idxs[j]!=-1) {
				glColor3f(1.0f, 1.0f, 0.0f);
				glVertex3fv(&objectModel_vertices[3*rigid_idxs[j]]);
			}
		}
        glEnd();
    }
    
    if (openglViewNumber==2 && theOperationMode==MODE_LASSO) {
        glDisable(GL_TEXTURE_2D);
		glPointSize(5);
		glBegin(GL_POINTS);
		for (int i=0; i<nvertices; i++) {
            if (isanchored[i]) {
                if (!groups || displayGroupNum==0 || displayGroupNum==groups[i]) {
                    glColor3f(0.0f, .0f, 1.0f);
                    glVertex3fv(&objectModel_vertices[3*i]);
                }
            }
        }
        glEnd();
    }
    
    if (theOperationMode==MODE_LASSODEFORM) {
        glDisable(GL_TEXTURE_2D);
		glPointSize(5);
		glBegin(GL_POINTS);
		for (int i=0; i<nvertices; i++) {
            if (islassodeformed[i]) {
                if (!groups || displayGroupNum==0 || displayGroupNum==groups[i]) {
                    glColor3f(.5f, .0f, 0.0f);
                    glVertex3fv(&objectModel_vertices[3*i]);
                }
            }
        }
        glEnd();
    }
    /*
     for (j=0; j<nanchored; j++) {
     if (!groups || displayGroupNum==0 || displayGroupNum==groups[anchored_idxs[j]]) {
     glColor3f(0.0f, .0f, 1.0f);
     glVertex3fv(&objectModel_vertices[3*anchored_idxs[j]]);
     }
     }
     for (j=0; j<nlassodeformed; j++) {
     if (!groups || displayGroupNum==0 || displayGroupNum==groups[lassodeformed_idxs[j]]) {
     glColor3f(.5f, .0f, 0.0f);
     glVertex3fv(&objectModel_vertices[3*lassodeformed_idxs[j]]);
     }
     }
     */
    
    
    if (openglViewNumber==2 && theOperationMode==MODE_MARKH) {
        glDisable(GL_TEXTURE_2D);
		glPointSize(5);
		glBegin(GL_POINTS);
        if (hmark>0) {
            glColor3f(1.0f, 1.0f, .0f);
            glVertex3f(hXstart, hYstart, hZstart);
        }
        if (hmark>1) {
            glColor3f(1.0f, 1.0f, .0f);
            glVertex3f(hXend, hYend, hZend);
        }
		glEnd();
    }
	
	if (openglViewNumber==1)
	{
		glDisable(GL_TEXTURE_2D);
		glPointSize(10);
		for (j=0; j<clickedPoints.size; j++)
		{
			GLfloat X, Y, Z;
			getElementFromArray(&clickedPoints, j, &X, &Y, &Z);
			drawSphere(X, Y, Z, r_sphere_val, colormap[j%9][0], colormap[j%9][1], colormap[j%9][2]);
		}
		
		if (!isInitObjectProj) {
			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glMultMatrixf(modelViewMatrix);
            
			multMatrix4SeveralVerticesDivw(mvpMatrix, objectModel_vertices, objectProjections, nvertices);
			glPopMatrix();
			isInitObjectProj=YES;
		}
	}
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

- (void) drawRect:(NSRect)rect
{
	// clear our drawable
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glViewport(rvport[0],rvport[1],rvport[2],rvport[3]);
	//glViewport(-10,-10,900,900);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	if (openglViewNumber==2)
	{
		[self renderBackground];		
	}
    [self renderLassoStrip];
    if ( !(openglViewNumber==2 && renderIsDone) ) {
        [self renderObject];
    }
	
	if (isBeingSaved) {
        /*	GLfloat dataPtr[640*360*3];
         glReadPixels(0, 0, 640, 360, GL_RGB, GL_FLOAT, dataPtr);
         FILE *pFile;
         pFile=fopen("/Users/nkholgad/CMU/Research/xcode/ObjectManipulation1/Pic1.dat", "w");
         fwrite(dataPtr, sizeof(GLfloat), 640*360*3, pFile);
         fclose(pFile);*/
		isBeingSaved=NO;
	}
	
	//if ([self inLiveResize])
    //glFlush ();
	//else
    [[self openGLContext]
     flushBuffer];
}


#pragma mark ---- mouse down, move, up ----
- (void)mouseDown:(NSEvent *)theEvent // trackball
{
    mousejustdown=true;
	if ([theEvent modifierFlags] & NSControlKeyMask) // send to pan
		[self rightMouseDown:theEvent];
	//else if ([theEvent modifierFlags] & NSAlternateKeyMask) // send to dolly
	//	[self otherMouseDown:theEvent];
	else {
		NSPoint location = [self convertPoint:[theEvent locationInWindow] fromView:nil];
		
		GLfloat vx=(GLfloat)location.x;
		GLfloat vy=(GLfloat)location.y;
		GLfloat vvx, vvy;
        if (openglViewNumber==2) {
            [self viewportTransformX:vx Y:vy toX:&vvx toY:&vvy];
        } else {
            vvx=2.f*(vx-rvport[0])/rvport[2]-1.0f;
            vvy=2.f*(vy-rvport[1])/rvport[3]-1.0f;
        }
		
		GLfloat pt2d[2]={vx,vy};
		if (theOperationMode==MODE_ZOOM) {
			doZoom=YES;
			x1=vx; y1=vy;
			//[self viewportTransformX:vx Y:vy toX:&x1 toY:&y1];
			//[self setNeedsDisplay:YES];
		} else if (theOperationMode==MODE_ROTATE || ([theEvent modifierFlags] & NSAlternateKeyMask) ||
                   (theOperationMode==MODE_AXISROTATE && openglViewNumber==2) ) {
            
            GLfloat mv[16];
            [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:mv];
            if (numComponentsSelected>0) {
                Xc=Xcomp; Yc=Ycomp; Zc=Zcomp;
                multMatrix4Vector3(mv, xAxisComp, xAxis);
                multMatrix4Vector3(mv, yAxisComp, yAxis);
                multMatrix4Vector3(mv, zAxisComp, zAxis);
            } else {
                if (theOperationMode==MODE_AXISROTATE) {
                    Xc=contactPoint[0]; Yc=contactPoint[1]; Zc=contactPoint[2];
                }
                else {
                    Xc=Xcentre; Yc=Ycentre; Zc=Zcentre;
                }
                multMatrix4Vector3(mv, xAxisObject, xAxis);
                multMatrix4Vector3(mv, yAxisObject, yAxis);
                multMatrix4Vector3(mv, zAxisObject, zAxis);
            }
            [self get3DPointOnSphere:startVec from2Dpoint:pt2d];
            
            isXaxis=NO; isYaxis=NO; isZaxis=NO;
            GLfloat dpx, dpy, dpz;
            
            dpx=(GLfloat)fabs((startVec[0]-Xc)*xAxis[0]+(startVec[1]-Yc)*xAxis[1]+(startVec[2]-Zc)*xAxis[2]);
            dpy=(GLfloat)fabs((startVec[0]-Xc)*yAxis[0]+(startVec[1]-Yc)*yAxis[1]+(startVec[2]-Zc)*yAxis[2]);
            dpz=(GLfloat)fabs((startVec[0]-Xc)*zAxis[0]+(startVec[1]-Yc)*zAxis[1]+(startVec[2]-Zc)*zAxis[2]);
            if (dpx<dpy && dpx<dpz && dpx<tol) isXaxis=YES; else if (dpy<dpx && dpy<dpz && dpy<tol) isYaxis=YES; else if (dpz<dpx && dpz<dpx && dpz<tol) isZaxis=YES;
            memcpy(verticesInMotion, objectModel_vertices, 3*sizeof(GLfloat)*nvertices);
		} else if (theOperationMode==MODE_SELECT) {
            float distanceOfNearest3DPoint=0;
            float threshold=1e-4;
            int nearest3DPointIndex=[self computeNearest3DPointIndexUsingProjectionsTo_x:vvx y:vvy distance:&distanceOfNearest3DPoint];
            if ([theEvent modifierFlags] & NSShiftKeyMask) {
                // do nothing if the shift key is on
            } else {
                // clear the components selected and the vertices selected
                numComponentsSelected=0;
                for (int j=0; j<ngroups; j++) {
                    isComponentSelected[j]=false;
                }
                for (int j=0; j<nvertices; j++) {
                    isVertexSelected[j]=false;
                }
            }
            //printf("Here, distance=%f\n",distanceOfNearest3DPoint);
            if (distanceOfNearest3DPoint < threshold)
            {
                // if the component was selected don't do anything
                
                // otherwise set this component to be selected
                if (!isComponentSelected[groups[nearest3DPointIndex]-1]) {
                    isComponentSelected[groups[nearest3DPointIndex]-1]=true;
                    numComponentsSelected++;
                    for (int j=0; j<nvertices; j++) {
                        if (groups[j]==groups[nearest3DPointIndex]) {
                            isVertexSelected[j]=true;
                        }
                    }
                }
            } else {
                // do nothing -- numComponentsSelected=0 will indicate that you are moving the whole object
            }
            [self computeCentroidAndAxesForSelection];
            GLfloat mv[16];
            [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:mv];
            transformPointToOther4(mv, &Xcompo, &Ycompo, &Zcompo, &Xcomp, &Ycomp, &Zcomp);
            [self setNeedsDisplay:YES];
        } else if (theOperationMode==MODE_LASSO || theOperationMode==MODE_LASSODEFORM) {
			////printf("mouse down: %f,%f\n",vx,vy);
			////printf("size=%d\n",arrayOfPoints.size);
			if (closeLoop) {
				closeLoop=NO;
				//clearArray(&arrayOfPoints);
				nlassopolygon=0;
			}
			lassoPolygon[2*nlassopolygon]=vvx;
			lassoPolygon[2*nlassopolygon+1]=vvy;
			nlassopolygon++;
			[self setNeedsDisplay:YES];
		} else if (theOperationMode==MODE_PICKPOINTS || theOperationMode==MODE_AXISROTATE) {
			GLfloat X, Y, Z;
			if (openglViewNumber==2 && theOperationMode==MODE_PICKPOINTS)
			{
				X=vvx; Y=vvy; Z=1.0f;
				insertElementIntoArray(&clickedPoints, X,Y,Z);
				pts2Drayconstrained[2*clickedPoints.size-2]=vvx;
				pts2Drayconstrained[2*clickedPoints.size-1]=vvy;
                printf("%f, %f;", vvx, vvy);
			} else if (openglViewNumber==1 && (theOperationMode==MODE_AXISROTATE || theOperationMode==MODE_PICKPOINTS)) {
                int nearest3DPointIndex=[self computeNearest3DPointIndexUsingProjectionsTo_x:vvx y:vvy];
                
                //                printf("point=[%f,%f], normalizedPoint=[%f,%f], pickedPointIndex=%d\n",vx, vy, vvx, vvy, nearest3DPointIndex);
                printf("%d, ", nearest3DPointIndex);
				X=objectModel_vertices[3*nearest3DPointIndex];
				Y=objectModel_vertices[3*nearest3DPointIndex+1];
				Z=objectModel_vertices[3*nearest3DPointIndex+2];
                
				if (theOperationMode==MODE_PICKPOINTS)
				{
					insertElementIntoArray(&clickedPoints, X,Y,Z);
					rayconstrained_idxs[nrayconstrained]=nearest3DPointIndex; nrayconstrained++;
					[[NSNotificationCenter defaultCenter] postNotificationName:@"PointsNotification" object:self];
				}
			}
			
			//printf("%d: %f, %f, %f\n", openglViewNumber, X, Y, Z);
			[self setNeedsDisplay:YES];
            
		} else if (openglViewNumber==2 && theOperationMode==MODE_MARKDEFORM) {
			toggleDeformDownState=!toggleDeformDownState;
			toggleState=true;
            int nearest3DPointIndex=[self computeNearest3DPointIndexUsingProjectionsTo_x:vvx y:vvy];
            
            rayconstrained_idxs[nrayconstrained]=nearest3DPointIndex;
            //rayconstrained_idxs[nrayconstrained]=3617;
            
            printf("ray constrained index=%d\n", rayconstrained_idxs[nrayconstrained]);
            dlinex=objectProjections[3*nearest3DPointIndex];
            dliney=objectProjections[3*nearest3DPointIndex+1];
            pts2Drayconstrained[2*nrayconstrained]=dlinex;
            pts2Drayconstrained[2*nrayconstrained+1]=dliney;
            nrayconstrained++;
			//}
			[self setNeedsDisplay:YES];
		}
        else if (openglViewNumber==2 && theOperationMode==MODE_MARKH) {
			toggleDeformDownState=!toggleDeformDownState;
			toggleState=true;
			int nearest3DPointIndex=[self computeNearest3DPointIndexUsingProjectionsTo_x:vvx y:vvy];
            
            if (hmark==0) {
                hXstart=objectModel_vertices[3*nearest3DPointIndex];
                hYstart=objectModel_vertices[3*nearest3DPointIndex+1];
                hZstart=objectModel_vertices[3*nearest3DPointIndex+2];
                hmark=1;
            } else if (hmark==1) {
                hXend=objectModel_vertices[3*nearest3DPointIndex];
                hYend=objectModel_vertices[3*nearest3DPointIndex+1];
                hZend=objectModel_vertices[3*nearest3DPointIndex+2];
                hmark=2;
            }
			//}
			[self setNeedsDisplay:YES];
		} else if (openglViewNumber==2 && theOperationMode==MODE_MARKRIGID) {
			toggleState=true;
			int nearest3DPointIndex=[self computeNearest3DPointIndexUsingProjectionsTo_x:vvx y:vvy];
            
            rigid_idxs[nrigid]=nearest3DPointIndex;
            
            printf("rigid index=%d\n", rigid_idxs[nrigid]);
            
            dlinex=objectProjections[3*nearest3DPointIndex];
            dliney=objectProjections[3*nearest3DPointIndex+1];
            pts2Drayconstrained[2*nrigid]=dlinex;
            pts2Drayconstrained[2*nrigid+1]=dliney;
            nrigid++;
			//}
			[self setNeedsDisplay:YES];
		} else if ( (openglViewNumber==2 && (theOperationMode==MODE_TRANSLATE || theOperationMode==MODE_SCALE) ) ||
                   (openglViewNumber==1 && theOperationMode==MODE_SCALE)) {
			GLfloat mv[16];
            [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:mv];
            if (numComponentsSelected > 0) {
                multMatrix4Vector3(mv, xAxisComp, xAxis);
                multMatrix4Vector3(mv, yAxisComp, yAxis);
                multMatrix4Vector3(mv, zAxisComp, zAxis);
                [self determineChoiceOfAxis_x:vvx _y:vvy _xC:Xcompo _yC:Ycompo _zC:Zcompo _xA:xAxisComp _yA:yAxisComp _zA:zAxisComp];
            } else {
                multMatrix4Vector3(mv, xAxisObject, xAxis);
                multMatrix4Vector3(mv, yAxisObject, yAxis);
                multMatrix4Vector3(mv, zAxisObject, zAxis);
                [self determineChoiceOfAxis_x:vvx _y:vvy _xC:Xcentreo _yC:Ycentreo _zC:Zcentreo _xA:xAxisObject _yA:yAxisObject _zA:zAxisObject];
            }
            printVector(Xinit);
            memcpy(verticesInMotion, objectModel_vertices, 3*sizeof(GLfloat)*nvertices);
            [self setNeedsDisplay:YES];
		} else if (theOperationMode==MODE_COMPUTEINTRINSICS && openglViewNumber==2) {
			if (ngroundpts==8) ngroundpts=0;
			[self viewportTransformX:vx Y:vy toX:&vvx toY:&vvy];
			
			groundpts[ngroundpts][0]=vvx;
			groundpts[ngroundpts][1]=vvy;
			groundpts[ngroundpts][2]=1.0f;
			ngroundpts++;
			
			if (ngroundpts==8) {
				[self computeIntrinsicsAndPlaneAxis];
			}
			[self setNeedsDisplay:YES];
		}
	}
}

- (void)mouseDragged:(NSEvent *)theEvent
{
	NSPoint location = [self convertPoint:[theEvent locationInWindow] fromView:nil];
	
	GLfloat vx=(GLfloat)location.x;
	GLfloat vy=(GLfloat)location.y;
	GLfloat vvx, vvy;
	
	GLfloat pt2d[2]={vx,vy};
	
	if (theOperationMode==MODE_ZOOM) {
		//[self viewportTransformX:vx Y:vy toX:&x2 toY:&y2];
		x2=vx; y2=vy;
		[self setNeedsDisplay:YES];
	} else if (theOperationMode==MODE_ROTATE || ([NSEvent modifierFlags] && NSAlternateKeyMask)
               || (theOperationMode==MODE_AXISROTATE && openglViewNumber==2)) {
        
		//if (mind>=-1e-3) {
		//memcpy(trackTranslationPrev, trackTranslation, 16*sizeof(float));
		//memcpy(trackBallRotationPrev, trackBallRotation, 16*sizeof(float));
        
        
		[self get3DPointOnSphere:endVec from2Dpoint:pt2d];
		[self computeRotation];
        if (numComponentsSelected>0) {
            deforming=YES;
            [[NSNotificationCenter defaultCenter] postNotificationName:@"arapNotification" object:self];
        }
		
        // Update the scaling axes for the whole object
		GLfloat multmat[16];
        [self multiplyMatrices_M1:trackBallRotation M2:objectRotation M3:initTransform Mout:multmat];
		
		//} else {
		//memcpy(trackTranslation, trackTranslationPrev, 16*sizeof(float));
		//memcpy(trackBallRotation, trackBallRotationPrev, 16*sizeof(float));
		//}
		[self setNeedsDisplay:YES];
	} else if (theOperationMode==MODE_MARKDEFORM) {
		[self viewportTransformX:vx Y:vy toX:&vvx toY:&vvy];
		pts2Drayconstrained[2*nrayconstrained-2]=vvx;
		pts2Drayconstrained[2*nrayconstrained-1]=vvy;
		//[self arapSolve];
		deforming=YES;
		[self setNeedsDisplay:YES];
	} else if (theOperationMode==MODE_MARKRIGID) {
		[self viewportTransformX:vx Y:vy toX:&vvx toY:&vvy];
		pts2Drigid[2*nrigid-2]=vvx;
		pts2Drigid[2*nrigid-1]=vvy;
		//[self arapSolve];
		deforming=YES;
		[self setNeedsDisplay:YES];
	} else if (theOperationMode==MODE_SCALE || theOperationMode==MODE_TRANSLATE) {
		GLfloat Xout[3];
        GLfloat ratio;
        if (openglViewNumber==2) {
            [self viewportTransformX:vx Y:vy toX:&vvx toY:&vvy];
        } else {
            vvx=2.f*(vx-rvport[0])/rvport[2]-1.0f;
            vvy=2.f*(vy-rvport[1])/rvport[3]-1.0f;
        }
        float cx, cy, cz;
        if (numComponentsSelected>0) {
            [self determineMotionOfAxis_x:vvx _y:vvy _xC:Xcompo _yC:Ycompo _zC:Zcompo _xA:xAxisComp _yA:yAxisComp _zA:zAxisComp _xout:Xout];
            cx=Xcompo; cy=Ycompo; cz=Zcompo;
            if (theOperationMode==MODE_SCALE)
                ratio=sqrtf((Xout[0]-Xcomp)*(Xout[0]-Xcomp)+(Xout[1]-Ycomp)*(Xout[1]-Ycomp)+(Xout[2]-Zcomp)*(Xout[2]-Zcomp))/
                sqrtf((Xinit[0]-Xcomp)*(Xinit[0]-Xcomp)+(Xinit[1]-Ycomp)*(Xinit[1]-Ycomp)+(Xinit[2]-Zcomp)*(Xinit[2]-Zcomp));
        } else {
            cx=Xcentreo; cy=Ycentreo; cz=Zcentreo;
            [self determineMotionOfAxis_x:vvx _y:vvy _xC:Xcentreo _yC:Ycentreo _zC:Zcentreo _xA:xAxisObject _yA:yAxisObject _zA:zAxisObject _xout:Xout];
            if (theOperationMode==MODE_SCALE)
                ratio=sqrtf((Xout[0]-Xcentre)*(Xout[0]-Xcentre)+(Xout[1]-Ycentre)*(Xout[1]-Ycentre)+(Xout[2]-Zcentre)*(Xout[2]-Zcentre))/
                sqrtf((Xinit[0]-Xcentre)*(Xinit[0]-Xcentre)+(Xinit[1]-Ycentre)*(Xinit[1]-Ycentre)+(Xinit[2]-Zcentre)*(Xinit[2]-Zcentre));
            //                ratio=sqrtf( (Xout[0]-Xinit[0])*(Xout[0]-Xinit[0]) + (Xout[1]-Xinit[1])*(Xout[1]-Xinit[1]) + (Xout[2]-Xinit[2])*(Xout[2]-Xinit[2]) ) /
            //              vNorm(xAxisObject) ;
        }
        /*printVector(Xout);
        printf(", raio=%f\n", ratio);
        if (ratio<.5) {
            printf("here");
        }*/
        if (theOperationMode==MODE_SCALE) {
            if (numComponentsSelected>0) {
                [self scaleIf_isX:isXaxis _isY:isYaxis _isZ:isZaxis _xA:xAxisComp _yA:yAxisComp _zA:zAxisComp Ratio:ratio _xC:Xcompo _yC:Ycompo _zC:Zcompo];
            } else {
                [self scaleIf_isX:isXaxis _isY:isYaxis _isZ:isZaxis _xA:xAxisObject _yA:yAxisObject _zA:zAxisObject Ratio:ratio _xC:Xcentreo _yC:Ycentreo _zC:Zcentreo];
            }
            /*for (int i=0; i<nvertices; i++) {
             if ( numComponentsSelected==0 || (numComponentsSelected>0 && isVertexSelected[i])) {
             // if (isXaxis) {
             //     objectModel_vertices[3*i]=(ratio)*xAxisObject[0]+(verticesInMotion[3*i]);
             //     objectModel_vertices[3*i+1]=(ratio)*xAxisObject[1]+(verticesInMotion[3*i+1]);
             //     objectModel_vertices[3*i+2]=(ratio)*xAxisObject[2]+(verticesInMotion[3*i+2]);
             // }
             if (isXaxis) objectModel_vertices[3*i]=ratio*(verticesInMotion[3*i]);//-cx)+cx;
             if (isYaxis) objectModel_vertices[3*i+1]=ratio*(verticesInMotion[3*i+1]);//-cy)+cy;
             if (isZaxis) objectModel_vertices[3*i+2]=ratio*(verticesInMotion[3*i+2]);//-cz)+cz;
             }
             }*/
            [[NSNotificationCenter defaultCenter] postNotificationName:@"arapNotification" object:self];
            deforming=YES;
        }
        if (theOperationMode==MODE_TRANSLATE) {
            GLfloat trackTranslationTemp[16];
            resetToIdentity(trackTranslationTemp);
            trackTranslationTemp[12]=Xout[0]-Xinit[0];
            trackTranslationTemp[13]=Xout[1]-Xinit[1];
            trackTranslationTemp[14]=Xout[2]-Xinit[2];
            
            GLfloat obinit[16];
            GLfloat obinitinv[16];
            [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:obinit];
            [self invertRigidTransform_in:obinit out:obinitinv];
            
            GLfloat multmat[16];
            GLfloat multmat2[16];
            [self multiplyMatrices_M1:trackTranslationTemp M2:obinit Mout:multmat];
            if (numComponentsSelected>0) {
                [self multiplyMatrices_M1:obinitinv M2:multmat Mout:multmat2];
                for(int i=0; i<nvertices; i++) {
                    if (isVertexSelected[i]) {
                        transformPoints4(multmat2, &objectModel_vertices[3*i], &objectModel_vertices[3*i+1], &objectModel_vertices[3*i+2]);
                    }
                }
                // transform the center to respond
                transformPoints4(multmat2, &(Xcompo), &(Ycompo), &(Zcompo));
                transformPoints4(trackTranslationTemp, &(Xcomp), &(Ycomp), &(Zcomp));
            } else {
                [self determineContactWithGround:multmat _prevTransf:trackTranslationTemp _nextTransf:trackTranslation];
            }
            
            if (numComponentsSelected>0) {
                memcpy(Xinit, Xout, 3*sizeof(GLfloat));
                [[NSNotificationCenter defaultCenter] postNotificationName:@"arapNotification" object:self];
                deforming=YES;
            }
        }
        
		[self setNeedsDisplay:YES];
	} else if (theOperationMode==MODE_LASSO || theOperationMode==MODE_LASSODEFORM) {
        if (openglViewNumber==2) {
            [self viewportTransformX:vx Y:vy toX:&vvx toY:&vvy];
        } else {
            vvx=2.f*(vx-rvport[0])/rvport[2]-1.0f;
            vvy=2.f*(vy-rvport[1])/rvport[3]-1.0f;
        }
        
		lassoPolygon[2*nlassopolygon]=vvx;
		lassoPolygon[2*nlassopolygon+1]=vvy;
		nlassopolygon++;
		[self setNeedsDisplay:YES];
	}
	
	if (openglViewNumber==2) {
		//	//printVector(before[0]);//printf("\n");
	}
	
	/*location.y = camera.viewHeight - location.y;
	 if (gTrackball) {
	 rollToTrackball (location.x, location.y, gTrackBallRotation);
	 [self setNeedsDisplay: YES];
	 } else if (gDolly) {
	 [self mouseDolly: location];
	 [self updateProjection];  // update projection matrix (not normally done on draw)
	 [self setNeedsDisplay: YES];
	 } else if (gPan) {
	 [self mousePan: location];
	 [self setNeedsDisplay: YES];
	 }*/
}


- (void)mouseUp:(NSEvent *)theEvent
{
	
	NSPoint location = [self convertPoint:[theEvent locationInWindow] fromView:nil];
	
	GLfloat vx=(GLfloat)location.x;
	GLfloat vy=(GLfloat)location.y;
	GLfloat vvx, vvy;
	
	GLfloat pt2d[2]={vx,vy};
	
	if (theOperationMode==MODE_ZOOM) {
		//[self viewportTransformX:vx Y:vy toX:&x2 toY:&y2];
		x2=vx; y2=vy;
		if (x1>x2) {
			GLfloat xsw=x2; x2=x1; x1=xsw;
		}
		if (y1>y2) {
			GLfloat ysw=y2; y2=y1; y1=ysw;
		}
		[self zoom];
		doZoom=NO;
		[self setNeedsDisplay:YES];
	} else if (theOperationMode==MODE_ROTATE //|| ([theEvent modifierFlags] && NSAlternateKeyMask)
               || (theOperationMode==MODE_AXISROTATE && openglViewNumber==2)) {
        if (numComponentsSelected>0) {
            [self undoredostore];
        }
		// update the object rotation to include the track ball rotation
        [self multiplyMatrices_M1:trackBallRotation M2:objectRotation Mout:objectRotation];
		resetToIdentity(trackBallRotation);
		
		////printMatrix4(objectRotation);
		if (openglViewNumber==2) {
			GLfloat mv[16];
            [self  multiplyMatrices_M1:objectRotation M2:initTransform Mout:mv];
            transformPointToOther4(mv, &Xcentreo, &Ycentreo, &Zcentreo, &Xcentre, &Ycentre, &Zcentre);
			transformPointToOther4(mv,&contactPointo[0], &contactPointo[1], &contactPointo[2],
                                   &contactPoint[0], &contactPoint[1], &contactPoint[2]);
			
			// apple specific
			//if (!manipPhase) [self appleRecalculateGroundAxis];
			if (!manipPhase) {
				string path=direc;
				FILE* fid=fopen(path.append(name).append("/initTransform.dat").c_str(), "w");
				fwrite(mv, 16, sizeof(GLfloat), fid);
				fclose(fid);
			}
		}
/*        if (numComponentsSelected>0) {
            if (smoothBetweenRigidMotions) {
                [self solveForSmoothingOverComponentMotions];
            }
        }*/
		
		[self setNeedsDisplay:YES];
		//glGetFloatv(GL_MODELVIEW_MATRIX, modelViewMatrix);
		if (openglViewNumber==1) {
            [self multiplyMatrices_M1:projectionMatrix M2:modelViewMatrix Mout:mvpMatrix];
			multMatrix4SeveralVerticesDivw(mvpMatrix, objectModel_vertices, objectProjections, nvertices); // change for orth.
		}
	} else if (theOperationMode==MODE_LASSO) {
		if ([theEvent clickCount]==2) {
			closeLoop=YES;
			[self setNeedsDisplay:YES];
			
			GLfloat mvMatrix[16];
			GLfloat x, y;
            [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:mvMatrix];
			multMatrix4SeveralVertices(mvMatrix, objectModel_vertices, objectProjections, nvertices); // change for orth.
			for (int i=0; i<nvertices; i++) {
				x=fu*objectProjections[3*i  ]/objectProjections[3*i+2]+uc;
				y=fv*objectProjections[3*i+1]/objectProjections[3*i+2]+vc;
				//objectProjections[3*i]=x; objectProjections[3*i+1]=y; //objectProjections[3*i+2]=1.0f;
				if (!groups || displayGroupNum==0 || displayGroupNum==groups[i]) {
					if (InsidePolygon(x, y, nlassopolygon, lassoPolygon)) {
						//anchored_idxs[nanchored]=i;
                        isanchored[i]=true;
						for (int jj=0; jj<nrayconstrained; jj++) {
							if (rayconstrained_idxs[jj]==i)
							{
								rayconstrained_idxs[jj]=-1;
							}
						}
						//nanchored++;
					}
				}
                //  if (isanchored[i]) printf("anchoredidx=%d\n",i);
			}
		}
	} else if (theOperationMode==MODE_LASSODEFORM) {
		
        if ([theEvent clickCount]==2) {
			closeLoop=YES;
			[self setNeedsDisplay:YES];
			GLfloat mvMatrix[16];
			GLfloat x, y;
            [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:mvMatrix];
			multMatrix4SeveralVertices(mvMatrix, objectModel_vertices, objectProjections, nvertices); // change for orth.
			for (int i=0; i<nvertices; i++) {
				x=fu*objectProjections[3*i  ]/objectProjections[3*i+2]+uc;
				y=fv*objectProjections[3*i+1]/objectProjections[3*i+2]+vc;
				//objectProjections[3*i]=x; objectProjections[3*i+1]=y; //objectProjections[3*i+2]=1.0f;
				if (!groups || displayGroupNum==0 || displayGroupNum==groups[i]) {
					if (InsidePolygon(x, y, nlassopolygon, lassoPolygon)) {
						//lassodeformed_idxs[nlassodeformed]=i;
                        islassodeformed[i]=true;
                        isVertexSelected[i]=true;
                        //for (int jj=0; jj<nrayconstrained; jj++) {
						//	if (rayconstrained_idxs[jj]==i)
						//	{
						//		rayconstrained_idxs[jj]=-1;
						//	}
						//}
						//nlassodeformed++;
					}
				}
                //   if (islassodeformed[i]) printf("lassodeformedidx=%d\n",i);
			}
            
            string path=direc; path.append(name).append("/islassodeformed.dat");
            FILE* fid=fopen(path.c_str(),"w");
            float ild=0;
            for (int k=0; k<nvertices; k++) {
                ild=islassodeformed[k] ? 1.0 : 0.0;
                fwrite(&ild, sizeof(float), 1, fid);
            }
            fclose(fid);
            
            numComponentsSelected=1;
            [self computeCentroidAndAxesForSelection];
            GLfloat mv[16];
            [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:mv];
            transformPointToOther4(mv, &Xcompo, &Ycompo, &Zcompo, &Xcomp, &Ycomp, &Zcomp);
            [self setNeedsDisplay:YES];
		}
        
        
        // printf("nlassodeformed=%d\n",nlassodeformed);
	} else if (theOperationMode==MODE_TRANSLATE && openglViewNumber==2) {
        
        if (numComponentsSelected>0) {
            [self undoredostore];
        }
        [self multiplyMatrices_M1:trackTranslation M2:objectRotation Mout:objectRotation];
		resetToIdentity(trackTranslation);
		GLfloat mv[16];
        [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:mv];
        
		if (openglViewNumber==2) {
			transformPointToOther4(mv, &Xcentreo, &Ycentreo, &Zcentreo, &Xcentre, &Ycentre, &Zcentre);
			transformPointToOther4(mv,&contactPointo[0], &contactPointo[1], &contactPointo[2],
                                   &contactPoint[0], &contactPoint[1], &contactPoint[2]);
		}
        [self setNeedsDisplay:YES];
		if (!manipPhase) {
			string path=direc;
			FILE* fid=fopen(path.append(name).append("/initTransform.dat").c_str(), "w");
			fwrite(mv, 16, sizeof(GLfloat), fid);
			fclose(fid);
		}
	} else if (theOperationMode==MODE_MARKDEFORM && openglViewNumber==2) {
		toggleDeformUpState=!toggleDeformUpState;
		toggleState=false;
		[self viewportTransformX:vx Y:vy toX:&vvx toY:&vvy];
		pts2Drayconstrained[2*nrayconstrained-2]=vvx;
		pts2Drayconstrained[2*nrayconstrained-1]=vvy;
        printf("Ray constrained end: %f, %f\n", vvx, vvy);
        //		pts2Drayconstrained[2*nrayconstrained-2]=1415.923950;
        //		pts2Drayconstrained[2*nrayconstrained-1]=1121.085327;
        [self undoredostore];
		[self setNeedsDisplay:YES];
		//if (toggleDeformUpState) {
		//	[self viewportTransformX:vx Y:vy toX:&vvx toY:&vvy];
		//	pts2Drayconstrained[2*nrayconstrained-2]=vvx;
		//	pts2Drayconstrained[2*nrayconstrained-1]=vvy;
		//}
    } else if (theOperationMode==MODE_MARKRIGID && openglViewNumber==2) {
		toggleState=false;
		[self viewportTransformX:vx Y:vy toX:&vvx toY:&vvy];
		pts2Drigid[2*nrigid-2]=vvx;
		pts2Drigid[2*nrigid-1]=vvy;
		[self setNeedsDisplay:YES];
        printf("nrigid=%d\n",nrigid);
		//if (toggleDeformUpState) {
		//	[self viewportTransformX:vx Y:vy toX:&vvx toY:&vvy];
		//	pts2Drayconstrained[2*nrayconstrained-2]=vvx;
		//	pts2Drayconstrained[2*nrayconstrained-1]=vvy;
		//}
	} else if (theOperationMode==MODE_SCALE) {
        [self undoredostore];
	}
	
	//	//printf("click count=%d",[theEvent clickCount]);
	gTrackingViewInfo = NULL;
}


- (void)scrollWheel:(NSEvent *)theEvent
{
	
	NSPoint location = [self convertPoint:[theEvent locationInWindow] fromView:nil];
	
	GLfloat xs=(GLfloat)location.x;
	GLfloat ys=(GLfloat)location.y;
	
	float wheelDelta = [theEvent deltaX] +[theEvent deltaY] + [theEvent deltaZ];
	//printf("%f\n",wheelDelta);
	
	GLfloat scalefactor=20.0f;
	if (wheelDelta) {
		//printf("%f\n",wheelDelta);
		GLfloat width_=rvport[2];
		GLfloat height_=rvport[3];
		GLfloat x0=rvport[0];
		GLfloat y0=rvport[1];
		GLfloat aspectratio=height_/width_;
		
		GLfloat xdelta1=wheelDelta*scalefactor;
		GLfloat xdelta2=width_-(width_*(xs-x0-xdelta1))/(xs-x0);
		GLfloat ydelta2=((width_+xdelta2)/width_)*height_-height_;
		GLfloat ydelta1=(ys-y0)*(ydelta2-height_)/height_+ys-y0;
		
		rvport[0]-=xdelta1;
		rvport[1]-=ydelta1;
		rvport[2]+=xdelta2;
		rvport[3]+=ydelta2;
		
		//rvport[0]+=wheelDelta*scalefactor;
		//rvport[1]+=aspectratio*wheelDelta*scalefactor;
		//rvport[2]-=2*wheelDelta*scalefactor;
		//rvport[3]-=2*aspectratio*wheelDelta*scalefactor;
		[self setNeedsDisplay:YES];
	}
	
	/*if (wheelDelta)
     {
     GLfloat deltaAperture = wheelDelta * -camera.aperture / 200.0f;
     camera.aperture += deltaAperture;
     if (camera.aperture < 0.1) // do not let aperture <= 0.1
     camera.aperture = 0.1;
     if (camera.aperture > 179.9) // do not let aperture >= 180
     camera.aperture = 179.9;
     //[self updateProjection]; // update projection matrix
     [self setNeedsDisplay: YES];
     }*/
	
}

- (void)rightMouseDown:(NSEvent *)theEvent // pan
{
	NSPoint location = [self convertPoint:[theEvent locationInWindow] fromView:nil];
	
	GLfloat vx=(GLfloat)location.x;
	GLfloat vy=(GLfloat)location.y;
	//[self viewportTransformX:vx Y:vy toX:&panx toY:&pany];
	panx=vx; pany=vy;
	r0=rvport[0]; r1=rvport[1];
}

- (void)otherMouseDown:(NSEvent *)theEvent //dolly
{
	
}

- (void)rightMouseUp:(NSEvent *)theEvent
{
	//[self mouseUp:theEvent];
	if ([theEvent clickCount]==2) {
		NSRect brect=[self frame];
		float wratio=((float)(brect.size.width))/imwidth;
		float hratio=((float)(brect.size.height))/imheight;
		if (wratio<hratio) {
			rvport[2]=brect.size.width; rvport[3]=wratio*imheight;
			rvport[0]=0; rvport[1]=(brect.size.height-rvport[3])/2.0f;
		} else {
			rvport[2]=hratio*imwidth; rvport[3]=brect.size.height;
			rvport[1]=0; rvport[0]=(brect.size.width-rvport[2])/2.0f;
		}
	}
	[self setNeedsDisplay:YES];
}

- (void)otherMouseUp:(NSEvent *)theEvent
{
	[self mouseUp:theEvent];
	
}

- (void)rightMouseDragged:(NSEvent *)theEvent
{
	//[self mouseDragged: theEvent];
	NSPoint location = [self convertPoint:[theEvent locationInWindow] fromView:nil];
	
	GLfloat vx=(GLfloat)location.x;
	GLfloat vy=(GLfloat)location.y;
	//GLfloat vvx, vvy;
	//[self viewportTransformX:vx Y:vy toX:&vvx toY:&vvy];
	GLfloat deltax=vx-panx;
	GLfloat deltay=vy-pany;
	rvport[0]=r0+deltax;
	rvport[1]=r1+deltay;
	[self setNeedsDisplay:YES];
}
- (void)otherMouseDragged:(NSEvent *)theEvent
{
	[self mouseDragged: theEvent];
}


// move camera in z axis
-(void)mouseDolly: (NSPoint) location
{
	
}

// ---------------------------------

// move camera in x/y plane
- (void)mousePan: (NSPoint) location
{
	
}

- (void) keyDown:(NSEvent *)theEvent
{
	/*if ([theEvent modifierFlags] & NSCommandKeyMask) {
     NSString *characters = [theEvent characters];
     if ([characters length]) {
     unichar character = [characters characterAtIndex:0];
     switch (character) {
     case 'z':
     printf("Cmd+z pressed\n");
     switch (theOperationMode) {
     case MODE_PICKPOINTS:
     clickedPoints.size--;
     [self setNeedsDisplay:YES];
     break;
     default:
     break;
     }
     break;
     default:
     break;
     }
     }
     }*/
	if (issetInitTransform && openglViewNumber==2) {
		NSString *characters = [theEvent characters];
		if ([characters length]) {
			unichar character = [characters characterAtIndex:0];
            if (character=='g') {
                if (!groups) {
                    if (theOperationMode==MODE_LASSODEFORM) displayGroupNum=0; else displayGroupNum=0;
                } else {
                    if (theOperationMode==MODE_LASSODEFORM) {
                        displayGroupNum++; if (displayGroupNum>ngroups) displayGroupNum=0;
                    }
                    else {
                        displayGroupNum++; if (displayGroupNum>ngroups) displayGroupNum=0;
                    }
                }
                [self setNeedsDisplay:YES];
                printf("displaying group=%d\n", theOperationMode==MODE_LASSODEFORM ? displayGroupNum : displayGroupNum);
                
            } else if (character=='G') {
                if (!groups) {
                    if (theOperationMode==MODE_LASSODEFORM) displayGroupNum=0; displayGroupNum=0;
                } else {
                    if (theOperationMode==MODE_LASSODEFORM) {
                        displayGroupNum--;
                        if (displayGroupNum<0) displayGroupNum=ngroups;
                    }
                    else {
                        displayGroupNum--;
                        if (displayGroupNum<0) displayGroupNum=ngroups;
                    }
                    printf("displaying group=%d\n", theOperationMode==MODE_LASSODEFORM ? displayGroupNum : displayGroupNum);
                }
                [self setNeedsDisplay:YES];
                printf("displaying group=%d\n", displayGroupNum);
            } else if (character=='z') {
                [self undo];
            } else if (character=='y') {
                [self redo];
            } else if (character=='0') {
                if (theOperationMode==MODE_LASSODEFORM) displayGroupNum=0; else displayGroupNum=0;
                printf("displaying group=%d\n", theOperationMode==MODE_LASSODEFORM ? displayGroupNum : displayGroupNum);
                [self setNeedsDisplay:YES];
            } else if (character=='m') {
                drawMeshOnly=!drawMeshOnly;
                printf("drawMeshOnly=%d\n", drawMeshOnly ? 1 : 0);
                [self setNeedsDisplay:YES];
            } else if (character=='n') {
                doDrawMesh=!doDrawMesh;
                printf("doDrawMesh=%d\n", doDrawMesh ? 1 : 0);
                [self setNeedsDisplay:YES];
            } else if (character=='f') {
                [self forceContactWithGround];                
            }
		}
        
    } else if (openglViewNumber==1) {
        NSString *characters = [theEvent characters];
        if ([characters length]) {
            unichar character = [characters characterAtIndex:0];
            if (character=='g') {
                [[NSNotificationCenter defaultCenter] postNotificationName:@"groupForwardNotification" object:self];
                
            } else if (character=='G') {
                [[NSNotificationCenter defaultCenter] postNotificationName:@"groupBackwardNotification" object:self];
                
            } else if (character=='z') {
                [[NSNotificationCenter defaultCenter] postNotificationName:@"imageUndoNotification" object:self];
                
            } else if (character=='y') {
                [[NSNotificationCenter defaultCenter] postNotificationName:@"imageRedoNotification" object:self];
                
            }
        }
    }
}

# pragma mark ---- Rotation and Translation transfer ----
-(void)setInitRotation:(double[3][3])Rot Translation:(double[3])t
{
	int i, j;
	////printf("HERE, num=%d\n", openglViewNumber);
	if (clickedPoints.size>=3)
        //if (false)
	{
		for (i=0; i<3; i++)
		{
			for (j=0; j<3; j++)
				initTransform[i+4*j]=(GLfloat)Rot[i][j];
			initTransform[i+12]=(GLfloat)t[i];
			initTransform[3+4*i]=0.0f;
		}
		initTransform[15]=1.0f;
		
        printf("Using transform computed with e-pnp\n");
        
		string path=direc;
		FILE* fid=fopen(path.append(name).append("/initTransform.dat").c_str(), "w");
		fwrite(initTransform, 16, sizeof(GLfloat), fid);
		fclose(fid);
        
	}
	else {
		/*initTransform[0]=-.7212; initTransform[1]=.2675; initTransform[2]=-.6390; initTransform[3]=0.0f;
		 initTransform[4]=-.0078; initTransform[5]=.9255; initTransform[6]= .3787; initTransform[7]=0.0f;
		 initTransform[8]= .6927; initTransform[9]=.2681; initTransform[10]=-.6695; initTransform[11]=0.0f;
		 initTransform[12]=8.6396; initTransform[13]=11.0504; initTransform[14]=124.3614; initTransform[15]=1.0f;*/
		//FILE* fp=fopen("/Users/nkholgad/CMU/Research/xcode/ObjectManipulation1/initTransform.dat","r");
		//		FILE* fp=fopen("/Users/nkholgad/CMU/Research/xcode/ObjectManipulation1/initTransform.dat","r");
		//		fread(initTransform, sizeof(GLfloat), 16, fp);
		//		fclose(fp);
		string path=direc;
		FILE* fid=fopen(path.append(name).append("/initTransform.dat").c_str(), "r");
		if (fid) {
			fread(initTransform, 16, sizeof(GLfloat), fid);
			fclose(fid);
		} else {
            
            for (i=0; i<3; i++) {
                for (j=0; j<3; j++) {
                    if (i==j) {
                        initTransform[i+4*j]=1.0f;
                    } else {
                        initTransform[i+4*j]=.0f;
                    }
                }
            }
            
		}
	}
	
    
    
	issetInitTransform=YES;
	inpainted=1;
	theOperationMode=MODE_TRANSLATE;
	transformPoints4(initTransform, &Xcentre, &Ycentre, &Zcentre);
	//Xcentreo=Xcentre; Ycentreo=Ycentre; Zcentreo=Zcentre;
    
	// apple specific
	//[self appleRecalculateGroundAxis];
	
	// Hard Coded ground axis---
	
	// taxi specific
	//groundAxis[0]=0.1088; groundAxis[1]=-0.5876; groundAxis[2]=-0.8018;
	
	// Evals Fruits2
	//groundAxis[0]=0.101460; groundAxis[1]=-0.812668; groundAxis[2]=-0.573827;
	
	// patrik specific
	//groundAxis[0]=0.042568; groundAxis[1]=-0.966233; groundAxis[2]=-0.254131;
	
	
	// groundAxis[0]=-0.008838;groundAxis[1]=-0.783831;groundAxis[2]=-0.620911;
	
	// Evals Fruits3/1
	//groundAxis[0]=-0.141923; groundAxis[1]=-0.444932; groundAxis[2]=-0.884247;
    
	// Evals Fruits3/3
	//groundAxis[0]=0.057756; groundAxis[1]=0.996873; groundAxis[2]=0.053937;
    
	// Evals Fruits3/5
	//groundAxis[0]=0.019193; groundAxis[1]=-0.949631; groundAxis[2]=-0.312783;
    
	// patrik demo
	//groundAxis[0]=0.010619; groundAxis[1]=-0.999435; groundAxis[2]=-0.031886;
	
	// tulsta
	//groundAxis[0]=0.077796; groundAxis[1]=-0.837433; groundAxis[2]=-0.540974;
	
	// corolla
	//groundAxis[0]=-0.013698; groundAxis[1]=-0.996875; groundAxis[2]=0.077803;
	
	// pen
	//groundAxis[0]=-0.021129; groundAxis[1]=-0.020147; groundAxis[2]=-0.999574;
    
    // mango edited
    //groundAxis[0]=0.009508; groundAxis[1]=-0.341560; groundAxis[2]=-0.939812;
	
    //groundAxis[0]=0.112044; groundAxis[1]=-0.941879; groundAxis[2]=-0.316716;
    
    // macbookpro
    // groundAxis[0]=0.013661; groundAxis[1]=-0.950302; groundAxis[2]=-0.311029;
    
    //  groundAxis[0]=0.022323; groundAxis[1]=-0.932953; groundAxis[2]=-0.359305;
    
    //groundAxis[0]=0.017822; groundAxis[1]=-0.931104; groundAxis[2]=-0.364319;
    
    //groundAxis[0]=-0.042902; groundAxis[1]=-0.936427; groundAxis[2]=-0.348230;
    
    //groundAxis[0]=-0.012908; groundAxis[1]=-0.949721; groundAxis[2]=-0.312831;
    
    //groundAxis[0]=-0.047835; groundAxis[1]=-0.952574; groundAxis[2]=-0.300523;
	//groundAxis[0]=-0.027782; groundAxis[1]=-0.936452; groundAxis[2]=-0.349693;
    // groundAxis[0]=-0.009738; groundAxis[1]=-0.933977; groundAxis[2]=-0.357200;
    // groundAxis[0]=0.029032; groundAxis[1]=-0.928030; groundAxis[2]=-0.371372;
    
    //groundAxis[0]=0.102932; groundAxis[1]=-0.918770; groundAxis[2]=-0.381139;
    
    //    groundAxis[0]=0.012059; groundAxis[1]=-0.930032; groundAxis[2]=-0.367280;
    
    // groundAxis[0]=-0.054986; groundAxis[1]=-0.984203; groundAxis[2]=-0.168286;
    
    //groundAxis[0]=-0.066003; groundAxis[1]=-0.980368; groundAxis[2]=-0.185799;
    
    //groundAxis[0]=-0.008255; groundAxis[1]=-0.988492; groundAxis[2]=-0.151048;
    
    // groundAxis[0]=0.109972; groundAxis[1]=0.889159; groundAxis[2]=0.444188;
    //groundAxis[0]=-0.087535; groundAxis[1]=-0.983208; groundAxis[2]=-0.160126;
    
    //groundAxis[0]=0.011674;groundAxis[1]=-0.949750;groundAxis[2]=-0.312792;
    
    //painting
    //groundAxis[0]=0.026421; groundAxis[1]=0.859342; groundAxis[2]=0.510718;
    
    // patrik in video
    //groundAxis[0]=-0.041234; groundAxis[1]=-0.974705; groundAxis[2]=-0.219659;
    //groundAxis[0]=-0.095143;groundAxis[1]=-0.981631;groundAxis[2]=-0.165374;
    
    //tophat
    // groundAxis[0]=-.016091; groundAxis[1]=-.929821; groundAxis[2]=-.367659;
    
    
    string path=direc;
    FILE* fid=fopen(path.append(name).append("/cameraplaneinfo.txt").c_str(), "r");
    if (fid) {
        float n1,n2,n3,n4;
        fscanf(fid, "campars=[%f,%f,%f,%f]\ngroundaxis=[%f,%f,%f,%f]",&n1,&n2,&n3,&n4,
               &groundAxis[0],&groundAxis[1],&groundAxis[2],&groundAxis[3]);
        fclose(fid);
    }
    
    if (!strcmp(name.c_str(),"person")) {
        groundAxis[0]=0.0160; groundAxis[1]=-0.9909; groundAxis[2]=-0.1337;
    } else if (!strcmp(name.c_str(),"sofa")) {
        groundAxis[0]=0.0082; groundAxis[1]=-0.9999; groundAxis[2]=-0.0074;
    }
    
    //fwrite(initTransform, 16, sizeof(GLfloat), fid);
    //fclose(fid);
    GLfloat* verticesTransformed=new GLfloat[nvertices*3];
	multMatrix4SeveralVertices(initTransform, objectModel_vertices, verticesTransformed, nvertices);
	GLfloat distance;
	
	GLfloat mind=INFINITY;
	int idxmind=0;
	for (int i=0; i<nvertices; i++) {
		distance=vDotProduct(groundAxis, &verticesTransformed[3*i]);
		if (distance<mind) { mind=distance; idxmind=i; };
	}
	groundAxis[3]=-mind;
    //groundAxis[3]=52;
	//GLfloat dr[3]={sqrtf(2.0f)/2,sqrtf(2.0f)/2,.0f};
	GLfloat dr[3]={1.0,.0,.0};
	
	memcpy(normalAxiso, groundAxis, 3*sizeof(float));
	memcpy(normalAxis, groundAxis, 3*sizeof(float));
	memcpy(contactPoint, &verticesTransformed[3*idxmind], 3*sizeof(float));
	memcpy(contactPointo, &objectModel_vertices[3*idxmind], 3*sizeof(float));
	memcpy(gridCentre, &verticesTransformed[3*idxmind], 3*sizeof(float));
	memcpy(gridAxis, groundAxis, 3*sizeof(float));
	vCrossProduct(dr, gridAxis, gridDr1); vNormalize(gridDr1);
	vCrossProduct(normalAxiso, gridDr1, gridDr2); vNormalize(gridDr2);
	
	delete[] verticesTransformed;
	
	// do an initial fit with the correspondences from epnp
	// give high weight to the ray constraints, and medium weight
	// to the laplacian operator and the anchors
    
	//if (clickedPoints.size>=3) {
	if (false) {
		for (j=0; j<nrayconstrained; j++) {
			printf("%f,%f\n", pts2Drayconstrained[2*j], pts2Drayconstrained[2*j+1]);
		}
		
		printf("Set up anchors\n");
		for (i=0; i<nvertices; i++) {
			for (j=0; j<nrayconstrained; j++) {
				if (rayconstrained_idxs[j]==i) {
					break;
				}
			}
			for (i=0; i<nvertices; i++) isanchored[i]=false;
            //if (j==nrayconstrained) {
            //anchored_idxs[nanchored]=i; nanchored++;
            //    isanchored[i]=true;
            //}
			//printf("anchor setup... %d\n",i);
		}
        //rayConstraintsWeight=.1f;
        
		nrayconstrained=0;
		//nanchored=0;
        //nlassodeformed=0;
        for (i=0; i<nvertices; i++) islassodeformed[i]=false;
        
        
	}
	[self undoredostore];
    
    //nanchored=0;
    nrayconstrained=0;
    nrigid=0;
    nlassomovelist=0;
    //nlassodeformed=0;
	for (i=0; i<nvertices; i++) isanchored[i]=false;
	for (i=0; i<nvertices; i++) {
		if (groups && groupsLocked[groups[i]-1]) {
			//anchored_idxs[nanchored]=i; nanchored++;
            isanchored[i]=true;
		}
	}
    for (i=0; i<nvertices; i++) islassodeformed[i]=false;
	//for (i=0; i<nvertices; i++) {
	//	if (groups && deformgroupsLocked[groups[i]-1]) {
    //lassodeformed_idxs[nlassodeformed]=i; nlassodeformed++;
    //        islassodeformed[i]=true;
	//	}
	//}
    
	
    //memcpy(lastRotation,initTransform,16*sizeof(float));
    resetToIdentity(lastRotation);
    
    // In this version, further allow the user to manipulate the object
    
    [self manipulate];
    [btnRotate setEnabled:YES];
    [btnTranslate setEnabled:YES];
    [btnScale setEnabled:YES];
    [btnReplicate setEnabled:YES];
    [btnRender setEnabled:YES];
    [btnManipulate setHidden:YES];
    [btnManipulate2 setHidden:NO];
    [btnManipulate2 setEnabled:NO];
    
	[self setNeedsDisplay:YES];
}

-(void) manipulate
{
    manipPhase=YES;
    renderIsDone=NO;
    
	GLfloat multmat[16];
    [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:multmat];
    
	GLfloat* verticesTransformed=new GLfloat[nvertices*3];
	multMatrix4SeveralVertices(multmat, objectModel_vertices, verticesTransformed, nvertices);
	GLfloat distance;
	GLfloat mind=INFINITY;
	int idxmind=0;
	for (int i=0; i<nvertices; i++) {
		distance=vDotProduct(groundAxis, &verticesTransformed[3*i]);
		if (distance<mind) { mind=distance; idxmind=i; };
	}
	groundAxis[3]=-mind;
    //groundAxis[3]=52;
	GLfloat* vT=new GLfloat[nvertices*3];
	multMatrix4SeveralVertices(initTransform, objectModel_vertices, vT, nvertices);
    memcpy(contactPoint, &verticesTransformed[3*idxmind], sizeof(float)*3);
	memcpy(contactPointo, &objectModel_vertices[3*idxmind], sizeof(float)*3);
	memcpy(gridCentre, &verticesTransformed[3*idxmind], sizeof(float)*3);
    
    // Change the axes
    
    /*
     GLfloat mvinv[16];
     [self invertRigidTransform_in:multmat out:mvinv];
     multMatrix4Vector3(mvinv, gridAxis, zAxisObject);
     multMatrix4Vector3(mvinv, gridDr1, xAxisObject);
     multMatrix4Vector3(mvinv, gridDr2, yAxisObject);
     
     delete[] vT;
     delete[] verticesTransformed;
     */
	
	string path=direc;
	FILE* fid=fopen(path.append(name).append("/verts_modified.txt").c_str(), "w");
	for (int i=0; i<nvertices; i++) {
		fprintf(fid, "%f %f %f\n", objectModel_vertices[3*i], objectModel_vertices[3*i+1], objectModel_vertices[3*i+2]);
	}
	fclose(fid);
    
    path=direc;
    fid=fopen(path.append(name).append("/plane_modified.dat").c_str(), "w");
    fwrite(plane, 4*sizeof(float), 1, fid);
    fclose(fid);
	
	path=direc;
	fid=fopen(path.append(name).append("/groundaxis.dat").c_str(), "w");
	fwrite(groundAxis, 4*sizeof(float), 1, fid);
	fclose(fid);
    drawMeshOnly=NO;
	[self setNeedsDisplay:YES];

}

#pragma mark ---- geometric computations for rotations, translations, and axis marking ----

- (void) multiplyMatrices_M1:(GLfloat*)m1 M2:(GLfloat*)m2 Mout:(GLfloat*)mout
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixf(m1);
    glMultMatrixf(m2);
    glGetFloatv(GL_MODELVIEW_MATRIX, mout);
    glPopMatrix();
}
- (void) multiplyMatrices_M1:(GLfloat*)m1 M2:(GLfloat*)m2 M3:(GLfloat*)m3 Mout:(GLfloat*)mout
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixf(m1);
    glMultMatrixf(m2);
    glMultMatrixf(m3);
    glGetFloatv(GL_MODELVIEW_MATRIX, mout);
    glPopMatrix();
}
- (void) multiplyMatrices_M1:(GLfloat*)m1 M2:(GLfloat*)m2 M3:(GLfloat*)m3 M4:(GLfloat*)m4 Mout:(GLfloat*)mout
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixf(m1);
    glMultMatrixf(m2);
    glMultMatrixf(m3);
    glMultMatrixf(m4);
    glGetFloatv(GL_MODELVIEW_MATRIX, mout);
    glPopMatrix();
}
- (void) invertRigidTransform_in:(GLfloat*)mv out:(GLfloat*)mvinv
{
    mvinv[0]=mv[0]; mvinv[1]=mv[4]; mvinv[2]=mv[8]; mvinv[3]=0.0f;
	mvinv[4]=mv[1]; mvinv[5]=mv[5]; mvinv[6]=mv[9]; mvinv[7]=0.0f;
	mvinv[8]=mv[2]; mvinv[9]=mv[6]; mvinv[10]=mv[10]; mvinv[11]=0.0f;
	mvinv[12]=-(mv[0]*mv[12]+mv[1]*mv[13]+mv[2]*mv[14]);
	mvinv[13]=-(mv[4]*mv[12]+mv[5]*mv[13]+mv[6]*mv[14]);
	mvinv[14]=-(mv[8]*mv[12]+mv[9]*mv[13]+mv[10]*mv[14]);
	mvinv[15]=1.0f;
	
}

- (void)computeRotation
{
	GLfloat v1[3];
	GLfloat v2[3];
	GLfloat r[3]; // axis
	GLfloat magv1, magv2, magr, dp12;
	GLfloat mv[16];
	
	GLfloat rrt[9];
	GLfloat rot[9];
	GLfloat objRot[16];
	
	
	v1[0]=startVec[0]-Xc;v1[1]=startVec[1]-Yc;v1[2]=startVec[2]-Zc;
	v2[0]=endVec[0]-Xc;	v2[1]=endVec[1]-Yc;	v2[2]=endVec[2]-Zc;
	
	GLfloat trackBallRotationTemp[16];
	
    /*
     glMatrixMode(GL_MODELVIEW);
     glPushMatrix();
     //glMultMatrixf(trackBallRotation);
     glLoadMatrixf(trackBallRotation);
     glMultMatrixf(objectRotation);
     glGetFloatv(GL_MODELVIEW_MATRIX, mv);
     glPopMatrix();
     */
    
    //[self multiplyMatrices_M1:trackBallRotation M2:objectRotation M3:initTransform Mout:mv];
	//multMatrix4Vector3(mv, xAxisObject, xAxis);multMatrix4Vector3(mv, yAxisObject, yAxis);multMatrix4Vector3(mv, zAxisObject, zAxis);
    multMatrix4Vector3(trackBallRotation, xAxis, xAxis);
	
	if (theOperationMode==MODE_AXISROTATE && openglViewNumber==2) {
		[self project:&v1[0] OntoPlanePerpToAxis:&groundAxis[0]];
		[self project:&v2[0] OntoPlanePerpToAxis:&groundAxis[0]];
		
	} else {
		if (isXaxis) {
			[self project:&v1[0] OntoPlanePerpToAxis:&xAxis[0]];
			[self project:&v2[0] OntoPlanePerpToAxis:&xAxis[0]];
		} else if (isYaxis) {
			[self project:&v1[0] OntoPlanePerpToAxis:&yAxis[0]];
			[self project:&v2[0] OntoPlanePerpToAxis:&yAxis[0]];
		} else if (isZaxis) {
			[self project:&v1[0] OntoPlanePerpToAxis:&zAxis[0]];
			[self project:&v2[0] OntoPlanePerpToAxis:&zAxis[0]];
		}
	}
	
	vCrossProduct(v1, v2, r);
	magv1=sqrt(vDotProduct(v1, v1));
	magv2=sqrt(vDotProduct(v2, v2));
	magr=sqrt(vDotProduct(r, r));
	dp12=vDotProduct(v1, v2);
	
	r[0]=r[0]/magr; r[1]=r[1]/magr; r[2]=r[2]/magr;
	
	GLfloat sinTheta=magr/(magv1*magv2);
	GLfloat cosTheta=dp12/(magv1*magv2);
	
	int i, j;
	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			rrt[i+3*j]=r[i]*r[j];
	GLfloat rx[9]={0.0f,r[2],-r[1],-r[2],0.0f,r[0],r[1],-r[0],0.0f};
	GLfloat eye[9]={1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	
	for (i=0; i<9; i++)
		rot[i]=eye[i]+sinTheta*rx[i]+(1-cosTheta)*(rrt[i]-eye[i]);
	
	resetToIdentity(trackBallRotationTemp);
	
	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			trackBallRotationTemp[i+4*j]=rot[i+3*j];
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
    glTranslatef(Xc, Yc, Zc);
    /*
     if (openglViewNumber==2 && theOperationMode==MODE_AXISROTATE)
     //glTranslatef(contactPoint[0],contactPoint[1],contactPoint[2]);
     glTranslatef(Xcentre, Ycentre, Zcentre);
     else
     glTranslatef(Xcentre, Ycentre, Zcentre);
     */
	glMultMatrixf(trackBallRotationTemp);
    /*
     if (openglViewNumber==2 && theOperationMode==MODE_AXISROTATE)
     //glTranslatef(-contactPoint[0],-contactPoint[1],-contactPoint[2]);
     glTranslatef(-Xcentre, -Ycentre, -Zcentre);
     else
     glTranslatef(-Xcentre, -Ycentre, -Zcentre);
     */
    glTranslatef(-Xc, -Yc, -Zc);
	glGetFloatv(GL_MODELVIEW_MATRIX, trackBallRotationTemp);
	glPopMatrix();
    
    GLfloat obinit[16];
    GLfloat obinitinv[16];
    [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:obinit];
	[self invertRigidTransform_in:obinit out:obinitinv];
	
	GLfloat multmat[16];
    GLfloat multmat2[16];
    [self multiplyMatrices_M1:trackBallRotationTemp M2:obinit Mout:multmat];
    
    // If selected vertices are being moved, then instead of updating the object rotation,
    // update the underlying vertices (in the same way as you would a deformation)
    // so apply the rotation, and then remove the effect of the object rotation and the init transform
    if (numComponentsSelected>0) {
        [self multiplyMatrices_M1:obinitinv M2:multmat Mout:multmat2];
        for(int i=0; i<nvertices; i++) {
            if (isVertexSelected[i]) {
                transformPoints4(multmat2, &objectModel_vertices[3*i], &objectModel_vertices[3*i+1], &objectModel_vertices[3*i+2]);
            }
        }
        // you will have to transform the axes to respond
        multMatrix4Vector3(multmat2, xAxisComp, xAxisComp);
        multMatrix4Vector3(multmat2, yAxisComp, yAxisComp);
        multMatrix4Vector3(multmat2, zAxisComp, zAxisComp);
        memcpy(startVec,endVec,3*sizeof(float));
    } else {
        // The following code figures out if any further motion of the vertices is to be allowed in case the vertices
        // touch the ground plane. Might need to modify this to deal with the selected vertices as well.
        
        [self determineContactWithGround:multmat _prevTransf:trackBallRotationTemp _nextTransf:trackBallRotation];
    }
}

-(void)computeIntrinsicsAndPlaneAxis
{
	float l11[3], l12[3], vp1[3];
	float l21[3], l22[3], vp2[3];
	float vl[3];
	
	vCrossProduct(groundpts[0], groundpts[1], l11); //vScaleByLast(l11);
	vCrossProduct(groundpts[2], groundpts[3], l12); //vScaleByLast(l12);
	vCrossProduct(l11, l12, vp1); vScaleByLast(vp1);
	
	vCrossProduct(groundpts[4], groundpts[5], l21); //vScaleByLast(l21);
	vCrossProduct(groundpts[6], groundpts[7], l22); //vScaleByLast(l22);
	vCrossProduct(l21, l22, vp2); vScaleByLast(vp2);
	
	vCrossProduct(vp1, vp2, vl); vScaleByLast(vl);
	
	float beta=-(vp1[2]*vp2[2])/(vp2[0]*(vp1[0] - vp1[2]*uc) - vp2[2]*(vp1[0]*uc + vp1[1]*vc - vp1[2]*(uc*uc + vc*vc)) + vp2[1]*(vp1[1] - vp1[2]*vc));
	float tempfu=fu, tempfv=fv;
	//fu=1/sqrtf(beta);
	fu=tempfu;
	fv=fu;
	printf("%f, %f, %f\n", beta, fu, fv);
	//if (beta<0) {
    // use exif
	//	fu=tempfu; fv=tempfv;
	//	printf("using, %f, %f\n", fu,fv);
	//}
	
	//groundAxis[0]=-fu*vl[0]; groundAxis[1]=-fu*vl[1]; groundAxis[2]=-uc*vl[0]-vc*vl[1]-vl[2];
    groundAxis[0]=fu*vl[0]; groundAxis[1]=fu*vl[1]; groundAxis[2]=uc*vl[0]+vc*vl[1]+vl[2];
	vNormalize(groundAxis);
    
	printf("groundAxis[0]=%f; groundAxis[1]=%f; groundAxis[2]=%f;\n", groundAxis[0], groundAxis[1], groundAxis[2]);
}

- (void) determineContactWithGround:(GLfloat*)multmat _prevTransf:(GLfloat*)prev
                        _nextTransf:(GLfloat*)next

{
    GLfloat* verticesTransformed=new GLfloat[nvertices*3];
    multMatrix4SeveralVertices(multmat, objectModel_vertices, verticesTransformed, nvertices);
    GLfloat distance;
    
    GLfloat mind=INFINITY;
    int idxmind=0;
    bool allowMotion;
    if (manipPhase) {
        for (int i=0; i<nvertices; i++) {
            distance=vDotProduct(groundAxis, &verticesTransformed[3*i])+groundAxis[3];
            if (distance<mind) { mind=distance; idxmind=i; };
        }
        allowMotion=mind>1e-3;
    } else {
        allowMotion=true;
    }
    //allowMotion=true;
    
    if (allowMotion) {
        //GLfloat* vT=new GLfloat[nvertices*3];
        memcpy(next, prev, 16*sizeof(float));
        //multMatrix4SeveralVertices(initTransform, objectModel_vertices, vT, nvertices);
        memcpy(contactPoint, &verticesTransformed[3*idxmind], sizeof(float)*3);
        memcpy(contactPointo, &objectModel_vertices[3*idxmind], sizeof(float)*3);
        
        //printf ("mind=%f\n", mind);
        if (doWrite && mind<10 && nwrite<1000) {
            writeX[nwrite]=contactPoint[0];
            writeY[nwrite]=contactPoint[1];
            writeZ[nwrite]=contactPoint[2];
            printf("Added %f, %f, %f, nwrite=%d\n", writeX[nwrite], writeY[nwrite], writeZ[nwrite],nwrite);
            nwrite++;
        }
        //delete[] vT;
    }
    delete[] verticesTransformed;
}

- (void)forceContactWithGround
{
    // Modify the objectRotation so that after application of objectRotation*initTransform, the
    // object makes contact with the ground
    GLfloat mv[16];
    GLfloat* verticesTransformed=new GLfloat[nvertices*3];
    [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:mv];
    
    multMatrix4SeveralVertices(mv, objectModel_vertices, verticesTransformed, nvertices);
    
    int idx1=12134, idx2=11988, idx3=11944;
    float x11, x21, x31, x12, x32, x22, x13, x23, x33, denom, a1, a2, a3, anorm, dobj;
    float nobj[3];
    
    x11=verticesTransformed[3*idx1], x12=verticesTransformed[3*idx1+1], x13=verticesTransformed[3*idx1+2];
    x21=verticesTransformed[3*idx2], x22=verticesTransformed[3*idx2+1], x23=verticesTransformed[3*idx2+2];
    x31=verticesTransformed[3*idx3], x32=verticesTransformed[3*idx3+1], x33=verticesTransformed[3*idx3+2];
    
    denom=(x11*x22*x33 - x11*x23*x32 - x12*x21*x33 + x12*x23*x31 + x13*x21*x32 - x13*x22*x31);
    if (fabs(denom)>1e-6) {
        a1=(x12*x23 - x13*x22 - x12*x33 + x13*x32 + x22*x33 - x23*x32)/denom;
        a2=-(x11*x23 - x13*x21 - x11*x33 + x13*x31 + x21*x33 - x23*x31)/denom;
        a3=(x11*x22 - x12*x21 - x11*x32 + x12*x31 + x21*x32 - x22*x31)/denom;
        anorm=sqrtf(a1*a1+a2*a2+a3*a3);
        nobj[0]=-a1/anorm; nobj[1]=-a2/anorm; nobj[2]=-a3/anorm;
        dobj=1/anorm;
        
        // align the rotation of the chair with the ground
        float costheta=nobj[0]*groundAxis[0]+nobj[1]*groundAxis[1]+nobj[2]*groundAxis[2];
        costheta = costheta > 1 ? 1 : (costheta < -1 ? -1 : costheta);
        float sintheta=sqrtf(1-costheta*costheta);
        float axis[3];
        vCrossProduct(nobj, groundAxis, axis);
        if (vNorm(axis) > 1e-6) {
            vNormalize(axis);
            GLfloat rot[16];
            rot[0]=costheta+(1-costheta)*axis[0]*axis[0]; rot[4]=-axis[2]*sintheta+(1-costheta)*axis[0]*axis[1]; rot[8]=axis[1]*sintheta+(1-costheta)*axis[0]*axis[2]; rot[12]=0;
            rot[1]=axis[2]*sintheta+(1-costheta)*axis[1]*axis[0]; rot[5]=costheta+(1-costheta)*axis[1]*axis[1]; rot[9]=-axis[0]*sintheta+(1-costheta)*axis[1]*axis[2]; rot[13]=0;
            rot[2]=-axis[1]*sintheta+(1-costheta)*axis[2]*axis[0]; rot[6]=axis[0]*sintheta+(1-costheta)*axis[2]*axis[1]; rot[10]=costheta+(1-costheta)*axis[2]*axis[2]; rot[14]=0;
            rot[3]=0; rot[7]=0; rot[11]=0; rot[15]=1;
            
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            glTranslatef(Xcentre, Ycentre, Zcentre);
            glMultMatrixf(rot);
            glTranslatef(-Xcentre, -Ycentre, -Zcentre);
            glGetFloatv(GL_MODELVIEW_MATRIX, rot);
            glPopMatrix();
            
            GLfloat orprev[16];
            
            memcpy(orprev, objectRotation, 16*sizeof(float));
            [self multiplyMatrices_M1:rot M2:orprev Mout:objectRotation];
            [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:mv];
            multMatrix4SeveralVertices(mv, objectModel_vertices, verticesTransformed, nvertices);
            
            GLfloat distance;
            GLfloat mind=INFINITY;
            int idxmind=0;
            for (int i=0; i<nvertices; i++) {
                distance=vDotProduct(groundAxis, &verticesTransformed[3*i])+groundAxis[3];
                if (distance<mind) { mind=distance; idxmind=i; };
            }
            GLfloat lambda=-mind;
            objectRotation[12]+=lambda*groundAxis[0];
            objectRotation[13]+=lambda*groundAxis[1];
            objectRotation[14]+=lambda*groundAxis[2];
            [self setNeedsDisplay:YES];
        }
    }
    
    delete[] verticesTransformed;
}

- (void)getProjectionOf:(GLfloat*)xpt NearAxis:(GLfloat*)ax AtCentre:(GLfloat*)centre PlacedIn:(GLfloat*)Xout MinDistance:(GLfloat*)dis
{
	GLfloat* Ph=projectionMatrix;
	GLfloat cx[3], cy[3];
	cx[0]=-fu; cx[1]=0; cx[2]=-uc+xpt[0];
	cy[0]=0; cy[1]=-fv; cy[2]=-vc+xpt[1];
	
	float lambda;
	float cxTcentre;
	float cyTcentre;
	float cxTax;
	float cyTax;
    
    float axbar[3], centrebar[3];
    if (openglViewNumber==2) {
        GLfloat mv[16];
        [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:mv];
        multMatrix4Vector4(mv, centre, centrebar);
        multMatrix4Vector3(mv, ax, axbar);
        cxTcentre=vDotProduct(cx, centrebar);
        cyTcentre=vDotProduct(cy, centrebar);
        cxTax=vDotProduct(cx, axbar);
        cyTax=vDotProduct(cy, axbar);
        
        
    } else {
        multMatrix4Vector3(objectRotation, ax, axbar);
        multMatrix4Vector3(switcherMatrix, axbar, axbar);
        multMatrix4Vector4(objectRotation, centre, centrebar);
        multMatrix4Vector4(switcherMatrix, centrebar, centrebar);
        //axbar[0]*=projectionFactor; axbar[1]*=projectionFactor; axbar[2]*=projectionFactor;
        //centrebar[0]*=projectionFactor; centrebar[1]*=projectionFactor; centrebar[2]*=projectionFactor;
        
        // OBJECT HAS BEEN ROTATED!!
        cxTcentre=(projectionFactor*centrebar[0]-xpt[0]); cyTcentre=(projectionFactor*centrebar[1]-xpt[1]);
        cxTax=projectionFactor*axbar[0]; cyTax=projectionFactor*axbar[1];
    }
	lambda=-(cxTcentre*cxTax+cyTcentre*cyTax)/(cxTax*cxTax+cyTax*cyTax);
	
	for (int j=0; j<3; j++) Xout[j]=centrebar[j]+lambda*axbar[j];
	GLfloat xnear, ynear;
    
    if (openglViewNumber==2) {
        xnear=fu*Xout[0]/Xout[2]+uc; ynear=fv*Xout[1]/Xout[2]+vc;
    } else {
        //float Xoutbar[3];
        //multMatrix4Vector4(switcherMatrix, Xout, Xoutbar);
        xnear=projectionFactor*Xout[0]; ynear=projectionFactor*Xout[1];
    }
    
	*dis=(xnear-xpt[0])*(xnear-xpt[0])+(ynear-xpt[1])*(ynear-xpt[1]);
}

- (BOOL)testContactWithPlaneAxis:(GLfloat*)ax GridCentre:(GLfloat*)gc Vertices:(GLfloat*)verts
{
	int i=0;
	int done=0;
	for (i=0; i<nvertices && !done; i++) {
		GLfloat x, y, z;
		x=(verts[3*i+0]-gc[0]);
		y=(verts[3*i+1]-gc[1]);
		z=(verts[3*i+2]-gc[2]);
		GLfloat dp=ax[0]*x+ax[1]*y+ax[2]*z;
		done=(dp<=.0f);
	}
	if (done) return YES; else return NO;
}

- (void)reprojectVector:(GLfloat*)vin ontoPlaneAxis:(GLfloat*)ax SphereRadius:(GLfloat)rad ToGet:(GLfloat*)vout
{
	memcpy(vout,vin,3*sizeof(GLfloat));
	[self project:vout OntoPlanePerpToAxis:ax];
	vNormalize(vout);
	vScaleVector(vout, rad);
}

- (void)project:(GLfloat *)v OntoPlanePerpToAxis:(GLfloat *)axis
{
	GLfloat r[3];vCrossProduct(axis, v, r);vCrossProduct(r, axis, v);
}

- (void) viewportTransformX:(GLfloat)xin Y:(GLfloat)yin toX:(GLfloat*)xout toY:(GLfloat*)yout
{
	//*xout=vport[2]*xin/rvport[2];
	//*yout=vport[3]-vport[3]*yin/rvport[3];
	*xout=(xin-rvport[0])/(rvport[2])*(vport[2]-vport[0])+vport[0];
	*yout=(yin-rvport[1])/(rvport[3])*(vport[1]-vport[3])+vport[3];
	
}

- (void) zoom {
	NSRect brect=[self frame];
    
	GLfloat wratio=(x2-x1)/((float)(brect.size.width));
	GLfloat hratio=(y2-y1)/((float)(brect.size.height));
	
	GLfloat x1n, x2n, y2n, y1n;
	GLfloat deltameas;
	
	GLfloat rvportn[4];
	if (wratio<hratio) {
		// selected area is narrower than viewport
		y1n=0.0f;
		y2n=(GLfloat)brect.size.height;
		deltameas=(x2-x1)/(y2-y1)*brect.size.height;
		x1n=(brect.size.width-deltameas)/2.0f;
		x2n=(brect.size.width+deltameas)/2.0f;
	} else {
		// selected area is wider than viewport
		x1n=0.0f;
		x2n=(GLfloat)brect.size.width;
		deltameas=(y2-y1)/(x2-x1)*brect.size.width;
		y1n=(brect.size.height-deltameas)/2.0f;
		y2n=(brect.size.height+deltameas)/2.0f;
	}
	
	rvportn[0]=(rvport[0]*x1n - rvport[0]*x2n + x1*x2n - x1n*x2)/(x1 - x2);
	rvportn[2]=(rvport[2]*x1n - rvport[2]*x2n + x1*x2n - x1n*x2)/(x1 - x2)-rvportn[0];
	rvportn[1]=(rvport[1]*y1n - rvport[1]*y2n + y1*y2n - y1n*y2)/(y1 - y2);
	rvportn[3]=(rvport[3]*y1n - rvport[3]*y2n + y1*y2n - y1n*y2)/(y1 - y2)-rvportn[1];
	memcpy(rvport, rvportn, 4*sizeof(GLfloat));
}

- (void) appleRecalculateGroundAxis
{
	GLfloat multmat[16];
    [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:multmat];
    
	//int idxsuse[3]={456,405,357};
	//int idxsuse[3]={1904,2252,2401};
    int idxsuse[3]={17367,17205,17043};
	GLfloat p1[3], p2[3], p3[3];
	transformPointToOther4(multmat, &objectModel_vertices[3*idxsuse[0]],  &objectModel_vertices[3*idxsuse[0]+1],
						   &objectModel_vertices[3*idxsuse[0]+2],
						   &p1[0], &p1[1], &p1[2]);
	transformPointToOther4(multmat, &objectModel_vertices[3*idxsuse[1]],  &objectModel_vertices[3*idxsuse[1]+1],
						   &objectModel_vertices[3*idxsuse[1]+2],
						   &p2[0], &p2[1], &p2[2]);
	transformPointToOther4(multmat, &objectModel_vertices[3*idxsuse[2]],  &objectModel_vertices[3*idxsuse[2]+1],
						   &objectModel_vertices[3*idxsuse[2]+2],
						   &p3[0], &p3[1], &p3[2]);
	GLfloat p12[3], p13[3];
	for (int j=0; j<3; j++) {
		p12[j]=p2[j]-p1[j];
		p13[j]=p3[j]-p1[j];
	}
	
	vCrossProduct(p12, p13, groundAxis);
	vNormalize(groundAxis);
    
	
	GLfloat* verticesTransformed=new GLfloat[nvertices*3];
	multMatrix4SeveralVertices(multmat, objectModel_vertices, verticesTransformed, nvertices);
	GLfloat distance;
	GLfloat* vT=new GLfloat[nvertices*3];
	multMatrix4SeveralVertices(initTransform, objectModel_vertices, vT, nvertices);
    
	GLfloat mind=INFINITY;
	int idxmind=0;
	for (int i=0; i<nvertices; i++) {
		distance=vDotProduct(groundAxis, &verticesTransformed[3*i]);
		if (distance<mind) { mind=distance; idxmind=i; };
	}
    
	
	groundAxis[3]=-mind;
    //groundAxis[3]=52;
	//GLfloat dr[3]={sqrtf(2.0f)/2,sqrtf(2.0f)/2,.0f};
	GLfloat dr[3]={1.0,.0,.0};
	
	memcpy(contactPoint, &verticesTransformed[3*idxmind], 3*sizeof(float));
	memcpy(contactPointo, &objectModel_vertices[3*idxmind], 3*sizeof(float));
	memcpy(gridCentre, &verticesTransformed[3*idxmind], 3*sizeof(float));
	memcpy(gridAxis, groundAxis, 3*sizeof(float));
	vCrossProduct(dr, gridAxis, gridDr1); vNormalize(gridDr1);
	vCrossProduct(groundAxis, gridDr1, gridDr2); vNormalize(gridDr2);
    
    printf("gaxis: [%f, %f, %f]\n", groundAxis[0], groundAxis[1], groundAxis[2]);
    
    delete[] vT;
	delete[] verticesTransformed;
	
}

- (void) get3DPointOnSphere:(GLfloat*)the3Dpoint from2Dpoint:(GLfloat *)the2Dpoint
{
	GLfloat vx=the2Dpoint[0];
	GLfloat vy=the2Dpoint[1];
	
	GLfloat x=(vx-rvport[0])/((rvport[2])*.5)-1.0f;  // look into
	GLfloat y=(vy-rvport[1])/((rvport[3])*.5)-1.0f;  // look into
	
	/*	if (openglViewNumber==2) {
	 x=vx/(640*.5)-1.0f;
	 y=vy/(360*.5)-1.0f;
	 }*/
	
	GLfloat X, Y, Z;
	
	GLfloat PM[16];
	
	int i, j;
	GLfloat M[16]={MVx,0.0,0.0,0.0,
		0.0,MVy,0.0,0.0,
		0.0,0.0,MVz,0.0,
		0.0,0.0,0.0,1.0};
	
	if (openglViewNumber==1) {
		M[0]=-1.0; M[5]=0.0f; M[6]=1.0f; M[9]=1.0f; M[10]=.0f;
	}
	
	//glGetFloatv(GL_PROJECTION_MATRIX, &P[0]);
	
	// love the hacks you can do just to multiply stuff ;)
	// but be careful with them!
    [self multiplyMatrices_M1:projectionMatrix M2:M Mout:PM];
	
	// center of mass of objectModel and radius of sphere
	project2Donto3Dsphere(x, y, PM, M, Xc, Yc, Zc, R, &X, &Y, &Z);
	////printf("//: ");//printVector(the3Dpoint);//printf("\n");
	the3Dpoint[0]=X; the3Dpoint[1]=Y; the3Dpoint[2]=Z;
}

- (void) partBasedRigidTransform
{
    GLfloat mv[16];
    [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:mv];
    
    int i=0;
    
	for (i=0; i<nvertices; i++) {
		objectModel_vertexSol[3*i]=mv[0]*objectModel_vertices[3*i]+mv[4]*objectModel_vertices[3*i+1]+mv[8]*objectModel_vertices[3*i+2]+mv[12];
		objectModel_vertexSol[3*i+1]=mv[1]*objectModel_vertices[3*i]+mv[5]*objectModel_vertices[3*i+1]+mv[9]*objectModel_vertices[3*i+2]+mv[13];
		objectModel_vertexSol[3*i+2]=mv[2]*objectModel_vertices[3*i]+mv[6]*objectModel_vertices[3*i+1]+mv[10]*objectModel_vertices[3*i+2]+mv[14];
	}
    
    epnp pnp;
    pnp.set_internal_parameters(uc, vc, fu, fv);
    pnp.set_maximum_number_of_correspondences(nrigid); // provide max number of correspondences
	
	pnp.reset_correspondences(); // reset correspondences
    
    for (i=0; i<nrigid; i++)
    {
        GLfloat xx, yy, zz, XX, YY, ZZ;
        
        xx=pts2Drigid[2*i]; yy=pts2Drigid[2*i+1];
        XX=objectModel_vertexSol[3*rigid_idxs[i]]; YY=objectModel_vertexSol[3*rigid_idxs[i]+1]; ZZ=objectModel_vertexSol[3*rigid_idxs[i]+2];
        printf("%f, %f, %f, %f, %f\n", xx, yy, XX, YY, ZZ);
        pnp.add_correspondence((double)XX, (double)YY, (double)ZZ, (double)xx, (double)yy); // add correspondence into system
    }
    
    double Rest[3][3];
	double test[3];
    double err=pnp.compute_pose(Rest, test);
    
    
    int ii, jj;
    printf("R=[");
    for (ii=0; ii<3; ii++) {
        for	(jj=0; jj<3; jj++) printf("%f ", Rest[ii][jj]);
        printf("\n");
    }
    printf("]\nt=[");
    for (ii=0; ii<3; ii++) printf("%f ", test[ii]);
    printf("]\n");
    
    
    int nmov=0;
    
    for (i=0; i<nvertices; i++) {
        if (!isanchored[i]) {
            nmov++;
            GLfloat XX, YY, ZZ;
            
            XX=Rest[0][0]*objectModel_vertexSol[3*i]+Rest[0][1]*objectModel_vertexSol[3*i+1]+Rest[0][2]*objectModel_vertexSol[3*i+2]+test[0];
            YY=Rest[1][0]*objectModel_vertexSol[3*i]+Rest[1][1]*objectModel_vertexSol[3*i+1]+Rest[1][2]*objectModel_vertexSol[3*i+2]+test[1];
            ZZ=Rest[2][0]*objectModel_vertexSol[3*i]+Rest[2][1]*objectModel_vertexSol[3*i+1]+Rest[2][2]*objectModel_vertexSol[3*i+2]+test[2];
            objectModel_vertexSol[3*i]=XX; objectModel_vertexSol[3*i+1]=YY; objectModel_vertexSol[3*i+2]=ZZ;
        }
    }
    printf("%d/%d vertices moved\n", nmov, nvertices);
    
    
    GLfloat mvinv[16];
	[self invertRigidTransform_in:mv out:mvinv];
    
	for (i=0; i<nvertices; i++) {
		objectModel_vertices[3*i]=mvinv[0]*objectModel_vertexSol[3*i]+mvinv[4]*objectModel_vertexSol[3*i+1]+mvinv[8]*objectModel_vertexSol[3*i+2]+mvinv[12];
		objectModel_vertices[3*i+1]=mvinv[1]*objectModel_vertexSol[3*i]+mvinv[5]*objectModel_vertexSol[3*i+1]+mvinv[9]*objectModel_vertexSol[3*i+2]+mvinv[13];
		objectModel_vertices[3*i+2]=mvinv[2]*objectModel_vertexSol[3*i]+mvinv[6]*objectModel_vertexSol[3*i+1]+mvinv[10]*objectModel_vertexSol[3*i+2]+mvinv[14];
	}
}

- (void) scaleIf_isX:(bool)isX _isY:(bool)isY _isZ:(bool)isZ _xA:(GLfloat*)xa _yA:(GLfloat*)ya _zA:(GLfloat*)za Ratio:(GLfloat)ratio _xC:(GLfloat)xc _yC:(GLfloat)yc _zC:(GLfloat)zc
{
    
    GLfloat mv[16], mvinv[16];
    for (int j=0; j<15; j++) {
        mv[j]=.0f; mvinv[j]=.0f;
    }
    mv[15]=1.0f; mvinv[15]=1.0f;
    mv[0]=xa[0]; mv[1]=xa[1]; mv[2]=xa[2];
    mv[4]=ya[0]; mv[5]=ya[1]; mv[6]=ya[2];
    mv[8]=za[0]; mv[9]=za[1]; mv[10]=za[2];
    mvinv[0]=xa[0]; mvinv[4]=xa[1]; mvinv[8]=xa[2];
    mvinv[1]=ya[0]; mvinv[5]=ya[1]; mvinv[9]=ya[2];
    mvinv[2]=za[0]; mvinv[6]=za[1]; mvinv[10]=za[2];
    
    int coord=0;
    if (isX) coord=0; else if (isY) coord=1; else if (isZ) coord=2;
    for (int i=0; i<nvertices; i++) {
        if ( numComponentsSelected==0 || (numComponentsSelected>0 && isVertexSelected[i])) {
            objectModel_vertices[3*i]=verticesInMotion[3*i]-xc;
            objectModel_vertices[3*i+1]=verticesInMotion[3*i+1]-yc;
            objectModel_vertices[3*i+2]=verticesInMotion[3*i+2]-zc;
            
            multMatrix4Vector3(mvinv, &objectModel_vertices[3*i], &objectModel_vertices[3*i]);
            objectModel_vertices[3*i+coord]=ratio*objectModel_vertices[3*i+coord];
            multMatrix4Vector3(mv, &objectModel_vertices[3*i], &objectModel_vertices[3*i]);
            
            objectModel_vertices[3*i]+=xc;
            objectModel_vertices[3*i+1]+=yc;
            objectModel_vertices[3*i+2]+=zc;
        }
    }
    
}

- (void) computeCentroidAndAxesForSelection
{
    // Calculate the latest transformation of the selected vertices
    //GLfloat mv[16];
    //GLfloat* vT=new GLfloat[3*nvertices];
    //[self multiplyMatrices_M1:objectRotation M2:initTransform Mout:mv];
    //multMatrix4SeveralVertices(mv, objectModel_vertices, vT, nvertices);
    
    // Get the centroid of the selected vertices
    Xcompo=0; Ycompo=0; Zcompo=0; int nverticesselected=0;
    for (int i=0; i<nvertices; i++) {
        if (isVertexSelected[i]) {
            Xcompo+=objectModel_vertices[3*i];
            Ycompo+=objectModel_vertices[3*i+1];
            Zcompo+=objectModel_vertices[3*i+2];
            nverticesselected++;
        }
    }
    Xcompo/=nverticesselected; Ycompo/=nverticesselected; Zcompo/=nverticesselected;
    
    // Get the principal directions of the selected vertices
    /*if (workengine && selectionAxesArePCs) {
        mxArray* mxvT=mxCreateDoubleMatrix(3, nvertices, mxREAL);
        mxArray* mxC=mxCreateDoubleMatrix(3,1,mxREAL);
        double* prvT=mxGetPr(mxvT);
        double* prC=mxGetPr(mxC);
        prC[0]=Xcompo; prC[1]=Ycompo; prC[2]=Zcompo;
        for (int i=0; i<nvertices; i++) {
            if (isVertexSelected[i]) {
                prvT[3*i]=objectModel_vertices[3*i]-Xcompo;
                prvT[3*i+1]=objectModel_vertices[3*i+1]-Ycompo;
                prvT[3*i+2]=objectModel_vertices[3*i+2]-Zcompo;
            } else {
                prvT[3*i]=INFINITY;
                prvT[3*i+1]=INFINITY;
                prvT[3*i+2]=INFINITY;
            }
        }
        engPutVariable(workengine, "vT", mxvT);
        engEvalString(workengine, "vT=vT(:,~isinf(vT(1,:))); C=vT*vT'; [EVecs,~]=eig(C);");
        mxArray* mxEVecs=engGetVariable(workengine, "EVecs");
        double* prEVecs=mxGetPr(mxEVecs);
        
        xAxisComp[0]=prEVecs[0]; xAxisComp[1]=prEVecs[1]; xAxisComp[2]=prEVecs[2];
        yAxisComp[0]=prEVecs[3]; yAxisComp[1]=prEVecs[4]; yAxisComp[2]=prEVecs[5];
        zAxisComp[0]=prEVecs[6]; zAxisComp[1]=prEVecs[7]; zAxisComp[2]=prEVecs[8];
        
        mxDestroyArray(mxvT);
        mxDestroyArray(mxC);
        mxDestroyArray(mxEVecs);
    } else {*/
        xAxisComp[0]=1.0f; xAxisComp[1]=0.0f; xAxisComp[2]=0.0f;
        yAxisComp[0]=0.0f; yAxisComp[1]=1.0f; yAxisComp[2]=0.0f;
        zAxisComp[0]=0.0f; zAxisComp[1]=0.0f; zAxisComp[2]=1.0f;
    //}
    //delete[] vT;
}

- (void) determineChoiceOfAxis_x:(GLfloat)vvx _y:(GLfloat)vvy _xC:(GLfloat)xc _yC:(GLfloat)yc _zC:(GLfloat)zc _xA:(GLfloat*)xa _yA:(GLfloat*)ya _zA:(GLfloat*)za
{
    GLfloat X1[3], X2[3], X3[3];
    GLfloat d1, d2, d3;
    GLfloat xpt[2];
    
    xpt[0]=vvx; xpt[1]=vvy;
    GLfloat centre[3];
    centre[0]=xc; centre[1]=yc; centre[2]=zc;
    [self getProjectionOf:xpt NearAxis:xa AtCentre:centre PlacedIn:X1 MinDistance:&d1];
    [self getProjectionOf:xpt NearAxis:ya AtCentre:centre PlacedIn:X2 MinDistance:&d2];
    [self getProjectionOf:xpt NearAxis:za AtCentre:centre PlacedIn:X3 MinDistance:&d3];
    
    //printf("First: x-dist=%f, y-dist=%f, z-dist=%f\n", d1,d2,d3 );
    isXaxis=(d1<d2) && (d1<d3); if (isXaxis) memcpy(Xinit, X1, 3*sizeof(float));
    isYaxis=(d2<d1) && (d2<d3); if (isYaxis) memcpy(Xinit, X2, 3*sizeof(float));
    isZaxis=(d3<d1) && (d3<d2); if (isZaxis) memcpy(Xinit, X3, 3*sizeof(float));
    
}

- (void) determineMotionOfAxis_x:(GLfloat)vvx _y:(GLfloat)vvy _xC:(GLfloat)xc _yC:(GLfloat)yc _zC:(GLfloat)zc _xA:(GLfloat*)xa _yA:(GLfloat*)ya _zA:(GLfloat*)za _xout:(GLfloat*)Xout
{
    GLfloat dout;
    GLfloat xpt[2];
    xpt[0]=vvx; xpt[1]=vvy;
    
    GLfloat centre[3];
    centre[0]=xc; centre[1]=yc; centre[2]=zc;
    if (isXaxis) {
        [self getProjectionOf:xpt NearAxis:xa AtCentre:centre PlacedIn:Xout MinDistance:&dout];
        //printf("x-dist=%f, ", dout);
    }
    else if (isYaxis) {
        [self getProjectionOf:xpt NearAxis:ya AtCentre:centre PlacedIn:Xout MinDistance:&dout];
        //printf("y-dist=%f, ", dout);
    }
    else if (isZaxis) {
        [self getProjectionOf:xpt NearAxis:za AtCentre:centre PlacedIn:Xout MinDistance:&dout];
        //printf("z-dist=%f, ", dout);
    }
    //printf("\n");
    
}

- (int) computeNearest3DPointIndexUsingProjectionsTo_x:(GLfloat)vx y:(GLfloat)vy
{
    GLfloat mindist;
    return [self computeNearest3DPointIndexUsingProjectionsTo_x:vx y:vy distance:&mindist];
}

- (int) computeNearest3DPointIndexUsingProjectionsTo_x:(GLfloat)vx y:(GLfloat)vy distance:(GLfloat*)dist
{
    if (openglViewNumber==2) {
        [self computeObjectProjections];
    }
    calculateDistancesFromPointToPointsInVector
    (objectProjections, vx, vy,clickPointToObjectProjDistances, nvertices);
    
    for (int i=0; i<nvertices; i++) {
        if (!groups || displayGroupNum==0 || displayGroupNum==groups[i]) {
        } else {
            clickPointToObjectProjDistances[i]=INFINITY;
        }
    }
    
    GLfloat mindists[Knn]; int idxmindists[Knn]; int i=0;
    GLfloat Zs[Knn];
    
    int iZ;
    GLfloat depthZ;
    
    for (i=0; i<Knn; i++)
    {
        minWithHighReplacement(clickPointToObjectProjDistances, &mindists[i],
                               &idxmindists[i], nvertices);
        Zs[i]=objectProjections[3*idxmindists[i]+2];
    }
    minWithHighReplacement(Zs, &depthZ, &iZ, Knn);
    *dist=mindists[iZ]+vx*vx+vy*vy;
    if (openglViewNumber==2) {
        *dist /= (2*uc*uc + 2*vc*vc);
    }
    
    return idxmindists[iZ];
}

- (void) computeObjectProjections
{
    GLfloat mvMatrix[16];
    GLfloat x, y;
    [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:mvMatrix];
    multMatrix4SeveralVertices(mvMatrix, objectModel_vertices, objectProjections, nvertices); // change for orth.
    for (int i=0; i<nvertices; i++) {
        x=fu*objectProjections[3*i  ]/objectProjections[3*i+2]+uc;
        y=fv*objectProjections[3*i+1]/objectProjections[3*i+2]+vc;
        objectProjections[3*i]=x; objectProjections[3*i+1]=y; //objectProjections[3*i+2]=1.0f;
    }
    
}

-(void) hingeMotion
{
    GLfloat Xe[3];
    GLfloat Xs[3];
    GLfloat k[3], kd;
    k[0]=hXend-hXstart; k[1]=hYend-hYstart; k[2]=hZend-hZstart;
    kd=1.f/vNorm(k);
    k[0]*=kd; k[1]*=kd; k[2]*=kd;
    GLfloat ctheta, stheta, onemctheta;
    ctheta=cosf(hTheta); stheta=sinf(hTheta); onemctheta=1-ctheta;
    
    printf("Centre=[%f,%f,%f], Axis=[%f,%f,%f], Angle=%f\n", hXstart, hYstart, hZstart, k[0], k[1], k[2], hTheta);
    
    for (int i=0; i<nvertices; i++) {
        if (!isanchored[i]) {
            
            Xs[0]=objectModel_vertices[3*i]-hXstart;
            Xs[1]=objectModel_vertices[3*i+1]-hYstart;
            Xs[2]=objectModel_vertices[3*i+2]-hZstart;
            
            GLfloat vck[3];
            vCrossProduct(k, Xs, vck);
            GLfloat vdk=vDotProduct(k, Xs);
            
            Xe[0]=Xs[0]*ctheta+vck[0]*stheta+k[0]*vdk*(onemctheta);
            Xe[1]=Xs[1]*ctheta+vck[1]*stheta+k[1]*vdk*(onemctheta);
            Xe[2]=Xs[2]*ctheta+vck[2]*stheta+k[2]*vdk*(onemctheta);
            
            objectModel_vertices[3*i]=Xe[0]+hXstart;
            objectModel_vertices[3*i+1]=Xe[1]+hYstart;
            objectModel_vertices[3*i+2]=Xe[2]+hZstart;
            
        }
    }
}


#pragma mark --- setters and getters----

- (operationMode) getOperationMode
{
    return theOperationMode;
}

- (BOOL) isGroupsExists
{
    return groups != 0;
}

- (void) setDisplayGroupNum:(int)theGroupNum
{
    displayGroupNum=theGroupNum;
}
- (int) getDisplayGroupNum;
{
    return displayGroupNum;
}
- (int) getNumGroups;
{
    return ngroups;
}
- (BOOL) getIsSetInitTransform
{
    return issetInitTransform;
}


-(void)getCameraParametersFu:(float *)ptrfu Fv:(float *)ptrfv Uc:(float *)ptruc Vc:(float *)ptrvc
{
	*ptrfu=fu;
	*ptrfv=fv;
	*ptruc=uc;
	*ptrvc=vc;
}


- (void)getPoint:(GLfloat *)pt Type:(int)tt Index:(int)idx
{
	if (tt==0) { // contact point
		pt[0]=contactPoint[0]; pt[1]=contactPoint[1]; pt[2]=contactPoint[2];
	} else if (tt==1) { // before point
		//		pt[0]=before[idx][0]; pt[1]=before[idx][1]; pt[2]=before[idx][2];
	} else if (tt==2) { // after point
		//		pt[0]=after[idx][0]; pt[1]=after[idx][1]; pt[2]=after[idx][2];
	} else if (tt==3) { // center
		pt[0]=Xcentre; pt[1]=Ycentre; pt[2]=Zcentre;
	}
}

-(void)setPoint:(GLfloat *)pt Type:(int)tt Index:(int)idx
{
	if (tt==0) { // contact point
		contactPoint[0]=pt[0]; contactPoint[1]=pt[1]; contactPoint[2]=pt[2];
	} else if (tt==1) { // before point
		//		before[idx][0]=pt[0]; before[idx][1]=pt[1]; before[idx][2]=pt[2];
	} else if (tt==2) { // after point
		//		after[idx][0]=pt[0]; after[idx][1]=pt[1]; after[idx][2]=pt[2];
	} else if (tt==3) { // center
		Xcentre=pt[0]; Ycentre=pt[1]; Zcentre=pt[2];
	}
}

- (void)getAClickedPointGivenPos:(int)pos X:(GLfloat*)X Y:(GLfloat*)Y Z:(GLfloat*)Z
{
	getElementFromArray(&clickedPoints, pos, X, Y, Z);
}

- (int)getNumClickedPoints
{
	return clickedPoints.size;
}

- (void)clearClickedPoints
{
	clearArray(&clickedPoints);
}

-(int) getOpenglViewNumber
{
	return openglViewNumber;
}

- (void)getVertices:(GLfloat *)VV
{
	// here COPY over from objectModel_vertices into new memory location
	// following needs VV to be declared and initialized
	memcpy(VV, objectModel_vertices, 3*nvertices*sizeof(GLfloat));
}
- (void)setVertices:(GLfloat *)VV
{
    /*	// here POINT objectModel_vertices to new memory location
     if (objectModel_vertices)
     delete[] objectModel_vertices;
     objectModel_vertices=VV;
     */
	memcpy(objectModel_vertices, VV, 3*nvertices*sizeof(GLfloat));
	[self setNeedsDisplay:YES];
}
- (int)getNumVertices {
	return nvertices;
}
- (int)  getNumIndices { return nrayconstrained; }
- (void) setNumIndices:(int)n { nrayconstrained=n; }
- (void) getIndices:(int*)idxs {
	memcpy(idxs, rayconstrained_idxs, nrayconstrained*sizeof(int));
}
- (void) setIndices:(int*)idxs {
	memcpy(rayconstrained_idxs, idxs, nrayconstrained*sizeof(int));
}
- (void) getPtsConstrained:(GLfloat*)pts {
	memcpy(pts, pts2Drayconstrained, nrayconstrained*2*sizeof(GLfloat));
}
- (void) setPtsConstrained:(GLfloat*)pts {
	memcpy(pts2Drayconstrained, pts, nrayconstrained*2*sizeof(GLfloat));
}
- (void) setOperationMode:(operationMode)myOperationMode
{
    theOperationMode=myOperationMode;
}


# pragma mark --- OpenGL update ----

// ---------------------------------

// this can be a troublesome call to do anything heavyweight, as it is called on window moves, resizes, and display config changes.  So be
// careful of doing too much here.
// window resizes, moves and display changes (resize, depth and display config change)
- (void) update {
	[super update];
	[self setNeedsDisplay:YES];
}



// ---------------------------------

// per-window timer function, basic time based animation preformed here
- (void)animationTimer:(NSTimer *)timer
{
	//	BOOL shouldDraw = NO;
	//	time = CFAbsoluteTimeGetCurrent (); //reset time in all cases
	/*
	 // if we have current messages
	 if (((getElapsedTime () - msgTime) < gMsgPresistance) || ((getElapsedTime () - gErrorTime) < gMsgPresistance))
	 shouldDraw = YES; // force redraw
	 if (YES == shouldDraw)
	 // redraw now instead dirty to enable updates during live resize
	 */
}



// -------------------------------
- (BOOL)acceptsFirstResponder {  return YES;  }
- (BOOL)becomeFirstResponder  {  return  YES; }
- (BOOL)resignFirstResponder  {  return YES;  }

// ---------------------------------

- (void) awakeFromNib
{
	// start animation timer
	timer = [NSTimer timerWithTimeInterval:(1.0f/60.0f) target:self selector:@selector(animationTimer:) userInfo:nil repeats:YES];
	[[NSRunLoop currentRunLoop] addTimer:timer forMode:NSDefaultRunLoopMode];
	[[NSRunLoop currentRunLoop] addTimer:timer forMode:NSEventTrackingRunLoopMode]; // ensure timer fires during resize
}

- (void) undo {
	if (nverticesmemory>1) {
		nverticesmemory--;
		nrayconstrained--;
		memcpy(objectModel_vertices, &(verticesMemory[3*nvertices*(nverticesmemory-1)]), 3*nvertices*sizeof(GLfloat));
//		printf("nstores=%d\n", nverticesmemory);
	}
	[[NSNotificationCenter defaultCenter] postNotificationName:@"arapNotification" object:self];
	[self setNeedsDisplay:YES];
}

- (void) undoredostore {
	if (nverticesmemory<40) {
		memcpy(&(verticesMemory[3*nvertices*nverticesmemory]), objectModel_vertices, 3*nvertices*sizeof(GLfloat));
		nverticesmemory++;
		nstores=nverticesmemory;
//		printf("nstores=%d\n", nverticesmemory);
	} else {
        printf("Cannot store any more, you may want to quit now and save");
    }
}

- (void) redo {
	if (nverticesmemory<nstores) {
		memcpy(objectModel_vertices, &(verticesMemory[3*nvertices*nverticesmemory]), 3*nvertices*sizeof(GLfloat));
		nverticesmemory++;
        nrayconstrained++;
//		printf("nstores=%d\n", nverticesmemory);
	}
    [[NSNotificationCenter defaultCenter] postNotificationName:@"arapNotification" object:self];
	[self setNeedsDisplay:YES];
}

#pragma mark ---- Actions -----

- (IBAction)setTransfAction:(id)sender
{
    memcpy(lastRotation,objectRotation,16*sizeof(float));
}
- (IBAction)getTransfAction:(id)sender
{
    memcpy(objectRotation,lastRotation,16*sizeof(float));
    [self setNeedsDisplay:YES];
}


- (IBAction) duplicateAction:(id)sender
{
    if (openglViewNumber==2 && issetInitTransform && manipPhase) {
        //[btnRotate setState:NSOffState];
        //[btnTranslate setState:NSOffState];
        //[btnReplicate setState:NSOnState];
        //[btnScale setState:NSOffState];
        
        GLfloat mv[16];
        [self multiplyMatrices_M1:objectRotation M2:initTransform Mout:mv];
        multMatrix4SeveralVertices(mv, objectModel_vertices, &duplicatedVertices[3*nvertices*nduplicated], nvertices);
        duplicated[n_transforms]=true;
        nduplicated++;
        [self setNeedsDisplay:YES];
    }
}

- (IBAction)setNumIterations:(id)sender {
	NSString *mystring=[sender stringValue];
	numiterations=atoi([mystring UTF8String]);
	printf("Number of Iterations=%d\n",numiterations);
}

- (IBAction)setAnchorWeight:(id)sender {
	NSString *mystring=[sender stringValue];
	anchorWeight=atof([mystring UTF8String]);
	printf("anchorWeight=%f\n",anchorWeight);
}

- (IBAction)setSymmetryWeight:(id)sender {
	NSString *mystring=[sender stringValue];
	symmetryWeight=atof([mystring UTF8String]);
	printf("symmetryWeight=%f\n", symmetryWeight);
}

- (IBAction)setRayConstraintsWeight:(id)sender {
	NSString *mystring=[sender stringValue];
	rayConstraintsWeight=atof([mystring UTF8String]);
	printf("rayConstraintsWeight=%f\n", rayConstraintsWeight);
}

- (IBAction)setDeformationWeight:(id)sender {
	NSString *mystring=[sender stringValue];
	deformationWeight=atof([mystring UTF8String]);
}

- (IBAction)setAugWeight:(id)sender {
    NSString *mystring=[sender stringValue];
	augweight=atof([mystring UTF8String]);
}

- (IBAction)setPlanarityWeight:(id)sender {
    NSString *mystring=[sender stringValue];
	planarityWeight=atof([mystring UTF8String]);
}

- (IBAction)lockGroup:(id)sender {
	// should really be lock/unlock group
    if (theOperationMode==MODE_LASSODEFORM) {
        if (groups && displayGroupNum != 0) {
            deformgroupsLocked[displayGroupNum-1]=!deformgroupsLocked[displayGroupNum-1];
        }
    } else {
        if (groups && displayGroupNum != 0) {
            groupsLocked[displayGroupNum-1]=!groupsLocked[displayGroupNum-1];
        }
    }
    
    //nanchored=0;
	//nrayconstrained=0;
    //nrigid=0;
    //nlassodeformed=0;
	int i=0, j=0;
    for (i=0; i<nvertices; i++) {
		for (j=0; j<9; j++) {
			if (j==0 || j==8 || j==4) arapRotations[9*i+j]=1;
			else arapRotations[9*i+j]=0;
		}
	}
	//for (i=0; i<nvertices; i++) isanchored[i]=false;
	for (i=0; i<nvertices; i++) {
		if (groups && groups[i]==displayGroupNum) {
			//anchored_idxs[nanchored]=i; nanchored++;
            isanchored[i]=groupsLocked[displayGroupNum-1];
            
		}
        if (groups && groups[i]==displayGroupNum) {
            islassodeformed[i]=deformgroupsLocked[displayGroupNum-1];
        }
        //  if (isanchored[i]) printf("anchoredidx=%d\n", i);
	}
    //for (i=0; i<nvertices; i++) islassodeformed[i]=false;
	//for (i=0; i<nvertices; i++) {
	//	if (groups && deformgroupsLocked[groups[i]-1]) {
    //lassodeformed_idxs[nlassodeformed]=i; nlassodeformed++;
    //        islassodeformed[i]=true;
	//	}
    //   if (islassodeformed[i]) printf("lassodeformedidx=%d\n", i);
	//}
    hTheta=0;
    hmark=0;
    //theOperationMode=MODE_MARKDEFORM;
    [self setNeedsDisplay:YES];
}

- (IBAction)lockAll:(id)sender {
    if (groups) {
        if (!alllocked) {
            for (int i=0; i<ngroups; i++) groupsLocked[i]=true;
            alllocked=true;
        } else {
            for (int i=0; i<ngroups; i++) groupsLocked[i]=false;
            alllocked=false;
        }
    }
    //nanchored=0;
    int i=0;
    //for (i=0; i<nvertices; i++) isanchored[i]=false;
	for (i=0; i<nvertices; i++) {
		if (groups && groupsLocked[groups[i]-1]) {
			//anchored_idxs[nanchored]=i; nanchored++;
            isanchored[i]=true;
		}
	}
    theOperationMode=MODE_MARKDEFORM;
	[self setNeedsDisplay:YES];
}

- (IBAction)clearAMAction:(id)sender
{
	//nanchored=0;
    nrayconstrained=0;
    nrigid=0;
    nlassomovelist=0;
    //nlassodeformed=0;
	int i=0, j=0;
    for (i=0; i<nvertices; i++) isanchored[i]=false;
	for (i=0; i<nvertices; i++) {
		if (groups && groupsLocked[groups[i]-1]) {
			//anchored_idxs[nanchored]=i; nanchored++;
            isanchored[i]=true;
		}
	}
    for (i=0; i<nvertices; i++) islassodeformed[i]=false;
    numComponentsSelected=0;
    
    for (i=0; i<nvertices; i++)
    {
        isVertexSelected[i]=false;
    }
    
	//for (i=0; i<nvertices; i++) {
	//	if (groups && deformgroupsLocked[groups[i]-1]) {
    //lassodeformed_idxs[nlassodeformed]=i; nlassodeformed++;
    //        islassodeformed[i]=true;
	//	}
	//}
    
    /*
    if (workengine) {
        engEvalString(workengine, "if exist('Aaug','var') clear Aaug; end; if exist('baug','var') clear baug; end ");
    }
     */
    
    [self setNeedsDisplay:YES];
	
}

-(IBAction) zoomAction:(id)sender
{
	theOperationMode=MODE_ZOOM;
}

-(IBAction) deformAction:(id)sender
{
	if (theOperationMode==MODE_MARKDEFORM) {
		theOperationMode=MODE_DEFORM;
		//nanchored=0;
        for (int i=0; i<nvertices; i++) isanchored[i]=false;
		nrayconstrained=0;
        nrigid=0;
        [self setNeedsDisplay:YES];
	}
}


-(IBAction) markRigidAction:(id)sender
{
	theOperationMode=MODE_MARKRIGID;
}

- (IBAction)markHinge:(id)sender
{
    theOperationMode=MODE_MARKH;
}
- (IBAction)foldUp:(id)sender
{
    GLfloat thetainc=M_PI/18;
    if (theOperationMode==MODE_MARKH && hmark==2) {
        deforming=YES;
        hTheta+=thetainc;
        [self hingeMotion];
        [self undoredostore];
        [[NSNotificationCenter defaultCenter] postNotificationName:@"arapNotification" object:self];
        [self setNeedsDisplay:YES];
    }
}
- (IBAction)foldDown:(id)sender
{
    GLfloat thetainc=M_PI/18;
    if (theOperationMode==MODE_MARKH && hmark==2) {
        deforming=YES;
        hTheta-=thetainc;
        [self hingeMotion];
        [self undoredostore];
        [[NSNotificationCenter defaultCenter] postNotificationName:@"arapNotification" object:self];
        [self setNeedsDisplay:YES];
    }
}


-(IBAction)computeRigidAction:(id)sender
{
    if (theOperationMode==MODE_MARKRIGID) {
        //printf("nanchored=%d\n", nanchored);
        [self partBasedRigidTransform];
        [self setNeedsDisplay:YES];
        //nanchored=0;
        //for (int i=0; i<nvertices; i++) isanchored[i]=false;
        int i=0; int j=0;
		for (i=0; i<nvertices; i++) {
            for (j=0; j<9; j++) {
                if (j==0 || j==8 || j==4) arapRotations[9*i+j]=1;
                else arapRotations[9*i+j]=0;
            }
        }
        for (i=0; i<nvertices; i++) {
            if (groups && groupsLocked[groups[i]-1]) {
                //anchored_idxs[nanchored]=i; nanchored++;
                isanchored[i]=true;
            }
        }
        [self undoredostore];
        [[NSNotificationCenter defaultCenter] postNotificationName:@"arapNotification" object:self];
        nrayconstrained=0;
        nrigid=0;
        //nlassodeformed=0;
        //for (int i=0; i<nvertices; i++) islassodeformed[i]=false;
        for (i=0; i<nvertices; i++) {
            if (groups && deformgroupsLocked[groups[i]-1]) {
                //lassodeformed_idxs[nanchored]=i; nlassodeformed++;
                islassodeformed[i]=true;
            }
        }
        
    }
}

-(IBAction) movableAction:(id)sender
{
	theOperationMode=MODE_MARKDEFORM;
}

-(IBAction) renderAction:(id)sender
{
	//pbrtStart(Vertices, objectModel_faces, TextureVertices, objectModel_texfaces, vertexNormals,
	//		  nsamp, filtertype, filterhsize, sigma, direc, outputfilename);
	if (openglViewNumber==2) {
		
		string path=direc;
		int n_trans=transforms.size()/17;
		FILE* fid=fopen(path.append(name).append("/transforms_ui.dat").c_str(), "w");
		for (int i=0; i<transforms.size(); i++) {
			fwrite(&(transforms[i]), 1, sizeof(float), fid);
		}
		fclose(fid);
		
		path=direc;
		fid=fopen(path.append(name).append("/cameraplaneinfo.txt").c_str(),"w");
		fprintf(fid, "campars=[%f,%f,%f,%f]\ngroundaxis=[%f,%f,%f,%f]\n",
				fu,fv,uc,vc,groundAxis[0],groundAxis[1],groundAxis[2],groundAxis[3]);
		fclose(fid);
        
        if (nduplicated>0) {
            path=direc;
            path.append(name).append("/vertices_dup.dat");
            fid=fopen(path.c_str(), n_transforms==0 ? "w" : "a");
            fwrite(duplicatedVertices, 3*sizeof(float), nvertices*nduplicated, fid);
            fclose(fid);
        }
        
        float* endtransform=new float[16*(nduplicated+1)];
        if (nduplicated>0) {
            int kcount=0;
            for (int k=0; k<n_transforms; k++) {
                if (duplicated[k]) {
                    for (int i=0; i<16; i++) {
                        endtransform[16*kcount+i]=transforms[ 17*k+i ];
                    }
                    kcount++;
                }
            }
            printf("kcount=%d\n",kcount);
            for (int i=0; i<16; i++) {
                endtransform[16*kcount+i]=transforms[transforms.size()-17+i];
            }
        } else {
            for (int i=0; i<16; i++) {
                endtransform[i]=transforms[transforms.size()-17+i];
            }
        }
        for (int i=0; i<16; i++) {
            printf("%f, ", endtransform[i]);
        }
        float* outputimage=new float[3*(int)reswidth*(int)resheight];
        char* snumrender=new char[3];
        sprintf(snumrender, "%03d",numrender);
        
        trace( direc, string(snumrender), endtransform, outputimage, n_previous_primitives, nduplicated+1);
        
        int outputimageindex;
        int renderedimageindex;
        for (int w=0; w<(int)reswidth; w++) {
            for (int h=0; h<(int)resheight; h++) {
                outputimageindex=w*resheight+h;
                renderedimageindex=h*reswidth+w;
                memcpy(&objectModel_renderedImage[3*renderedimageindex], &outputimage[3*outputimageindex], 3*sizeof(float));
            }
        }
        
        numrender++;
        delete[] endtransform;
        delete[] outputimage;
        renderIsDone=YES;
        [btnRotate setEnabled:NO];
        [btnTranslate setEnabled:NO];
        [btnScale setEnabled:NO];
        [btnReplicate setEnabled:NO];
        [btnRender setEnabled:NO];
        [btnManipulate2 setEnabled:YES];
        
        [btnRotate setState:NSOffState];
        [btnTranslate setState:NSOffState];
        //[btnReplicate setState:NSOffState];
        [btnScale setState:NSOffState];

		[self setNeedsDisplay:YES];
	}
}

- (IBAction)selectAction:(id)sender
{
    theOperationMode=MODE_SELECT;
}
- (IBAction)selectionChoiceAction:(id)sender
{
    selectionAxesArePCs=!selectionAxesArePCs;
    if (numComponentsSelected>0) [self computeCentroidAndAxesForSelection];
    [self setNeedsDisplay:YES];
}

- (IBAction)smoothBetweenRigid:(id)sender
{
    smoothBetweenRigidMotions=!smoothBetweenRigidMotions;
    [self setNeedsDisplay:YES];
}

- (IBAction)rotateAction:(id)sender
{
	theOperationMode=MODE_ROTATE;
    [btnRotate setState:NSOnState];
    [btnTranslate setState:NSOffState];
    //[btnReplicate setState:NSOffState];
    [btnScale setState:NSOffState];
	
	//[self setNeedsDisplay:YES];
}
- (IBAction)lassoAction:(id)sender
{
	theOperationMode=MODE_LASSO;
	
}
- (IBAction)lassodeformAction:(id)sender
{
	theOperationMode=MODE_LASSODEFORM;
	[[NSNotificationCenter defaultCenter] postNotificationName:@"lassoDeformNotification" object:self];
}
- (IBAction)bboxAction:(id)sender
{
	theOperationMode=MODE_BBOX;
}
- (IBAction)pickPointsAction:(id)sender
{
	theOperationMode=MODE_PICKPOINTS;
    [[NSNotificationCenter defaultCenter] postNotificationName:@"pickPointsNotification" object:self];
	
}

- (IBAction)saveAction:(id)sender
{
	
	isBeingSaved=YES;
	[self setNeedsDisplay:YES];
}
- (IBAction)axisRotateAction:(id)sender
{
	theOperationMode=MODE_AXISROTATE;
	//[self setNeedsDisplay:YES];
}
- (IBAction)computeIntrinsicsAction:(id)sender
{
	theOperationMode=MODE_COMPUTEINTRINSICS;
}
-(IBAction)revertAction:(id)sender
{
	
	[self undo];
	//inpainted=FALSE;
	//resetToIdentity(trackBallRotation);
	//resetToIdentity(objectRotation);
	//[self setNeedsDisplay:YES];
	
}
-(IBAction) translateAction:(id)sender
{
	theOperationMode=MODE_TRANSLATE;
    [btnRotate setState:NSOffState];
    [btnTranslate setState:NSOnState];
    //[btnReplicate setState:NSOffState];
    [btnScale setState:NSOffState];

	//[self setNeedsDisplay:YES];
}
-(IBAction) scaleAction:(id)sender
{
	theOperationMode=MODE_SCALE;
    [btnRotate setState:NSOffState];
    [btnTranslate setState:NSOffState];
    //[btnReplicate setState:NSOffState];
    [btnScale setState:NSOnState];

}
-(IBAction) startManipAction:(id)sender
{
	[self manipulate];
    [btnRotate setEnabled:YES];
    [btnTranslate setEnabled:YES];
    [btnScale setEnabled:YES];
    [btnReplicate setEnabled:YES];
    [btnRender setEnabled:YES];
    [btnManipulate2 setEnabled:NO];
    [btnRotate setState:NSOffState];
    [btnTranslate setState:NSOffState];
    //[btnReplicate setState:NSOffState];
    [btnScale setState:NSOffState];

    [self setNeedsDisplay:YES];
}

- (IBAction)sxpa:(id)sender
{
	theOperationMode=MODE_SCALE;
	float delta=.02;
	float kappa=1.f+delta;
	
	for (int i=0; i<nvertices; i++) {
		objectModel_vertices[3*i]*=kappa;
	}
	[[NSNotificationCenter defaultCenter] postNotificationName:@"arapNotification" object:self];
	
	deforming=YES;
	[self setNeedsDisplay:YES];
}
- (IBAction)sxma:(id)sender
{
	theOperationMode=MODE_SCALE;
	float delta=.02;
	float kappa=1./(1.f+delta);
	for (int i=0; i<nvertices; i++) {
		objectModel_vertices[3*i]*=kappa;
	}
	[[NSNotificationCenter defaultCenter] postNotificationName:@"arapNotification" object:self];
	
	deforming=YES;
	[self setNeedsDisplay:YES];
}
- (IBAction)sypa:(id)sender
{
	theOperationMode=MODE_SCALE;
	float delta=.02;
	float kappa=1.f+delta;
	for (int i=0; i<nvertices; i++) {
		objectModel_vertices[3*i+1]*=kappa;
	}
	[[NSNotificationCenter defaultCenter] postNotificationName:@"arapNotification" object:self];
	
	deforming=YES;
	[self setNeedsDisplay:YES];
}
- (IBAction)syma:(id)sender
{
	theOperationMode=MODE_SCALE;
	float delta=.02;
	float kappa=1./(1.f+delta);
	for (int i=0; i<nvertices; i++) {
		objectModel_vertices[3*i+1]*=kappa;
	}
	[[NSNotificationCenter defaultCenter] postNotificationName:@"arapNotification" object:self];
	
	deforming=YES;
	[self setNeedsDisplay:YES];
}
- (IBAction)szpa:(id)sender
{
	theOperationMode=MODE_SCALE;
	float delta=.02;
	float kappa=1.f+delta;
	for (int i=0; i<nvertices; i++) {
		objectModel_vertices[3*i+2]*=kappa;
	}
	[[NSNotificationCenter defaultCenter] postNotificationName:@"arapNotification" object:self];
	
	deforming=YES;
	[self setNeedsDisplay:YES];
}
- (IBAction)szma:(id)sender
{
	theOperationMode=MODE_SCALE;
	float delta=.02;
	float kappa=1./(1.f+delta);
	for (int i=0; i<nvertices; i++) {
		objectModel_vertices[3*i+2]*=kappa;
	}
	[[NSNotificationCenter defaultCenter] postNotificationName:@"arapNotification" object:self];
	
	deforming=YES;
	[self setNeedsDisplay:YES];
}


@end
