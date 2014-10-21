/*
 New copyright for GNU Public License 2 goes here.
 */


// main/pbrt.cpp*
#include "definitions.h"
#include "helpers.h"
#include "composition.h"
#include <algorithm>
#include <vector>
#include <utility>



int trace(int argc, char *argv[]) {
	
	// You need Vertices, Faces, Pixels, Albedo, Ambient, Diffuse, Specular
	// nv, nt, npixels, ndivs
	
	// Declarations
	
	//testGeneral();
	//testCut();
	
	//RNG rng((uint)time(NULL));
	//bool hasSpecular=true;
	bool hasSpecular=false;
	
	int ndivs, ntheta, nphi, nsamp=1, nkernels;
	int xmin=0, xmax=0, ymin=1000, ymax=1000;
	int npixels, nt, nv, height, width,ntexv, i1, i2, i3, i, j, k, filtertype=GAUSS_FILTER, filterhsize;
	//int stex=512;
	
    // Parameters
    param parameters;
    parameters.lambda1=1e-3;
    parameters.lambda2=2.0;
    parameters.shadowweight=.04f;
    parameters.numIterations=1;
    parameters.groups_instead_of_color=true;
    parameters.lambda_rho=0;
    parameters.lambda_rho2=5.0;
    parameters.nstepappearance=50;
    parameters.nstepillumination=4;
    parameters.nstepgroundillumination=16;
    parameters.nransac=5000;
    parameters.nrounds=1;
    parameters.mrfgamma=.005;
    parameters.mrfbeta=.001;
    parameters.texturewinsize=6;
    parameters.ransacthresholdratio=.01;
    parameters.ransacappearancethreshold=.001;
    parameters.ransacnormalthreshold=.0;
    parameters.symstopnumber=5;
    parameters.symstopunseennumber=1;
    parameters.estimateAmbient=true;
    
	//float restheta=M_PI*1.0/(ntheta-1), resphi=M_PI*2.0/(nphi-1), sigma=1.0f;
	float restheta, resphi;
    float sigma=2.0f;
	float fu, fv, u0, v0, ifu, ifv;
	float x, y, z, inv_sigma_sq;
	int n_on=0;
	int n_transforms=71;
	float groundAxis[4];
	float o2w[4][4]={{1.,0.,0.,0.},{0.,1.,0.,0.},{0.,0.,1.,0.},{0.,0.,0.,1.}};
	float w2o[4][4]={{1.,0.,0.,0.},{0.,1.,0.,0.},{0.,0.,1.,0.},{0.,0.,0.,1.}};
	
	float eye[4][4]={{1.,0.,0.,0.},{0.,1.,0.,0.},{0.,0.,1.,0.},{0.,0.,0.,1.}};
	
	string path, outname, name="obj", outputfilename="outputs", direc="/Users/nkholgad/CMU/Research/Models/pbrt/";
	string transformsname="transforms";
	string deformedverts="vertices_ui";
	
	Vector v1, v2, n, reflection, albedo, groundAlbedo;
	Vector direction(.0,.0,1.0), lightDirection, color, endcolor;
	
	PbrtPoint origin(.0,.0,.0);
	PbrtPoint intersectionPt;
	
	FILE* fid, *fidinfo;
	TriangleMesh* trotmesh;
	BVHAccel* firstAccel;
	TriangleMesh* tmesh;
	BVHAccel* bvhAccel;	
	TriangleMesh* textmesh;
	BVHAccel* texAccel;
	
	Transform* obj2wld;
	Transform* wld2obj;
	Transform* eyedentity=new Transform(eye);
	int shapeIdOffset=1, texIdOffset=1;
	Reference<Texture<float> > alphaTex = NULL;
	Reference<Material> mtl;
	Ray r(origin,direction,0,INFINITY);
	Ray s(intersectionPt,lightDirection,0,INFINITY);
	Intersection* isect=new Intersection();
	Intersection* lisect=new Intersection();
	//float* Itex;//=new float[3*stex*stex];
	float* Itexartist;//=new float[3*stex*stex];
	float* Itexartistambient;
    float* Itexartistspecular=NULL;
    float* Itexdiff;
	
	vector<Reference<Primitive> > prims(1);
	vector<Reference<Primitive> > texprims(1);
	vector<Reference<Primitive> > rotprims(1); 
	//vector<float> colorsamples;
	//vector<float> pointsamples;
	
	//int textureType=TYPE_LINEAR;
	int lightType=TYPE_COLOR;
	//int albedoType=TYPE_MEDIAN;
	
    nsamp=1;
    filterhsize=(int)ceilf(3*sigma);
    
    
	if (argc>1) name=string(argv[1]);
	if (argc>2) direc=string(argv[2]);
	if (argc>3) outputfilename=string(argv[3]);
	if (argc>4) transformsname=string(argv[4]);
	if (argc>5) n_transforms=atoi(argv[5]);
    if (argc>6) lightType=atoi(argv[6]);
    if (argc>7) hasSpecular=atoi(argv[7])!=0;
    
	//if (argc>6)	nsamp=atoi(argv[6]);
	//if (argc>7) filtertype=atoi(argv[7]);
	//if (argc>9) lightType=atoi(argv[9]);
	//if (argc>10) textureType=atoi(argv[10]);
	//if (argc>11) albedoType=atoi(argv[11]);
	//if (argc>12) hasSpecular=atoi(argv[12])!=0;
	
    /*
	if (filtertype==GAUSS_FILTER) {
		if (argc>8) sigma=atof(argv[8]);
		filterhsize=(int)round(3*sigma);
	} else if (filtertype==BOX_FILTER) {
		if (argc>8) filterhsize=atoi(argv[8]);
		else filterhsize=3;
	}
     */
	direc.append(name).append("/");
	/*if (argc==1) printf("Object not provided, using %s\n", path.c_str());
	 if (argc==2)printf("Directory not provided, using %s\n", path.c_str());
	 if (argc<4) printf("Output file name not provided, using %s\n", outputfilename.c_str());
	 if (argc<5) printf("Number of samples not provided, using %d\n",nsamp);
	 if (argc<6) printf("Filter type not provided, using Gaussian filter\n");
	 if (argc<8) printf("Transforms file not provided, using transforms.dat\n");*/
	printf("Summary: Directory: %s,\n getting transforms from %s, putting output into %s,\n using %d samples with ",
		   direc.c_str(), transformsname.c_str(), outputfilename.c_str(), nsamp);
	printf("%s filter ", filtertype==GAUSS_FILTER ? "Gaussian" : "Box");
	printf("and filter half-size %d\n", filterhsize);
    printf("and lightType=%s\n", lightType==TYPE_COLOR ? "Color" : "GrayScale");
	inv_sigma_sq=1/(2*sigma*sigma);
	
	//Load info
	path=direc;	
	fidinfo=fopen(path.append("info.txt").c_str(), "r");
	if (!fidinfo) {
		printf("Invalid info file!\n");
		delete isect; delete lisect; delete obj2wld; delete wld2obj; 
		return -1;
	}
	fscanf(fidinfo, "numtri=%d\nnumverts=%d\nnumtexverts=%d\nndivs=%d\nnpixels=%d\nheight=%d\nbbox=[%d,%d,%d,%d]\n",
		   &nt,&nv,&ntexv,&ndivs,&npixels,&height,&xmin,&xmax,&ymin,&ymax);
	
	path=direc;
	fid=fopen(path.append("cameraplaneinfo.txt").c_str(), "r");
	if (!fid) {
		printf("Invalid camera plane info file!\n");
		delete isect; delete lisect; delete obj2wld; delete wld2obj; 
		return -1;
	}
	fscanf(fid, "campars=[%f,%f,%f,%f]\ngroundaxis=[%f,%f,%f,%f]\n",
		   &fu,&fv,&u0,&v0,&groundAxis[0],&groundAxis[1],&groundAxis[2],&groundAxis[3]);
	fclose(fid);
	
	printf("campars=[%f,%f,%f,%f], ground axis=[%f,%f,%f,%f]\n",fu,fv,u0,v0,groundAxis[0],groundAxis[1],groundAxis[2],groundAxis[3]);
	
	ifu=1/fu; ifv=1/fv;
	nkernels=ndivs;
	ntheta=(int)sqrtf(ndivs);
	nphi=(int)sqrtf(ndivs);
	restheta=M_PI*1.0/(ntheta); resphi=M_PI*2.0/(nphi);
	width=npixels/height;
	
	
	//Vector diffuseLight[ndivs]; // light sourcesdiffuseLight[ndivs*3];
	//Vector specularLight[ndivs];
	Vector** diffuseLight;
	Vector** specularLight;
	int nshines=hasSpecular ? 1 : 1;
    
	
	diffuseLight=new Vector*[nshines];
	specularLight=new Vector*[nshines];
	int ishine=0;
	for (ishine=0; ishine<nshines; ishine++) {
		diffuseLight[ishine]=new Vector[ndivs];
		specularLight[ishine]=new Vector[ndivs];
	}
	
	
	//Vector ambientLight(.1,.1,.1); // patrik
//	Vector ambientLight(.0,.0,.0); // taxi cab and fruits
    Vector ambientLight(-1,-1,-1);
	
	float Theta[ndivs], Phi[ndivs];		
	float sTheta[ndivs], cTheta[ndivs], sPhi[ndivs], cPhi[ndivs];
	PbrtPoint* Vertices=new PbrtPoint[nv];
	PbrtPoint* VerticesOriginal=new PbrtPoint[nv];
	PbrtPoint* TextureVertices=new PbrtPoint[ntexv];
	int* Faces=new int[3*nt]; // i'll use Faces, Triangles, and indices interchangeably
	int* TextureFaces=new int[3*nt]; // i'll use Faces, Triangles, and indices interchangeably
	Vector* vertexNormals=new Vector[nv];
	Vector* vertexNormalsTransformed=new Vector[nv];
	float* Ktranspose=new float[nkernels*nkernels];
	bool* B=new bool[nkernels*nkernels];
	
	//Load vertices
	path=direc;
	fid=fopen(path.append("verts.txt").c_str(), "r");	
	if (!fid) {
		printf("Invalid vertices file!\n");
		delete isect; delete lisect; delete obj2wld; delete wld2obj; 
		return -1;
	}	
	for (i=0; i<nv; i++) {
		fscanf(fid, "%f %f %f\n", &x, &y, &z);
		Vertices[i].x=x; Vertices[i].y=y; Vertices[i].z=z;		
	}
	fclose(fid);
	
	path=direc;
	fid=fopen(path.append("verts_original.txt").c_str(), "r");	
	if (!fid) {
		printf("Invalid vertices file! using other vers\n");
		for (i=0; i<nv; i++) {
			VerticesOriginal[i].x=Vertices[i].x; VerticesOriginal[i].y=Vertices[i].y; VerticesOriginal[i].z=Vertices[i].z;
		}
	}	
	for (i=0; i<nv; i++) {
		fscanf(fid, "%f %f %f\n", &x, &y, &z);
		VerticesOriginal[i].x=x; VerticesOriginal[i].y=y; VerticesOriginal[i].z=z;
	}
	fclose(fid);
	
	
	// Load faces
	path=direc;
	fid=fopen(path.append("faces.txt").c_str(), "r");	
	if (!fid) {
		printf("Invalid faces file!\n");
		delete isect; delete lisect; delete obj2wld; delete wld2obj; 
		return -1;
	}
	for (i=0; i<nt; i++) {
		fscanf(fid, "%d %d %d\n", &i1, &i2, &i3);
		Faces[3*i]=i1; Faces[3*i+1]=i2; Faces[3*i+2]=i3;
       // printf("%d, Verts=[%d,%d,%d]\n",nt, Faces[3*i], Faces[3*i+1], Faces[3*i+2]);
	}	
	fclose(fid);
		
	//Load texture vertices
	path=direc;
	fid=fopen(path.append("textureverts.txt").c_str(), "r");	
	if (!fid) {
		printf("Invalid vertices file!\n");
		delete isect; delete lisect; delete obj2wld; delete wld2obj; 
		return -1;
	}	
	for (i=0; i<ntexv; i++) {
		
		fscanf(fid, "%f %f\n", &x, &y);
		TextureVertices[i].x=x; TextureVertices[i].y=y; TextureVertices[i].z=1.f; // you should scale up/down the texture vertices before testing against texAccel
	}
	fclose(fid);
	
	// Load texture faces
	path=direc;
	fid=fopen(path.append("texturefaces.txt").c_str(), "r");	
	if (!fid) {
		printf("Invalid faces file!\n");
		delete isect; delete lisect; delete obj2wld; delete wld2obj; 
		return -1;
	}
	for (i=0; i<nt; i++) {
		fscanf(fid, "%d %d %d\n", &i1, &i2, &i3);
		TextureFaces[3*i]=i1; TextureFaces[3*i+1]=i2; TextureFaces[3*i+2]=i3;
         //{
           // printf("%d, TextureVerts=[%d,%d,%d]\n",nt, TextureFaces[3*i], TextureFaces[3*i+1], TextureFaces[3*i+2]);
        //}
	}	
	fclose(fid);
	
	// Setup normals
	int idx=0, idxprev=0, idxnext=0, jprev=0, jnext=0;
	for (i=0; i<nt; i++) {
		for (j=0; j<3; j++) {
			jprev=j-1; if (jprev<0) jprev=j+2;
			jnext=j+1; if (jnext>2) jnext=j-2;
			idx=3*i+j;
			idxprev=3*i+jprev;
			idxnext=3*i+jnext;
			v1=Vertices[Faces[idxnext]]-Vertices[Faces[idx]];
			v2=Vertices[Faces[idxprev]]-Vertices[Faces[idx]];
			n=Normalize(Cross(v1, v2));
			
			if (Faces[idx]==763) 
				printf("idxprev=%d, idx=%d, idxnext=%d, n=[%f,%f,%f]\n",Faces[idxprev],Faces[idx],Faces[idxnext],n.x,n.y,n.z);
			vertexNormals[Faces[idx]]+=n;
		}
	}
	for (i=0; i<nv; i++) {
		//printf("%d\n",i);		
		if (Dot(vertexNormals[i],vertexNormals[i])>1e-6) vertexNormals[i]=Normalize(vertexNormals[i]);
	}
	
	path=direc;
	fid=fopen(path.append("vertexnormals.dat").c_str(), "w");
	for (j=0; j<nv; j++) {
		fwrite(&vertexNormals[j].x, sizeof(float), 1, fid);
		fwrite(&vertexNormals[j].y, sizeof(float), 1, fid);
		fwrite(&vertexNormals[j].z, sizeof(float), 1, fid);
	}
	fclose(fid);
	
	// Set up sphere
	for (j=0; j<ndivs; j++) {
		Theta[j]=((j / nphi)+.5)*restheta;
		sTheta[j]=sinf(Theta[j]); cTheta[j]=cosf(Theta[j]);
		Phi[j]=((j % nphi)+.5)*resphi;
		sPhi[j]=sinf(Phi[j]); cPhi[j]=cosf(Phi[j]);
	}
	
	// Load spherical coefficients
	path=direc;
	int kappatry=600;
	int kappa=80;
	char* ndivs_kernelsizetry=new char[20];
	char* ndivs_kernelsize=new char[20];
	sprintf(ndivs_kernelsizetry, "KFloat%d_%d.dat",ndivs,kappatry);
    //sprintf(ndivs_kernelsizetry, "K_delta.dat");
	FILE* fidk=fopen(path.append(ndivs_kernelsizetry).c_str(),"r");
	if (!fidk) { 
		path=direc;
		sprintf(ndivs_kernelsize, "KFloat%d_%d.dat",ndivs,kappa);
		fidk=fopen(path.append(ndivs_kernelsize).c_str(),"r");
	}
    
    // Remember that you are reading a column-wise matrix stored out of MATLAB
    // as a row-wise matrix for BLAS computations here. This causes the K
    // of MATLAB to be Ktranspose here.
	fread(Ktranspose, nkernels*nkernels, sizeof(float), fidk);
	fclose(fidk);
    delete[] ndivs_kernelsize; delete[] ndivs_kernelsizetry;
	
	/*
	 double xx, xy, xz;
	 double mux, muy, muz;
	 double kappa=50.0f;
	 double C;
	 double p;
	 C=kappa/(2*M_PI*(exp(kappa)-exp(-kappa)));
	 
	 int hercount=0;
	 for (j=0; j<nkernels; j++) {
	 xx=sin((double)(Theta[j]))*cos((double)(Phi[j]));
	 xy=sin((double)(Theta[j]))*sin((double)(Phi[j]));
	 xz=cos((double)(Theta[j]));
	 for (k=0; k<nkernels; k++) {
	 // K[k,j]=K[k*nkernels+j] is the value of the j-th division for the k-th kernel
	 mux=sin((double)(Theta[k]))*cos((double)(Phi[k]));
	 muy=sin((double)(Theta[k]))*sin((double)(Phi[k]));
	 muz=cos((double)(Theta[k]));
	 p=exp(kappa*(mux*xx+muy*xy+muz*xz));
	 K[k*nkernels+j]=C*p*sin((double)(Theta[j]))*restheta*resphi;
	 //K[k*nkernels+j]=C*p*restheta*resphi;
	 }
	 }
	 
	 path=direc;
	 fid=fopen(path.append("K_xcode.dat").c_str(),"w");
	 fwrite(K, nkernels*nkernels, sizeof(double), fid);
	 fclose(fid);
	 */
	for (j=0; j<nkernels; j++) {
		for (k=0; k<nkernels; k++) {
			B[k*nkernels+j]=Ktranspose[k*nkernels+j]>1e-4;
			if (B[k*nkernels+j]) n_on++;
		}
	}
  	
	// open image
	path=direc;
	float* I=new float[npixels*3];
	float* Igroundtex=new float[3*npixels];
	bool* Igroundmask=new bool[npixels];
	bool* Ishadowmask=new bool[npixels];
    
    readImage(direc, "image.png", I);
    readImage(direc, "background.png", Igroundtex);
    readBinaryImage(direc, "groundMask.png", Igroundmask);
    readBinaryImage(direc, "shadowMask.png", Ishadowmask);
    
	//readFile(direc, I, npixels*3*sizeof(float), "image.dat");
	//readFile(direc, Igroundtex, npixels*3*sizeof(float), "groundtex.dat");
    
    //path=direc; FILE* fgnd=fopen(path.append("groundMaskpart.dat").c_str(), "r");
    //if (!fgnd) fgnd=fopen(path.append("groundMask.dat").c_str(), "r");
    //fread(Igroundmask, npixels*sizeof(float), 1, fgnd); fclose(fgnd);
    
	//readFile(direc, Igroundmask, npixels*sizeof(float), "groundMask.dat");
	//readFile(direc, Ishadowmask, npixels*sizeof(float), "shadowMask.dat");
	printf("Populated Image\n");
	
	
	// open artist texture(s)
	int nmaps=1;
	int nmaptypes;
	fscanf(fidinfo, "Maps(%d): ", &nmaptypes);
	int* mapwidths=new int[nmaps];
	int* mapheights=new int[nmaps];
	int* mapids=new int[nt];
	//Itexs=new float*[nmaps];
	//Itexorigs=new float*[nmaps];
	//Itexartists=new float*[nmaps];
	//Itexartistsambient=new float*[nmaps];
	//float** Itexdiffs=new float*[nmaps];
	char* mapnameholder=new char[30];
	int dmapwidth, dmapheight;
	int ntexv_d;
	
	//for (i=0; i<nmaps; i++) {
    i=0;
		path=direc;
		fscanf(fidinfo, "%s", mapnameholder);
		fscanf(fidinfo, "%d %d ",&mapwidths[i],&mapheights[i]);
		fid=fopen(path.append(mapnameholder).append(".dat").c_str(),"r");
		if (nmaptypes==2) {
			fscanf(fidinfo, "%d %d %d ",&dmapwidth,&dmapheight,&ntexv_d);
		} else {
			dmapwidth=mapwidths[i]; dmapheight=mapheights[i]; ntexv_d=ntexv;
		}
        Itexartist=new float[dmapwidth*dmapheight*3];
        Itexartistambient=new float[dmapwidth*dmapheight*3];
    if (hasSpecular) Itexartistspecular=new float[dmapheight*dmapwidth*3];
		//Itex=new float[dmapwidth*dmapheight*3];
		Itexdiff=new float[dmapwidth*dmapheight*3];
		fread(Itexartist, sizeof(float), 3*dmapwidth*dmapheight, fid);
		memcpy(Itexartistambient, Itexartist, 3*dmapwidth*dmapheight*sizeof(float));
#ifdef SPECULAR_GAIN
    if (hasSpecular) {
        for (i=0; i<dmapheight*dmapwidth*3; i++) {
            Itexartistspecular[i]=1.0f;
        }
    }
#else
        if (hasSpecular) memcpy(Itexartistspecular, Itexartist, 3*dmapwidth*dmapheight*sizeof(float));
#endif
		fclose(fid);
	//}
	
	int* TextureFaces_D=new int[3*nt];
	PbrtPoint* TextureVertices_D=new PbrtPoint[ntexv_d];
	
	// load texture vertices (difference)
	path=direc;
	fid=fopen(path.append("textureverts_difference.txt").c_str(), "r");	
	if (!fid) {
		for (i=0; i<ntexv_d; i++) {
			TextureVertices_D[i].x=TextureVertices[i].x; 
			TextureVertices_D[i].y=TextureVertices[i].y;
			TextureVertices_D[i].z=TextureVertices[i].z;
		}
	} else {
		for (i=0; i<ntexv_d; i++) {		
			fscanf(fid, "%f %f\n", &x, &y);
			TextureVertices_D[i].x=x; TextureVertices_D[i].y=y; TextureVertices_D[i].z=1.f; // you should scale up/down the texture vertices before testing against texAccel
		}
	}
	fclose(fid);
	
	// Load texture faces (difference)
	path=direc;
	fid=fopen(path.append("texturefaces_difference.txt").c_str(), "r");	
	if (!fid) {
		memcpy(TextureFaces_D, TextureFaces, 3*nt*sizeof(int));
	} else {
		for (i=0; i<nt; i++) {
			fscanf(fid, "%d %d %d\n", &i1, &i2, &i3);
			TextureFaces_D[3*i]=i1; TextureFaces_D[3*i+1]=i2; TextureFaces_D[3*i+2]=i3;
		}	
	}
	fclose(fid);
	
	
	for (i=0;i<dmapwidth*dmapheight*3;i++) {
		Itexdiff[i]=.0f; 
	}
    
	fclose(fidinfo);
    
    float* Idiffuse=new float[npixels*3];
    float* Iambient=new float[npixels*3];
    float* Ispecular=NULL;
    if (hasSpecular) Ispecular=new float[npixels*3];
    float* Idifference=new float[npixels*3];
    
    for (i=0; i<npixels*3; i++) {
        Idiffuse[i]=.0f; Iambient[i]=.0f; Idifference[i]=.0f; if (hasSpecular) Ispecular[i]=.0f;
    }
	
	path=direc;
	fid=fopen(path.append("mapids.dat").c_str(), "r");
	//for (i=0; i<nt; i++) {
	//	fscanf(fid, "%d\n",&mapids[i]);
	//}
	if (fid)
	{ fread(mapids, sizeof(int), nt, fid); 
		fclose(fid);
	} else {
		for (i=0; i<nt; i++) {
			mapids[0]=0;
		}
		fid=fopen(path.append("mapids.dat").c_str(), "w");
		fwrite(mapids, sizeof(int), nt, fid);
		fclose(fid);
	}
	
	
	// Load albedos
	path=direc;
	fid=fopen(path.append("albedo.dat").c_str(),"r");
	fread(&albedo.x, 1, sizeof(float), fid);
	fread(&albedo.y, 1, sizeof(float), fid);
	fread(&albedo.z, 1, sizeof(float), fid);
	fread(&groundAlbedo.x, 1, sizeof(float), fid);
	fread(&groundAlbedo.y, 1, sizeof(float), fid);
	fread(&groundAlbedo.z, 1, sizeof(float), fid);
	fclose(fid);
	Vector ambientGroundAlbedo(groundAlbedo);
	
	// do a quick conversion over here
	//textureMapUniquifier(TextureVertices_D, TextureFaces_D, TextureVertices, TextureFaces, ntexv_d, ntexv, nt, Itexs[0], Itexartist, 
	//					 dmapwidth, dmapheight, dmapwidth, dmapheight, mapids, albedo);
	//path=direc; fid=fopen(path.append("artisttexture_uniquified.dat").c_str(), "w"); fwrite(Itexs[0], dmapwidth*dmapheight*3, sizeof(float), fid); fclose(fid);
	
	
	// compute adjacencies
	
    int* adjacencies=NULL;
    int* fadjacencies=NULL;
	//int* adjacencies=new int[9*nt];
	//int* fadjacencies=new int[9*nt];
	//computeFaceAdjacency(Faces, TextureFaces, fadjacencies, adjacencies, nt);
	
	path=direc;
	float* Btex;
	
	fid=fopen(path.append("B.dat").c_str(), "r"); 
	if (fid) {
		Btex=new float[dmapheight*dmapheight];
		fread(Btex, sizeof(float), dmapheight*dmapwidth, fid); 
		fclose(fid);
	} else {
		Btex=NULL;
	}
	
	
	// read in transforms
	//float blendfactors[n_transforms];
	//path=direc;
	//fid=fopen(path.append("sig.dat").c_str(), "r");
	//fread(blendfactors, n_transforms, sizeof(float), fid);
	//fclose(fid);
	
	//float* C_ugnv=new float[(ntheta+1)*(nphi+1)];
	//float* C_v=new float[ntheta+1];
	//float* P_uv=new float[ndivs];
	
	
	char* si=new char[3];
	path=direc;    
    
	FILE* fidtransforms=fopen(path.append(transformsname.c_str()).append(".dat").c_str(),"r"); 
	path=direc;
	FILE* fidvdef=fopen(path.append(deformedverts.c_str()).append(".dat").c_str(), "r");
	
	printf("%s\n", path.c_str());
	float transf[16];
	
	path=direc;
	
	Vector nn;
	float sigmoidd;
	float sigmoidscale=10;
	float blendfactor;
	float defval;
	bool bdefval;
	
	
	float o2w0[4][4];
	
	int ishinebest=0;
	
	
	float SphereRadius;
	int ncount=0; 
	
	int iview=0;
	
	float o2wView[4][4];
	float w2oView[4][4];
	
	float* transforms=new float[n_transforms*16];
	bool* bdefvals=new bool[n_transforms];
	
	int ndef=0;
	
	for (i=0; i<n_transforms; i++) {
		fread(&transforms[16*i],sizeof(float),16,fidtransforms);
		fread(&defval,sizeof(float),1,fidtransforms);
		bdefvals[i]=defval>.5;
		if (bdefvals[i])
			ndef++;
	}
	
	float* VerticesDef=new float[3*ndef*nv];
	fread(VerticesDef, sizeof(float), 3*nv*ndef, fidvdef);
	
	i=iview;
	
	o2wView[0][0]=transforms[16*i+0]; o2wView[1][0]=transforms[16*i+1]; o2wView[2][0]=transforms[16*i+2]; o2wView[3][0]=transforms[16*i+3];
	o2wView[0][1]=transforms[16*i+4]; o2wView[1][1]=transforms[16*i+5]; o2wView[2][1]=transforms[16*i+6]; o2wView[3][1]=transforms[16*i+7];
	o2wView[0][2]=transforms[16*i+8]; o2wView[1][2]=transforms[16*i+9]; o2wView[2][2]=transforms[16*i+10]; o2wView[3][2]=transforms[16*i+11];
	o2wView[0][3]=transforms[16*i+12]; o2wView[1][3]=transforms[16*i+13]; o2wView[2][3]=transforms[16*i+14]; o2wView[3][3]=transforms[16*i+15];
	
	w2oView[0][0]=transforms[16*i+0]; w2oView[0][1]=transforms[16*i+1]; w2oView[0][2]=transforms[16*i+2];	w2oView[0][3]=-(transforms[16*i+0]*transforms[16*i+12]+transforms[16*i+1]*transforms[16*i+13]+transforms[16*i+2]*transforms[16*i+14]);
	w2oView[1][0]=transforms[16*i+4]; w2oView[1][1]=transforms[16*i+5]; w2oView[1][2]=transforms[16*i+6];  w2oView[1][3]=-(transforms[16*i+4]*transforms[16*i+12]+transforms[16*i+5]*transforms[16*i+13]+transforms[16*i+6]*transforms[16*i+14]);
	w2oView[2][0]=transforms[16*i+8]; w2oView[2][1]=transforms[16*i+9]; w2oView[2][2]=transforms[16*i+10]; w2oView[2][3]=-(transforms[16*i+8]*transforms[16*i+12]+transforms[16*i+9]*transforms[16*i+13]+transforms[16*i+10]*transforms[16*i+14]);
	w2oView[3][0]=0.0f; w2oView[3][1]=0.f; w2oView[3][2]=0.f; w2oView[3][3]=1.f; 
	
	Transform* obj2wldView=new Transform(o2wView);
	Transform* wld2objView=new Transform(w2oView);
	
	
	for (int j=0; j<4; j++) {
		for (int k=0; k<4; k++) {
			printf("%f ", o2wView[j][k]);
		}
		printf("\n");
	}
	
	int* FacesFirst=new int[nt*3];
	PbrtPoint* VerticesFirst=new PbrtPoint[nt];
    PbrtPoint* VerticesFirstTransformed=new PbrtPoint[nt];
	memcpy(FacesFirst, Faces, 3*nt*sizeof(int));
	
	for (k=0; k<nv; k++) {
		VerticesFirst[k].x=VerticesDef[iview*3*nv+3*k];
		VerticesFirst[k].y=VerticesDef[iview*3*nv+3*k+1];
		VerticesFirst[k].z=VerticesDef[iview*3*nv+3*k+2];
	}
	
	
	//int textureType=TYPE_LINEAR_NORMAL;
	int ndefcount=0;
    
    
    bool duplicate=true;
    int* FacesDup;
    PbrtPoint* VerticesDup;
    Vector* vertexNormalsTransformedDup;
    float w2od[4][4];
    int dupstart=1;
    int ndups=n_transforms-dupstart;
	
    float* transfs=new float[16];
    float xx, yy, zz;
    
    
    float* PixelSpaceAppearance=new float[npixels*3];
    IplImage* binaryImage=cvCreateImage(cvSize(height,npixels/height), IPL_DEPTH_32F,1);
	IplImage* erodedImage=cvCreateImage(cvSize(height,npixels/height), IPL_DEPTH_32F,1);
	IplImage* middilatedImage=cvCreateImage(cvSize(height,npixels/height), IPL_DEPTH_32F,1);
    float** diffuseLightOutput=new float*[1];
	float** groundNoObjLightOutput=new float*[1];
	diffuseLightOutput[0]=new float[3*npixels];
	groundNoObjLightOutput[0]=new float[3*npixels];
	
	for (i=0; i<n_transforms; i++)
	{
		outname=outputfilename;
		sprintf(si, "%03d",i);
		outname.append(si).append(".png");
		
		memcpy(transf, &transforms[16*i], 16*sizeof(float));
		bdefval=bdefvals[i];
		
		
			
			o2w[0][0]=transf[0]; o2w[1][0]=transf[1]; o2w[2][0]=transf[2]; o2w[3][0]=transf[3];
			o2w[0][1]=transf[4]; o2w[1][1]=transf[5]; o2w[2][1]=transf[6]; o2w[3][1]=transf[7];
			o2w[0][2]=transf[8]; o2w[1][2]=transf[9]; o2w[2][2]=transf[10]; o2w[3][2]=transf[11];
			o2w[0][3]=transf[12]; o2w[1][3]=transf[13]; o2w[2][3]=transf[14]; o2w[3][3]=transf[15];
			
			
			
			w2o[0][0]=transf[0]; w2o[0][1]=transf[1]; w2o[0][2]=transf[2];	w2o[0][3]=-(transf[0]*transf[12]+transf[1]*transf[13]+transf[2]*transf[14]);
			w2o[1][0]=transf[4]; w2o[1][1]=transf[5]; w2o[1][2]=transf[6];  w2o[1][3]=-(transf[4]*transf[12]+transf[5]*transf[13]+transf[6]*transf[14]);
			w2o[2][0]=transf[8]; w2o[2][1]=transf[9]; w2o[2][2]=transf[10]; w2o[2][3]=-(transf[8]*transf[12]+transf[9]*transf[13]+transf[10]*transf[14]);
			w2o[3][0]=0.0f; w2o[3][1]=0.f; w2o[3][2]=0.f; w2o[3][3]=1.f; 
			
			obj2wld=new Transform(o2w);
			wld2obj=new Transform(w2o);
			
			
			if (bdefval) {
				// this means that the vertices have been deformed, so read in the deformed vertices from vertices_ui
				printf("DEFORMATION\n");
				for (k=0; k<nv; k++) {
					Vertices[k].x=VerticesDef[ndefcount*3*nv+3*k];
					Vertices[k].y=VerticesDef[ndefcount*3*nv+3*k+1];
					Vertices[k].z=VerticesDef[ndefcount*3*nv+3*k+2];
				}
				ndefcount++;
											  
				for (k=0; k<nv; k++) {
                    vertexNormals[k]=Vector(.0f,.0f,.0f);
                }
                
				// also redo the normals
				for (k=0; k<nt; k++) {
					for (j=0; j<3; j++) {
						jprev=j-1; if (jprev<0) jprev=j+2;
						jnext=j+1; if (jnext>2) jnext=j-2;
						idx=3*k+j;
						idxprev=3*k+jprev;
						idxnext=3*k+jnext;
						v1=Vertices[Faces[idxnext]]-Vertices[Faces[idx]];
						v2=Vertices[Faces[idxprev]]-Vertices[Faces[idx]];
						n=Normalize(Cross(v1, v2));
						vertexNormals[Faces[idx]]+=n;
					}
				}
				printf("DEFORMATION done");
				for (k=0; k<nv; k++) if (Dot(vertexNormals[k],vertexNormals[k])>1e-6) vertexNormals[k]=Normalize(vertexNormals[k]);
				
				Vector up(.0f,1.0f,.0f);
				
			}
			
			
			// Set up triangle mesh
			
			//	if (i==0 || i==90 || i==126 || i==184 || i==368 || i==539 || i==864 || i==932 || i==1131 || i==1202 || i==1310 || i==1462
			//		|| i==1506 || i==1532 || i==2040 || i==2116 || i==2398 || i==2778 || i==3639 || i==4469 || i==4550) {  // Evals, Mango1
			
			if (i==0 && duplicate) {
                VerticesDup=new PbrtPoint[nv*(ndups)];
                FacesDup=new int[3*nt*(ndups)];
                vertexNormalsTransformedDup=new Vector[nv*ndups];
                
                
                int ii=0;
                for (int iistart=dupstart; iistart<n_transforms; iistart++) {
                    ii=iistart-dupstart;
                    memcpy(transfs, &transforms[16*iistart], 16*sizeof(float));
                    for (int jj=0; jj<nv; jj++) {
                        VerticesDup[ii*nv+jj].x=transfs[0]*Vertices[jj].x+transfs[4]*Vertices[jj].y+transfs[8]*Vertices[jj].z+transfs[12];
                        VerticesDup[ii*nv+jj].y=transfs[1]*Vertices[jj].x+transfs[5]*Vertices[jj].y+transfs[9]*Vertices[jj].z+transfs[13];
                        VerticesDup[ii*nv+jj].z=transfs[2]*Vertices[jj].x+transfs[6]*Vertices[jj].y+transfs[10]*Vertices[jj].z+transfs[14];
                        
                        if (jj==0) {
                            //printf("Verts=[%f,%f,%f], VertsDup=[%f,%f,%f]\n", Vertices[jj].x,Vertices[jj].y,Vertices[jj].z,VerticesDup[ii*nv+jj].x,VerticesDup[ii*nv+jj].y,VerticesDup[ii*nv+jj].z);
                            //for (int kk=0; kk<16; kk++) {
                            //    printf("%f, ", transfs[kk]);
                            //}
                            //printf("\n");
                        }
                        
                        
                        w2od[0][0]=transfs[0]; w2od[0][1]=transfs[1]; w2od[0][2]=transfs[2];  w2od[0][3]=-(transfs[0]*transfs[12]+transfs[1]*transfs[13]+transfs[2]*transfs[14]);
                        w2od[1][0]=transfs[4]; w2od[1][1]=transfs[5]; w2od[1][2]=transfs[6];  w2od[1][3]=-(transfs[4]*transfs[12]+transfs[5]*transfs[13]+transfs[6]*transfs[14]);
                        w2od[2][0]=transfs[8]; w2od[2][1]=transfs[9]; w2od[2][2]=transfs[10]; w2od[2][3]=-(transfs[8]*transfs[12]+transfs[9]*transfs[13]+transfs[10]*transfs[14]);
                        w2od[3][0]=0.0f; w2od[3][1]=0.f; w2od[3][2]=0.f; w2od[3][3]=1.f;
                        
                        
                        nn.x=vertexNormals[jj].x*w2od[0][0]+vertexNormals[jj].y*w2od[1][0]+vertexNormals[jj].z*w2od[2][0]+w2od[3][0];
                        nn.y=vertexNormals[jj].x*w2od[0][1]+vertexNormals[jj].y*w2od[1][1]+vertexNormals[jj].z*w2od[2][1]+w2od[3][1];
                        nn.z=vertexNormals[jj].x*w2od[0][2]+vertexNormals[jj].y*w2od[1][2]+vertexNormals[jj].z*w2od[2][2]+w2od[3][2];
                        if (Dot(nn,nn)>1e-6) vertexNormalsTransformedDup[ii*nv+jj]=Normalize(nn);
                    }
                    
                   // printf("maxf=%d, nv*ii=%d\n",Faces[3*3095],nv*ii);
                    
                    for (int jj=0; jj<nt; jj++) {
                        FacesDup[3*(ii*nt+jj)]=Faces[3*jj]+(nv*ii);
                        FacesDup[3*(ii*nt+jj)+1]=Faces[3*jj+1]+(nv*ii);
                        FacesDup[3*(ii*nt+jj)+2]=Faces[3*jj+2]+(nv*ii);
                    }
                }
                
                path=direc; FILE* fvout=fopen(path.append("VerticesDup.dat").c_str(), "w");
                path=direc; FILE* fvnout=fopen(path.append("vertexNormalsDup.dat").c_str(), "w");
                for (int ii=0; ii<ndups*nv; ii++) {
                    fwrite(&VerticesDup[ii].x,1,sizeof(float),fvout);
                    fwrite(&VerticesDup[ii].y,1,sizeof(float),fvout);
                    fwrite(&VerticesDup[ii].z,1,sizeof(float),fvout);
                    fwrite(&vertexNormalsTransformedDup[ii].x,1,sizeof(float),fvnout);
                    fwrite(&vertexNormalsTransformedDup[ii].y,1,sizeof(float),fvnout);
                    fwrite(&vertexNormalsTransformedDup[ii].z,1,sizeof(float),fvnout);
                }
                fclose(fvout); fclose(fvnout);
                
                path=direc; FILE* ffout=fopen(path.append("FacesDup.dat").c_str(), "w");
                fwrite(FacesDup, 3*nt*ndups*sizeof(int), 1, ffout);
                fclose(ffout);
            }
			
			if (i==0) {				
				trotmesh=new TriangleMesh(obj2wldView,wld2objView,false,nt,nv,FacesFirst,VerticesFirst,NULL,NULL,NULL,alphaTex); // 1+nt prims
				Reference<Primitive> rotprim=new GeometricPrimitive(trotmesh, mtl, NULL); // 1 prim
				rotprims[0]=rotprim;
				firstAccel=new BVHAccel(rotprims);
                
                for (k=0; k<nv; k++) {
                    xx=VerticesFirst[k].x; yy=VerticesFirst[k].y; zz=VerticesFirst[k].z;
                    VerticesFirstTransformed[k].x=o2w[0][0]*xx+o2w[0][1]*yy+o2w[0][2]*zz+o2w[0][3];
                    VerticesFirstTransformed[k].y=o2w[1][0]*xx+o2w[1][1]*yy+o2w[1][2]*zz+o2w[1][3];
                    VerticesFirstTransformed[k].z=o2w[2][0]*xx+o2w[2][1]*yy+o2w[2][2]*zz+o2w[2][3];
                }
			}
			
			if (duplicate && i>=dupstart) {
                tmesh=new TriangleMesh(eyedentity,eyedentity,false,nt*( (i-dupstart)+1),nv*( (i-dupstart)+1),FacesDup,VerticesDup,NULL,NULL,NULL,alphaTex); // 1+nt prims                
            } else {
                tmesh=new TriangleMesh(obj2wld,wld2obj,false,nt,nv,Faces,Vertices,NULL,NULL,NULL,alphaTex); // 1+nt prims
            }
			
			textmesh=new TriangleMesh(eyedentity,eyedentity,false,nt,ntexv_d,TextureFaces_D,TextureVertices_D,NULL,NULL,NULL,alphaTex); // 1+nt prims
            if (duplicate && i>=dupstart) {
                shapeIdOffset=( ncount*nt + dupstart*nt + ((ncount-dupstart)*(ncount-dupstart+1)/2)*nt)+2*(ncount+1)+1+(nt+1);
                texIdOffset=( ncount*nt + dupstart*nt + ((ncount-dupstart)*(ncount-dupstart+1)/2)*nt + ncount*nt)+2*(ncount+1)+1+(nt+1); // last nt+1 is for trotmesh
            } else {
                shapeIdOffset=(ncount*nt) + (ncount*nt) +2*(ncount+1)+1+(nt+1); // previous textures + previous shapes + offsets
                texIdOffset=(ncount*nt) + (ncount*nt) + nt + 2*(ncount+1)+1+(nt+1);// last nt+1 is for trotmesh, previous textures + previous shapes + current shape + offsets
            }

			ncount++;
			
			// Set up primitives and acceleration structure
			Reference<Primitive> prim=new GeometricPrimitive(tmesh, mtl, NULL); // 1 prim
			prims[0]=prim;
			bvhAccel=new BVHAccel(prims);
			
			Reference<Primitive> texprim=new GeometricPrimitive(textmesh,mtl,NULL);
			texprims[0]=texprim;
			texAccel=new BVHAccel(texprims);
			//}
			//printf("Setup done\n");
			
			//printf("Image num = %d, Tmesh ShapeId=%d, tmesh_id_diff=%d, shapeIdOffset=%d, shapeiddiff=%d, nt*i=%d, TmeshShapeId-2*nt*(i+1)=%d\n", i, tmesh->shapeId, tmesh->shapeId-idxprev, shapeIdOffset, shapeIdOffset-shapeidprev, nt*i, -2*nt*(i+1)+tmesh->shapeId);
			
			for (j=0; j<nv; j++) {
				nn.x=vertexNormals[j].x*w2o[0][0]+vertexNormals[j].y*w2o[1][0]+vertexNormals[j].z*w2o[2][0]+w2o[3][0];
				nn.y=vertexNormals[j].x*w2o[0][1]+vertexNormals[j].y*w2o[1][1]+vertexNormals[j].z*w2o[2][1]+w2o[3][1];
				nn.z=vertexNormals[j].x*w2o[0][2]+vertexNormals[j].y*w2o[1][2]+vertexNormals[j].z*w2o[2][2]+w2o[3][2];
				if (Dot(nn,nn)>1e-6) vertexNormalsTransformed[j]=Normalize(nn);
			}
			
            
            path=direc;
            fid=fopen(path.append("vertexnormals_transf.dat").c_str(), "w");
            for (j=0; j<nv; j++) {
                fwrite(&vertexNormals[j].x, sizeof(float), 1, fid);
                fwrite(&vertexNormals[j].y, sizeof(float), 1, fid);
                fwrite(&vertexNormals[j].z, sizeof(float), 1, fid);
            }
            fclose(fid);

            
			// For each pixel in the convex hull of the object,
			//printf("Number of pixels: %d, number of divisions=%d\n", npixels, ndivs);
						
			if (i==0) {
				for (j=0; j<4; j++) {
					for (k=0; k<4; k++) {
						o2w0[j][k]=o2w[j][k];
					}
				}
			}
			
			sigmoidd=.0f;
			for (j=0; j<4; j++) {
				for (k=0; k<4; k++) {
					if (k==3) {
						sigmoidd+=(o2w[j][k]-o2w0[j][k])/100*(o2w[j][k]-o2w0[j][k])/100;
					} else {
						sigmoidd+=(o2w[j][k]-o2w0[j][k])*(o2w[j][k]-o2w0[j][k]);
					}
				}
			}
			
			// blendfactor of 1 means algorithm, blendfactor of 0 means original image
			blendfactor=2*(1./(1+expf(-sigmoidscale*sigmoidd))-.5);
			
			
			//blendfactor=1.0f;
			
			path=direc; fid=fopen(path.append("diffuseLight.dat").c_str(),"r");
			for (j=0; j<ndivs; j++) {
				fread(&diffuseLight[0][j].x, 1, sizeof(float), fid);
				fread(&diffuseLight[0][j].y, 1, sizeof(float), fid);
				fread(&diffuseLight[0][j].z, 1, sizeof(float), fid);
			} 
			fclose(fid);
            path=direc; fid=fopen(path.append("ambientLight.dat").c_str(), "r");
            fread(&ambientLight.x,1,sizeof(float),fid);
            fread(&ambientLight.y,1,sizeof(float),fid);
            fread(&ambientLight.z,1,sizeof(float),fid);
            fclose(fid);

            path=direc; fid=fopen(path.append("Itexdiff.dat").c_str(),"r");
			fread(Itexdiff, 3*dmapwidth*dmapheight, sizeof(float), fid); fclose(fid);
			path=direc; fid=fopen(path.append("Itexartist.dat").c_str(),"r");
			fread(Itexartist, 3*dmapwidth*dmapheight, sizeof(float), fid); fclose(fid);
             
			path=direc; fid=fopen(path.append("Igroundtex.dat").c_str(),"r");
			fread(Igroundtex, 3*npixels, sizeof(float), fid); fclose(fid);
			path=direc; fid=fopen(path.append("Idiffuse.dat").c_str(),"r");
			fread(PixelSpaceAppearance, 3*npixels, sizeof(float), fid); fclose(fid);
			path=direc; fid=fopen(path.append("Idifference.dat").c_str(),"r");
			fread(Idifference, 3*npixels, sizeof(float), fid); fclose(fid);
			path=direc; fid=fopen(path.append("groundAlbedo.dat").c_str(), "r");
			fread(&groundAlbedo.x, sizeof(float), 1, fid);
			fread(&groundAlbedo.y, sizeof(float), 1, fid);
			fread(&groundAlbedo.z, sizeof(float), 1, fid);
			fclose(fid);
			path=direc; fid=fopen(path.append("ambientGroundAlbedo.dat").c_str(), "r");
			fread(&ambientGroundAlbedo.x, sizeof(float), 1, fid);
			fread(&ambientGroundAlbedo.y, sizeof(float), 1, fid);
			fread(&ambientGroundAlbedo.z, sizeof(float), 1, fid);
			fclose(fid);
			
			printf("Image num = %d\n", i);
            
            //delete[] Itexorig;
			
			//if ( (i%8)==0) {
			
			//if (i==0 || i==90 || i==126 || i==184 || i==368 || i==539 || i==864 || i==932 || i==1131 || i==1202 || i==1310 || i==1462
			//	|| i==1506 || i==1532 || i==2040 || i==2116 || i==2398 || i==2778 || i==3639 || i==4469 || i==4550) {  // Evals, Mango1
			//if (i==0 || i==3 || i==5) {
     //       if (i>0 && (i<4 || (i>=4 && i%3==0))) {
            //if (i==5) {
            if (true) {
				
				//if (i==0 || i==78 || i==233 || i==273 || i==325 || i==380 || i==500 || i==648 || i==746 || i==997 || i==1988 || i==2400
				//	|| i==2715 || i==2846 || i==2980 || i==3107 || i==3471 || i==3697 || i==3738 || i==3771 || i==3811 || i==3879 || i==3917 || i==4124) { // Evals, Apple3
				
				//if ( i==250 || i==300 || i==350 || i==400 || i==450 || i==500 || i==550 || i==600 || i==650 || i==700 || i==750 || i==800 || i==850 ) {
				//if ( i==124 || i==437 || i==641) {
				//	if (i==0) {
				//if (i==0 || i==n_transforms-1) {
				//if (i==0 || i==163 || i==239 || i==332 || i==470) {// || i==1018 || i==1477) {
				//if (i==0 || i==140 || i==233 || i==282 || i==560 || i==681) {
				//	if (i==0 || i==104 || i==170 || i==302 || i==449 || i==596 || i==813 || i==1248) {
				//if (i==1018 || i==1477) {
				//if (i==150) {
				//if (i==0 || i==50 || i==124 || i==200 || i==300 || i==350 || i==435 || i==480 || i==520 || i==590 || i==640) { // apple previous try
				//if (i==0 || i==169 || i==357 || i==419 || i==731 || i==1050 || i==1217 || i==1271 || i==1457 || i==1657 || i==1854 || i==1920 || i==2273) {	// taxi
				//if (i==0 || i==92 || i==125 || i==183 || i==245 || i==305 || i==380 || i==435 || i==490 || i==628 || i==859 || i==1182 || i==1558 || i==1875 || i==2236) { // apple next try
				//if (i==0 || i==144 || i==288 || i==300 || i==450 || i==580 || i==700 || i==800 || i==1000 || i==1200 || i==1300 || i==1469) { // chair
				//if (i==0 || i==59 || i==95 || i==124 || i==174 || i==200 || i==255 || i==336 || i==448 || i==487 || i==530 || i==659 || i==700 || i==745 || i==790) {
				//if (false) {
				printf("Manipulation phase\n");
				
				
					composition(bvhAccel, (duplicate && i>=dupstart) ? vertexNormalsTransformedDup : vertexNormalsTransformed, (duplicate && i>=dupstart) ? FacesDup : Faces,
									  TextureVertices, TextureFaces, TextureVertices_D, TextureFaces_D,adjacencies,// geometry
									  I, Itexartist, Itexdiff,  Btex, blendfactor, Igroundtex, Igroundmask,
                                      PixelSpaceAppearance, Idifference,firstAccel,VerticesFirstTransformed,
									  u0,  v0,  ifu,  ifv, groundAxis,// camera parameters
									  cTheta, sTheta, cPhi, sPhi, SphereRadius, diffuseLight[ishinebest], ambientLight,// light sphere parameters
									  albedo, groundAlbedo,ambientGroundAlbedo,
									  width, height, nt, nkernels,  ndivs,  nsamp,  shapeIdOffset,dmapwidth, dmapheight,
									  filterhsize,  filtertype,  inv_sigma_sq, // filter parameters
									  path,  direc,  outname);
				
				
			}
			
			delete bvhAccel;
			delete texAccel;
			delete obj2wld; 
			delete wld2obj;
		}
		
		
	
	fclose(fidtransforms);
	if (fidvdef) fclose(fidvdef);
	
	/*
	Engine* workengine=engOpen("matlab -nosplash -nodesktop -nojvm");
	mxArray* mxName=mxCreateStringFromNChars_730(name.c_str(), name.length());
	mxArray* mxOutputFileName=mxCreateStringFromNChars_730(outputfilename.c_str(), outputfilename.length());
	engPutVariable(workengine, "name", mxName);
	engPutVariable(workengine, "outputfilename", mxOutputFileName);
	engPutVariable(workengine, "uselist", mxUseList);
	mxArray* mxSize=mxCreateDoubleMatrix(1,3,mxREAL);
	double* prSize=mxGetPr(mxSize);
	prSize[0]=height;
	prSize[1]=width;
	prSize[2]=3;
	engPutVariable(workengine, "imsize", mxSize);
	engEvalString(workengine, "uselist=find(uselist>.5)-1; save ~/CMU/Research/xcode/ObjectManipulation1/matlab/matfiles/imsave; write_output2(name,outputfilename,uselist,imsize,[],1/2);");
	engClose(workengine);
	*/
	
    delete eyedentity;
    delete firstAccel;
    
	//delete[] C_ugnv;
	//delete[] C_v;
	//delete[] P_uv;
    delete[] I;
	//delete[] Itex;
	delete[] Itexartist;
    delete[] Itexartistambient;
    if (hasSpecular) delete[] Itexartistspecular;
    delete[] Itexdiff;
	delete[] Igroundtex;
	delete[] Igroundmask;
	delete[] Ishadowmask;
	
    for (i=0; i<nshines; i++) {
        delete[] diffuseLight[i];
        if (specularLight && specularLight[i]) delete[] specularLight[i];
    }
    
    delete[] diffuseLight;
    delete[] specularLight;
    
    delete[] Idifference;
    delete[] Idiffuse;
    delete[] Iambient;
    if (hasSpecular) delete[] Ispecular;
    
    delete[] Vertices;
    delete[] VerticesDef;
    delete[] transforms;
    delete[] bdefvals;
    delete[] VerticesOriginal;
    delete[] TextureVertices;
    delete[] TextureVertices_D;
    delete[] Faces;
    delete[] FacesFirst;
    delete[] VerticesFirst;
    delete[] TextureFaces;
    delete[] TextureFaces_D;
    delete[] vertexNormals;
    delete[] vertexNormalsTransformed;
    delete[] VerticesFirstTransformed;
    
    delete[] mapwidths;
    delete[] mapheights;
    delete[] mapids;
    delete[] mapnameholder;
    
    
    
	/*
	 pbrtInit(options);
	 // Process scene description
	 PBRT_STARTED_PARSING();
	 if (filenames.size() == 0) {
	 // Parse scene from standard input
	 ParseFile("-");
	 } else {
	 // Parse scene from input files
	 for (u_int i = 0; i < filenames.size(); i++)
	 if (!ParseFile(filenames[i]))
	 Error("Couldn't open scene file \"%s\"", filenames[i].c_str());
	 }
	 pbrtCleanup();
	 */
	if (adjacencies) delete[] adjacencies;
    if (fadjacencies) delete[] fadjacencies;
	delete[] si;
    
    delete[] PixelSpaceAppearance;
    cvReleaseImage(&binaryImage);
    cvReleaseImage(&middilatedImage);
    cvReleaseImage(&erodedImage);
    delete[] diffuseLightOutput[0];
    delete[] groundNoObjLightOutput[0];
	delete[] diffuseLightOutput;
	delete[] groundNoObjLightOutput;

	
	return 0;
}



