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

// global definitions
#define Knn 1
#define MVx 1.0
#define MVy 1.0
#define MVz -1.0
#define Pz -1.0
#define NSPH 2
#define GAUSS_FILTER 1
#define BOX_FILTER 2

// object/image-example specific definitions
#define TEXH 512
#define TEXW 512
#define N_CONNMAX 40
#define R_SPHERE_RATIO .0125
#define RADIUS_RATIO .5
#define JOINTHRESH_RATIO .01
#define T_SCALE_RATIO 1.0
#define skip 1

#import "epnp.h"
//#import <opencv2/opencv.hpp>
#import <OpenGL/gl.h>
#import <OpenGL/glext.h>
#import <OpenGL/glu.h>
#import <Cocoa/Cocoa.h>
#import "matrixUtils.h"
//#import "Shader.h"
#import "string.h"
#import "lsqr.h"
//#import "geometry.h"
#ifdef USEMATLAB
#import "engine.h"
#endif

enum operationMode {
	MODE_ROTATE,
	MODE_TRANSLATE,
	MODE_SCALE,
	MODE_LASSO,
	MODE_BBOX,
	MODE_PICKPOINTS,
	MODE_AXISROTATE,
	MODE_REGULAR,
	MODE_COMPUTEINTRINSICS,
    MODE_LASSODEFORM,
	MODE_MARKDEFORM,
    MODE_MARKRIGID,
    MODE_MARKH,
	MODE_DEFORM,
	MODE_ZOOM,
    MODE_SELECT,
};

@interface BasicOpenGLView : NSOpenGLView
{
	/*
     // string attributes
     NSMutableDictionary * stanStringAttrib;
     
     // string textures
     GLString * helpStringTex;
     GLString * infoStringTex;
     GLString * camStringTex;
     GLString * capStringTex;
     GLString * msgStringTex;
	 */
	CFAbsoluteTime msgTime; // message posting time for expiration
	
	NSTimer* timer;
    
    time_t date1;
    time_t date2;
    double interval;
	
	BOOL isInitObjectProj;
	BOOL closeLoop;
	BOOL issetInitTransform;
	BOOL isBeingSaved;
	BOOL isgroundaxis, isgriddr1, isgriddr2;
	BOOL justMadeEdit;
	
    BOOL doDrawMesh;
    BOOL renderIsDone;
    int numrender;
    
	int nfaces;
	bool toggleState;
	int nvertices;
	int nvperface;
	int ntexturevertices;
	
	float rr;
	float fu, fv, uc, vc;
    float focallength;
    float sensorwidth;
	float groundAxis[4];
    float plane[4];
	GLfloat groundpts[8][3];
	int ngroundpts;
	
	GLfloat imheight;
	GLfloat imwidth;
    GLint reswidth;
    GLint resheight;
	GLfloat x1, y1, x2, y2; // zoom nums;
	
	// arguments, pbrt stuff
    std::string name;
	std::string direc;
    std::string outputfilename;
	int nsamp;
	int filtertype;
	float sigma;
	int filterhsize;
	int eqcountmax;
    
    BOOL drawMeshOnly;
	
	int nptsforlines;
	BOOL toggleDeformUpState;
	BOOL toggleDeformDownState;
	BOOL toggleRigidDownState;
	BOOL doZoom;
	BOOL deforming;
	BOOL manipPhase;
    BOOL alllocked;
    BOOL smoothBetweenRigidMotions;
    
    int numComponentsSelected;
    bool* isComponentSelected;
    bool* isVertexSelected;
    int nduplicated;
    
    float augweight;
    
    float xAxisComp[3];
    float yAxisComp[3];
    float zAxisComp[3];
	
	enum operationMode theOperationMode;
    
    CFAbsoluteTime mytime;
	
    //	ptsArray arrayOfPoints;
	GLfloat* lassoPolygon;
	int nlassopolygon;
	ptsArray clickedPoints;
    int* clickedPointsIndex;
    int nclicked;
	GLfloat* objectProjections;
	GLfloat* clickPointToObjectProjDistances;
	GLfloat* objectCentroidProjections;
	GLfloat* clickPointToCentroidProjDistances;
    GLfloat* duplicatedVertices;
	
	//Vector* vertexNormals;
	//PbrtPoint* Vertices;
	//PbrtPoint* TextureVertices;
	GLfloat* objectModel_vertices;
	GLfloat* verticesMemory;
	int nverticesmemory, nstores;
    int nmemmax;
	GLfloat* objectModel_texcoords;
	GLint* objectModel_faces;
	GLint* objectModel_texfaces;
	int* rayconstrained_idxs;
	//int* anchored_idxs;
    bool* isanchored;
    
    //int* lassodeformed_idxs;
    bool* islassodeformed;
    float* lassomovelist;
    int nlassomovelist;
    int nlassomovelistprevert;
    
    int* rigid_idxs;
    GLfloat* pts2Drigid;
	int* symmetric;
    int* planar;
	int nrayconstrained;
	//int nanchored;
    //int nlassodeformed;
    
    int nrigid;
	GLfloat* pts2Drayconstrained;
	GLfloat* objectModel_vertexSol;
	float anchorWeight;
	float symmetryWeight;
	float deformationWeight;
	float rayConstraintsWeight;
    float symmetryConstraintsWeight;
    float planarityWeight;
    float laplacianWeight;
	
	GLfloat** objectModel_teximages;
	GLint* mapwidths;
	GLint* mapheights;
	GLuint* texbindings;
	GLfloat* objectModel_image;
	GLfloat* objectModel_image1;
    GLfloat* objectModel_renderedImage;
	GLfloat* objectModel_diffuseColor;
	GLfloat* objectModel_specularColor;
	GLfloat* objectModel_ambientColor;
	GLfloat* objectModel_shininess;
	GLfloat* arapRotations;
	GLfloat* verticesInMotion;
	GLfloat* Reflection;
	int* neighboring_idxs;
	int* nneighbors;
	int* groups;
	int ngroups;
	bool* groupsLocked;
    int numiterations;
    
    bool mousejustdown;
    
    bool* deformgroupsLocked;
    int displayGroupNum;
    //int displayDeformGroupNum;
	
	GLfloat dlinex;
	GLfloat dliney;
	
	// camera handling
	GLfloat worldRotation[4];
	GLfloat objectRotation[16];
    GLfloat lastRotation[16];
	GLfloat trackBallRotation[16];
	GLfloat trackTranslation[16];
	GLfloat trackTranslationPrev[16];
	GLfloat trackBallRotationPrev[16];
	GLfloat initTransform[16];
	GLfloat shapeSize;
	GLfloat normalAxis[3];
	GLfloat contactPoint[3];
	GLfloat normalAxiso[3];
	
	GLfloat gridCentre[3];
	GLfloat gridDr1[3];
	GLfloat gridDr2[3];
	GLfloat gridAxis[3];
    
    GLfloat writeX[1000];
    GLfloat writeY[1000];
    GLfloat writeZ[1000];
    int nwrite;
    bool doWrite;
    bool selectionAxesArePCs;
    GLfloat switcherMatrix[16];
    GLfloat projectionFactor;
	
	GLfloat Xtransinit[3], Xscaleinit[3], Xinit[3];
	
	GLfloat contactPointo[3];
	GLfloat modelViewMatrix[16];
	GLfloat modelViewMatrixStart[16];
	GLfloat projectionMatrix[16];
	GLfloat mvpMatrix[16];
	GLfloat normalMat[9];
	GLfloat sampleColors[9];
	GLfloat pixelColorTransformer[9];
	std::vector<float> transforms;
	
	GLfloat uu[3], vv[3];
	
	GLfloat Xcentre, Ycentre, Zcentre, R;
	GLfloat Xcentreo, Ycentreo, Zcentreo;
    GLfloat Xcompo, Ycompo, Zcompo;
    GLfloat Xcomp, Ycomp, Zcomp;
	GLfloat Xc, Yc, Zc;
    
    GLfloat hXstart, hYstart, hZstart, hXend, hYend, hZend;
    GLfloat hTheta;
    int hmark;
	
	GLfloat Xmin, Ymin, Zmin;
	GLfloat Xmax, Ymax, Zmax;
    GLfloat BoxDiag;
    GLfloat r_sphere_val, rot_ax_val, trans_ax_val, join_thresh_val;
	GLfloat Mmin, Mmax;
	GLfloat panx, pany;
	GLfloat r0, r1;
	
	int numseen;
	int n_transforms;
	int nmaps;
	
	GLfloat startVec[3];
	GLfloat endVec[3];
	GLfloat source[3];
	GLfloat vport[4];
	GLfloat rvport[4];
    //	GLfloat dd[100*100*100];
	int inpainted;

    IBOutlet NSButton *btnRotate;
	IBOutlet NSButton *btnScale;
	IBOutlet NSButton *btnTranslate;
	IBOutlet NSButton *btnPickPoints;
    IBOutlet NSButton *btnSelectPart;
    IBOutlet NSButton *btnCamera;
    IBOutlet NSButton *btnPullPoints;
    IBOutlet NSButton *btnSymmetryOff;
    IBOutlet NSButton *btnGroundPlane;
    IBOutlet NSButton *btnSave;
    IBOutlet NSButton *btnUndo;
    IBOutlet NSButton *btnRedo;
    
	IBOutlet NSTextField* txtAnchorWeight;
    IBOutlet NSTextField* txtNumIterations;
    IBOutlet NSTextField* txtSymmetryWeight;
    IBOutlet NSTextField* txtRayConstraintsWeight;
    IBOutlet NSTextField* txtDeformationWeight;
    IBOutlet NSTextField* txtAugWeight;
    IBOutlet NSTextField* txtPlanarityWeight;
    
	int openglViewNumber;
	
	GLuint lightTex;
	GLuint backgroundTex;
	GLuint backgroundTexInpainted;
	GLuint xyzTex;
	GLuint inTex;
	
	GLfloat Ambient[3];
	
		
	GLfloat rmxl;
	GLfloat rmnl;
	GLfloat gmxl;
	GLfloat gmnl;
	GLfloat bmxl;
	GLfloat bmnl;
	
	GLfloat xmxl;
	GLfloat xmnl;
	GLfloat ymxl;
	GLfloat ymnl;
	GLfloat zmxl;
	GLfloat zmnl;
	
    double* work; // used for lsqr matrix
    int mA, nA; // size of array, mA x nA
    int mAlb; // number of elements that are just the laplacian (this part won't change)
    int mArc;
    int mAa;
    int mAs;
    double* b; // size of rhs vector
    int valcountlb;
    int eqcountlb;
    int valcountall;
    int eqcountall;
    
#ifdef USEMATLAB
    Engine* we;
#endif
	
}

#pragma mark ---- setup ----
- (int) readVertices;
+ (NSOpenGLPixelFormat*) basicPixelFormat;
- (void) processArguments;
- (id) initWithFrame: (NSRect) frameRect;
- (void) initObject;
- (void) prepareOpenGL;
- (void) computeMaxMinMean;
- (void) initTextures;

#pragma mark ---- OpenGL ObjectiveC drawing ----
- (void) drawGrid;
- (void) drawRect:(NSRect)rect;
- (void) drawObjectModel;
- (void) drawObjectModelMesh;
- (void) drawObjectModelTextured;
- (void) renderBackground;
- (void) renderObject;
- (void) renderLassoStrip;
- (void) pushState:(NSButton* )btn;

#pragma mark ---- mouse down, move, up ----
- (void) mouseDown:(NSEvent *)theEvent;
- (void) mouseDragged:(NSEvent *)theEvent;
- (void) mouseUp:(NSEvent *)theEvent;
- (void) keyDown:(NSEvent *)theEvent;
- (void) rightMouseDown:(NSEvent *)theEvent;
- (void) otherMouseDown:(NSEvent *)theEvent;
- (void) rightMouseUp:(NSEvent *)theEvent;
- (void) otherMouseUp:(NSEvent *)theEvent;
- (void) scrollWheel:(NSEvent *)theEvent;
- (void) rightMouseDragged:(NSEvent *)theEvent;
- (void) otherMouseDragged:(NSEvent *)theEvent;

# pragma mark ---- Rotation and Translation transfer ----
- (void) setInitRotation:(double[3][3])Rot Translation:(double[3])t;
- (void) manipulate;

#pragma mark ---- geometric computations for rotations, translations, and axis marking ----
- (void) multiplyMatrices_M1:(GLfloat*)m1 M2:(GLfloat*)m2 Mout:(GLfloat*)mout;
- (void) multiplyMatrices_M1:(GLfloat*)m1 M2:(GLfloat*)m2 M3:(GLfloat*)m3 Mout:(GLfloat*)mout;
- (void) multiplyMatrices_M1:(GLfloat*)m1 M2:(GLfloat*)m2 M3:(GLfloat*)m3 M4:(GLfloat*)m4 Mout:(GLfloat*)mout;
- (void) computeIntrinsicsAndPlaneAxis;
- (void) getProjectionOf:(GLfloat*)xpt NearAxis:(GLfloat*)ax AtCentre:(GLfloat*)centre PlacedIn:(GLfloat*)Xout MinDistance:(GLfloat*)dis;
- (void) get3DPointOnSphere:(GLfloat*)the3Dpoint from2Dpoint:(GLfloat *)the2Dpoint;
- (void) computeRotation;
- (void) project:(GLfloat*)v OntoPlanePerpToAxis:(GLfloat*)axis;
- (BOOL) testContactWithPlaneAxis:(GLfloat*)ax GridCentre:(GLfloat*)gc Vertices:(GLfloat*)verts;
- (void) reprojectVector:(GLfloat*)vin ontoPlaneAxis:(GLfloat*)ax SphereRadius:(GLfloat)rad ToGet:(GLfloat*)vout;
- (void) viewportTransformX:(GLfloat)xin Y:(GLfloat)yin toX:(GLfloat*)xout toY:(GLfloat*)yout;
- (void) zoom;
- (void) appleRecalculateGroundAxis;
- (void) partBasedRigidTransform;
- (void) hingeMotion;
- (int) computeNearest3DPointIndexUsingProjectionsTo_x:(GLfloat)vx y:(GLfloat)vy;
- (int) computeNearest3DPointIndexUsingProjectionsTo_x:(GLfloat)vx y:(GLfloat)vy distance:(GLfloat*)dist;
- (void) computeObjectProjections;
- (void) computeCentroidAndAxesForSelection;
- (void) invertRigidTransform_in:(GLfloat*)mv out:(GLfloat*)mvinv;
- (void) determineChoiceOfAxis_x:(GLfloat)vvx _y:(GLfloat)vvy _xC:(GLfloat)xc _yC:(GLfloat)yc _zC:(GLfloat)zc _xA:(GLfloat*)xa _yA:(GLfloat*)ya _zA:(GLfloat*)za;
- (void) determineContactWithGround:(GLfloat*)multmat _prevTransf:(GLfloat*)prev
                        _nextTransf:(GLfloat*)next;
- (void) determineMotionOfAxis_x:(GLfloat)vvx _y:(GLfloat)vvy _xC:(GLfloat)xc _yC:(GLfloat)yc _zC:(GLfloat)zc _xA:(GLfloat*)xa _yA:(GLfloat*)ya _zA:(GLfloat*)za _xout:(GLfloat*)Xout;
- (void) scaleIf_isX:(bool)isX _isY:(bool)isY _isZ:(bool)isZ _xA:(GLfloat*)xa _yA:(GLfloat*)ya _zA:(GLfloat*)za Ratio:(GLfloat)ratio _xC:(GLfloat)xc _yC:(GLfloat)yc _zC:(GLfloat)zc;

#pragma mark ----arap computations----
-(void) getMeshConnectivity;
-(void) setupLBOperator;
-(void) setupRayConstraints:(GLfloat*)mv;
-(void) setupAnchors:(GLfloat*)mv;
-(void) setupSymmetryConstraints:(GLfloat*)mv;
-(void) setup_b_verts:(GLfloat*)verts _mv:(GLfloat*)mv;
-(void) arapSolve;

#pragma mark --- setters and getters (and clearers) ----
- (void) getPoint:(GLfloat*)pt Type:(int)tt Index:(int)idx;
- (void) setPoint:(GLfloat*)pt Type:(int)tt Index:(int)idx;
- (void) getCameraParametersFu:(float*)ptrfu Fv:(float*)ptrfv Uc:(float*)ptruc Vc:(float*)ptrvc;
- (int)  getOpenglViewNumber;
- (void) getAClickedPointGivenPos:(int)pos X:(GLfloat*)X Y:(GLfloat*)Y Z:(GLfloat*)Z;
- (int)  getNumClickedPoints;
- (void) clearClickedPoints;
- (void) getVertices:(GLfloat*)VV;
- (void) setVertices:(GLfloat*)VV;
- (int)  getNumVertices;
- (int)  getNumIndices;
- (void) setNumIndices:(int)n;
- (void) getIndices:(int*)idxs;
- (void) setIndices:(int*)idxs;
- (void) getPtsConstrained:(GLfloat*)pts;
- (void) setPtsConstrained:(GLfloat*)pts;
- (void) setOperationMode:(operationMode)myOperationMode;
- (operationMode) getOperationMode;
- (void) setDisplayGroupNum:(int)theGroupNum;
- (int) getDisplayGroupNum;
- (int) getNumGroups;
- (BOOL) getIsSetInitTransform;
- (BOOL) isGroupsExists;

#pragma mark ---- update ----
- (void)animationTimer:(NSTimer *)timer;
- (void) update;		// moved or resized
- (BOOL) acceptsFirstResponder;
- (BOOL) becomeFirstResponder;
- (BOOL) resignFirstResponder;
- (void) awakeFromNib;
- (void) undo;
- (void) undoredostore;
- (void) redo;
- (void)enableCamera;

#pragma mark ---- actions ----
- (IBAction)redoAction:(id)sender;
- (IBAction)undoAction:(id)sender;
- (IBAction)rotateAction:(id)sender;
- (IBAction)lassoAction:(id)sender;
- (IBAction)lassodeformAction:(id)sender;
- (IBAction)bboxAction:(id)sender;
- (IBAction)pickPointsAction:(id)sender;
- (IBAction)saveAction:(id)sender;
- (IBAction)axisRotateAction:(id)sender;
- (IBAction)revertAction:(id)sender;
- (IBAction)translateAction:(id)sender;
- (IBAction)computeRigidAction:(id)sender;
- (IBAction)scaleAction:(id)sender;
- (IBAction)renderAction:(id)sender;
- (IBAction)computeIntrinsicsAction:(id)sender;
- (IBAction)deformAction:(id)sender;
- (IBAction)markRigidAction:(id)sender;
- (IBAction)selectAction:(id)sender;
- (IBAction)selectionChoiceAction:(id)sender;
- (IBAction)smoothBetweenRigid:(id)sender;

- (IBAction)markHinge:(id)sender;
- (IBAction)foldUp:(id)sender;
- (IBAction)foldDown:(id)sender;
- (IBAction) duplicateAction:(id)sender;

- (IBAction)movableAction:(id)sender;
- (IBAction)zoomAction:(id)sender;
- (IBAction)lockGroup:(id)sender;
- (IBAction)lockAction:(id)sender;
- (IBAction)clearAMAction:(id)sender;
- (IBAction)undoAction:(id)sender;
- (IBAction)startManipAction:(id)sender;
- (IBAction)sxpa:(id)sender;
- (IBAction)sxma:(id)sender;
- (IBAction)sypa:(id)sender;
- (IBAction)syma:(id)sender;
- (IBAction)szpa:(id)sender;
- (IBAction)szma:(id)sender;
- (IBAction)setAnchorWeight:(id)sender;
- (IBAction)setNumIterations:(id)sender;
- (IBAction)setSymmetryWeight:(id)sender;
- (IBAction)setRayConstraintsWeight:(id)sender;
- (IBAction)setDeformationWeight:(id)sender;
- (IBAction)setAugWeight:(id)sender;
- (IBAction)setPlanarityWeight:(id)sender;
- (IBAction)setTransfAction:(id)sender;
- (IBAction)getTransfAction:(id)sender;
- (IBAction)symmetryOnOffAction:(id)sender;

@end

// tricks to get interactibility
// large sphere ==> ||Xs-Xi|| ~ ||Xs-Xc|| ~ Radius
// assume that a fragment shares the occlusion characteristics of closest randomly selected point
//