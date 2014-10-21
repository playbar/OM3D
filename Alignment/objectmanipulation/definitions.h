//
//  definitions.h
//  pbrt
//
//  Created by Natasha Kholgade on 11/1/12.
//
//

#ifndef pbrt_definitions_h
#define pbrt_definitions_h


// Task-level pre-processor definitions
//#define NO_LIGHTING
//#define NO_TEXTURE
#define MIN_LIGHT_INTENSITY 2.6e-6
#define MIN_LIGHT_DIRECTION_ALPHA 1e-2

// Object-level pre-processor definitions
//#define DISTANT
//#define NO_DIFFERENCE_GROWTH
//#define NO_AT_GROWTH

// Old pre-processor definitions
#define GAUSS_FILTER 1
#define BOX_FILTER 2
#define NNBMAX 20
#define SHINE 8.0

enum TextureTypes {
	TYPE_LINEAR,
	TYPE_LINEAR_NORMAL,
	TYPE_LINEAR_NB,
	TYPE_LINEAR_NORMAL_NB,
};

enum LightTypes {
	TYPE_COLOR,
	TYPE_GRAYSCALE,
};

enum AlbedoTypes {
	TYPE_MEDIAN,
	TYPE_TEXTURE,
};

struct param {
    float lambda1;
    float lambda2;
	float shadowweight;
    float numIterations;
    bool groups_instead_of_color;
    float lambda_rho;
    float lambda_rho2;;
    int nstepappearance;
    int nstepillumination;
    int nstepgroundillumination;
    int nransac;
    int nrounds;
    float mrfgamma;
    float mrfbeta;
    int texturewinsize;
    float ransacthresholdratio;
    float ransacappearancethreshold;
    float ransacnormalthreshold;
    int symstopnumber;
    int symstopunseennumber;
    bool estimateAmbient;
    int ndivs;
};

#endif
