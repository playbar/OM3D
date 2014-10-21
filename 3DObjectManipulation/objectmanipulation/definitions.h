/*
 OM3D allows users to manipulate objects in photographs in 3D
 by using stock 3D models.
s Copyright (C) 2014 Natasha Kholgade and Tomas Simon
 
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
