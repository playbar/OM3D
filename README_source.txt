Welcome to version 1.0.1 of OM3D, the software to perform 3D object manipulation in photographs using stock 3D models. We provide the source code for this project here. All code has been written and tested in Xcode 5.1 on OS X 10.9. 

We provide this software under the GNU General Public License v2.0. Details about the license in the LICENSE.txt file.

We provide three Xcode projects in individual directories as follows:

1) 3DObjectManipulation: This allows users to load an example from the `examples/‘ directory, and perform manipulation of the objects in the photograph. 

2) Alignment: This allows users to align a 3D model in an example directory to an image in the same directory. 

The projects 3DObjectManipulation.xcodeproj and Alignment.xcodeproj (in directories 3DObjectManipulation and Alignment) have been packaged with static libraries (with the exception of libz). These should compile correctly on Mavericks and Mountain Lion. 

The Alignment application uses lsqr to perform least squares solve, however, this is currently very slow. We have provided an accelerated version that uses the MATLAB engine to perform the solve. The following steps may be used to recompile the Application.xcodeproj project so that MATLAB is used to perform the solve. This requires MATLAB to be installed on the system.

- In the Symbol Navigator of the Xcode project, replace ‘libmx.dylib’ and ‘libeng.dylib’ with the versions on the system if they do not match (i.e., if they show up in red). This can be done by deleting the links, right clicking on the project name, and selecting ‘Add Files to “Alignment”’. Navigate to /Applications/<matlabversionname>/bin/maci64/ and select the ‘libmx.dylib’ library (replace <matlabversionname> with the name of the MATLAB version, e.g., MATLAB_R2013a.app).  Repeat for ‘libeng.dylib’. 

- In the Build Settings for the *target* Alignment (different from those for the *project* Alignment), create the preprocessor macro USEMATLAB.

- Also, in the Build Settings for target Alignment, replace the user-defined variable MATLAB_NAME with <matlabversionname> (e.g., MATLAB_R2013a.app). This allows the compiler to find the correct header path.

- Build the project by going to Product -> Build. The built executable should show up under /path/to/OM3D_1.0.1_source/Release/Alignment. In case you obtained an executable separately from http://www.cs.cmu.edu/~om3d/agreement.html, you may replace that with the version compiled through these instructions.

- Before running the executable, set the following environment variables:

export PATH=/Applications/<matlabversionname>/bin:/Applications/<matlabversionname>/bin/maci64:/Library/Frameworks/EPD64.framework/Versions/Current/bin:$PATH
export DYLD_LIBRARY_PATH=/Applications/<matlabversionname>/runtime/maci64:/Applications/<matlabversionname>/bin/maci64:/Applications/<matlabversionname>/sys/os/maci64:/System/Library/Frameworks/JavaVM.framework/JavaVM:/System/Library/Frameworks/JavaVM.framework/Libraries:$DYLD_LIBRARY_PATH
export XAPPLRESDIR=/Applications/<matlabversionname>/X11/app-defaults

- Run the executable according to the README_executable.txt file (obtained separately during download of executables).


3) illumapp: This application automatically estimates illumination, reflectance, and appearance difference to provide plausible object manipulation. The application uses the MOSEK optimization software to estimate illumination, which may be obtained from:

http://mosek.com/resources/download/

A 90-day trial license is available for students from MOSEK. The following environment variables need to be set:

DYLD_LIBRARY_PATH: append path to MOSEK dynamic library.
MOSEKLM_LICENSE_FILE: provide file together with the absolute path

The project illumapp.xcodeproj (in the IlluminationAndAppearanceEstimation directory) requires a link to the MOSEK library. Please add the MOSEK library using the `Add Files’ option in Xcode, and set the user-defined setting MOSEK_DIR to the directory containing the `include/‘ folder for MOSEK files before compiling.

