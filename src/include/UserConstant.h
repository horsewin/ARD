#ifndef USER_CONSTANT_H_
#define USER_CONSTANT_H_

//***************** Macro for preprocessor of if-def ********************/
#define SIM_MICROMACHINE 1
#define USE_ARMM_VRPN 1
//#define USE_ARMM_SERVER_VIEW 1
//#define USE_ARMM_VRPN_RECEIVER 1
#define USE_SKIN_SEGMENTATION 1
//#define USE_OPTICAL_FLOW 1 
//#define USE_PARTICLES 1
#define USE_OSGMENU 1
#define UPDATE_TRIMESH 1
#define SIM_FREQUENCY 10
#define SHOWSEGMENTATION 1
#define SHOWKINECTIMG 0
//***********************************************************************/

//***************** Abbreviation of for statement  ********************/
#define REP(i,n) for(int i=0;i<(int)n;++i)
//***********************************************************************/


//***************** Define of types  ********************/
#ifndef VRPN_ARCH
	typedef  char            vrpn_int8;
	typedef  unsigned char   vrpn_uint8;
	typedef  short           vrpn_int16;
	typedef  unsigned short  vrpn_uint16;
	typedef  int             vrpn_int32;
	typedef  unsigned int	 vrpn_uint32;
	typedef  float           vrpn_float32;
	typedef  double          vrpn_float64;
#endif

enum panelinput{NOTHING, ADDARMODEL, SPHERE, CAR1, CAR2, REGENERATE};
//***********************************************************************/

//***************** Global Macro constants********************/
//for image processing
#define CAPTURE_SIZE cvSize(640,480)
#define REGISTRATION_SIZE cvSize(640,480) //cvSize(320,240)->cvSize(640,480)
#define MESH_SIZE cvSize(160,120)
#define CV_RED cvScalar(255,0,0)
#define CV_GREEN cvScalar(0,255,0)
#define CV_BLUE cvScalar(0,0,255)
#define CV_WHITE cvScalar(255,255,255)

//for hand segmentation and representation
enum{MAX_NUM_HANDS	= 1};
enum{MIN_HAND_PIX	= 21};
enum{HAND_GRID_SIZE = MIN_HAND_PIX*MIN_HAND_PIX};
enum{HAND_SIZE		= HAND_GRID_SIZE};

//for file loading
#define MARKER_FILENAME				"Data/Celica.bmp"
#define CAMERA_PARAMS_FILENAME		"Data/Cameras/camera.yml"
#define KINECT_PARAMS_FILENAME		"Data/Cameras/kinect.yml"
#define KINECT_TRANSFORM_FILENAME	"Data/Cameras/KinectTransform.yml"
#define KINECT_CONFIG_FILENAME		"Data/Cameras/KinectConfig.xml"
#define KINECT_CLIENT_TRANSFORM_FILENAME "C:/Users/Frontier2/Dropbox/Lab/M2/prog/Cameras/KinectTransform.yml"

//for VRPN connection
enum{UDP_LIMITATION = MIN_HAND_PIX*MIN_HAND_PIX};

//for soft texture
enum{resX=12, resY=9};
enum interaction{INIT, STROKE1, PINCH, TEXTUREGET, KEEP, STROKE2, PASTE};

//Ground vertex size
enum{NUM_VERTS_X = 160, NUM_VERTS_Y = 120};

//for shadow mask
#define invisibleMask  0x0
#define rcvShadowMask  0x1
#define castShadowMask 0x2

//for physics simulation
#define FRICTION 1.0f
#define RESTITUTION 0.0

#endif