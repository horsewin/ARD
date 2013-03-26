//TODO オブジェクトロスト後の処理がおかしいので修正の必要あり

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "main.h"
#include "UserConstant.h"
#include "constant.h"
#include <windows.h>
#pragma comment(lib, "winmm.lib")
//OpenNI
#include <XnOS.h>
#include <XnCppWrapper.h> 
//OpenCV
#include "opencv\cv.h"
#include "opencv\highgui.h"
//OPIRA
#include "CaptureLibrary.h"
#include "OPIRALibrary.h"
#include "OPIRALibraryMT.h"
#include "RegistrationAlgorithms/OCVSurf.h"
//Bullet
#include "Physics/bt_ARMM_world.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
 //Graphics calls
#include "Rendering\osg_Root.h"
#include "Rendering\osg_Object.h"
#include "ARMM\Rendering\osg_Menu.h"
#include "Rendering\osg_geom_data.h"
//Transforms
#include "leastsquaresquat.h"
#include "Transforms.h"
//Controller Input
#include "Controls/KeyboardControls.h"
//#include "Controls/XBoxControls.h"
//Network using VRPN
#ifdef USE_ARMM_VRPN
#include "Network\Communicator.h"
#endif
//hand segmentation
#ifdef USE_SKIN_SEGMENTATION
#include "Skin_Col_Segment/HandRegion.h"
#include "Skin_Col_Segment/FlowCapture.h"
#endif
//STL
#include <deque>
#include <assert.h>

//---------------------------------------------------------------------------
// Constant/Define
//---------------------------------------------------------------------------
#define SHOW_FPS_ON_WINDOW 1
using namespace std; 
using namespace xn;

const int MAX_FRAMES_SIZE = 100;

//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
int	gOsgArInputButton;
int gOsgArAddModelIndex;
vector<int> objVectorDeletable;

extern vector<int> fingerIndex;

extern int collide[2];
extern btVector3 pCollision;
extern int collisionInd;
extern interaction interact_state;

CvPoint2D32f center_trimesh_osg;

bt_ARMM_world *m_world;
boost::shared_ptr<osg_Root> pOsgRoot;

KeyboardController *kc;
//XboxController *xc;

// OpenNI Global
Context niContext;
DepthMetaData niDepthMD;
ImageMetaData niImageMD;

bool running = true;

Capture *capture;
CvMat *RegistrationParams;

//fo updating terrain
int counter = 5;
float *ground_grid, *voxel_grid;
float MaxHeight, MinHeight;

//image processing
IplImage *colourIm, *depthIm, *prev_gray, *curr_gray ,*transDepth160, *transDepth320, *transColor320;
IplImage *depthmask_for_mesh;

int input_key;
bool panelCollisionLock;	//for mutual exclusion of panel input
panelinput panelInput;
bool bTextureTransfer;

#ifdef USE_ARMM_VRPN
vrpn_Connection_IP* m_Connection;
ARMM_Communicator* ARMM_server;
vrpn_Imager_Server* ARMM_img_server;
vrpn_float32 ARMM_img_buffer[19200];
int	channel_id;

#ifdef USE_ARMM_VRPN_RECEIVER
	//Receiver part added by Atsushi 
	char *ARMM_CLIENT_IP = "ARMM_Client_Server@192.168.102.128";
	vrpn_Tracker_Remote	*ARMM_sever_receiver;
	int pass_key;
#endif

#endif

#ifdef USE_SKIN_SEGMENTATION
HandRegion gHandRegion;
CvPoint curr_hands_corners[MAX_NUM_HANDS], prev_hands_corners[MAX_NUM_HANDS];
float hand_depth_grids[MAX_NUM_HANDS][HAND_GRID_SIZE];
float curr_hands_ratio[MAX_NUM_HANDS];
float ratio;
#endif

/////////////////////
// To calculate FPS//
/////////////////////
double previous_time = 0.0;
int    count_frame   = 0;
deque<double> fps;
double time_spent    = 0.0;
double showFPS = 0.0;
/////////////////////

//---------------------------------------------------------------------------
//Prototype
//---------------------------------------------------------------------------
//for Kinect
bool loadKinectParams(char *filename, CvMat **params, CvMat **distortion);
void loadKinectTransform(char *filename);
void GenerateTrimeshGroundFromDepth(IplImage* depthIm, float markerDepth);
void inpaintDepth(DepthMetaData *niDepthMD, bool halfSize);
void TransformImage(IplImage* depthIm, IplImage* ResDepth, float markerDepth, CvSize img_size, bool isDepth);

//reflesh all scene
void RenderScene(IplImage *arImage, Capture *capture);

namespace{
	double cal_mean() {
		double sum = 0;
		for(unsigned int i = 0; i < fps.size(); i++){
			sum += fps[i];
		}
		sum /= fps.size();
		return sum;
	}

	double cal_std(double mean) {
		double sum = 0;
		for(unsigned int i = 0; i < fps.size(); i++){
			sum += (fps[i] - mean) * (fps[i] - mean);
		}
		sum /= fps.size();
		return ( sqrt(sum) );
	}

	void TickCountAverageBegin()
	{
		previous_time = static_cast<double>(cv::getTickCount());
	}

	bool TickCountAverageEnd()
	{
		count_frame++;
		double current_time = static_cast<double>(cv::getTickCount());
		time_spent += ( current_time - previous_time ) / cv::getTickFrequency();		
		if( count_frame == 30){ // you can change the frame count if you want
			if( fps.size() < MAX_FRAMES_SIZE)
			{
				//fps.push_back(count_frame/time_spent);
				fps.push_back(1000*time_spent/count_frame);
			}
			else
			{
				fps.pop_front();
				fps.push_back(1000*time_spent/count_frame);
			}
			count_frame = 0;	
			time_spent = 0.0;		
			double mean = cal_mean();
			showFPS = 1000/mean;
			//cout << "MEAN = " << mean << "ms  " << "SIGMA = " << cal_std(mean) << endl;
			previous_time = current_time;
			return true;
		}
		return false;
	}
}

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

inline CvMat* scaleParams(CvMat *cParams, double scaleFactor) 
{
	CvMat *sParams = cvCloneMat(cParams);
	sParams->data.db[0]*= scaleFactor;	sParams->data.db[4]*= scaleFactor;
	sParams->data.db[2]*= scaleFactor;	sParams->data.db[5]*= scaleFactor;
	return sParams;
}

#ifdef USE_ARMM_VRPN_RECEIVER
void VRPN_CALLBACK handle_object (void * userData, const vrpn_TRACKERCB t)
{
	switch(t.sensor){
		case 78:
			pass_key = 78; //N
			break;

		case 2:
			break;

		default:
			pass_key = 0;
			break;
	}
}
#endif

//////////////////// Entry point //////////////////// 
int main(int argc, char* argv[]) 
{
	depthmask_for_mesh = cvCreateImage(MESH_SIZE, IPL_DEPTH_8U, 1);
	markerSize.width = -1;  
	markerSize.height = -1;

	//init OpenNI
	EnumerationErrors errors;
	switch (XnStatus rc = niContext.InitFromXmlFile(KINECT_CONFIG_FILENAME, &errors)) {
		case XN_STATUS_OK:
			break;
		case XN_STATUS_NO_NODE_PRESENT:
			XnChar strError[1024];	errors.ToString(strError, 1024);
			printf("%s\n", strError);
			return rc; break;
		default:
			printf("Open failed(xn): %s\n", xnGetStatusString(rc));
			return rc;
	}

	//set camera parameter
	capture = new Camera(0, CAPTURE_SIZE, CAMERA_PARAMS_FILENAME);
	RegistrationParams = scaleParams(capture->getParameters(), double(REGISTRATION_SIZE.width)/double(CAPTURE_SIZE.width));

	//creating rendering object
	pOsgRoot = boost::shared_ptr<osg_Root>(new osg_Root());

	//init parameter for rendering
	pOsgRoot->osg_init(calcProjection(RegistrationParams, capture->getDistortion(), REGISTRATION_SIZE));

	//for Kinect view
	loadKinectParams(KINECT_PARAMS_FILENAME, &kinectParams, &kinectDistort);
	kinectDistort =0;
	//kinectParams->data.db[2]=320.0; 
	//kinectParams->data.db[5]=240.0;
	kinectParams->data.db[2]=640.0; 
	kinectParams->data.db[5]=480.0;

	//setting kinect context
	niContext.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
	niContext.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
	g_depth.GetMirrorCap().SetMirror(false);
	g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);

	//registration
	kinectReg = new RegistrationOPIRA(new OCVSurf());
	kinectReg->addResizedMarker(MARKER_FILENAME, 400);

	//physics
	m_world = new bt_ARMM_world();
	ground_grid = new float[ARMM::ConstParams::GRID_SIZE];
	for (int i =0;i < ARMM::ConstParams::GRID_SIZE; i++) 
	{
		ground_grid[i] = 0; 
	}

	//controls
	kc = new KeyboardController();

	loadKinectTransform(KINECT_TRANSFORM_FILENAME);

#ifdef USE_ARMM_VRPN
	//----->Server part
	m_Connection = new vrpn_Connection_IP();
	ARMM_server = new ARMM_Communicator(m_Connection);

	//Open the imager server and set up channel zero to send our data.
	//if ( (ARMM_img_server = new vrpn_Imager_Server("ARMM_Image", m_Connection, MESH_SIZE.width, MESH_SIZE.height)) == NULL) {
	//	fprintf(stderr, "Could not open imager server\n");
	//	return -1;
	//}
	//if ( (channel_id = ARMM_img_server->add_channel("Grid")) == -1) {
	//	fprintf(stderr, "Could not add channel\n");
	//	return -1;
	//}
	ARMM_server->SetObjectsData(&(m_world->m_objectsBody));
	ARMM_server->SetHandsData(&(m_world->HandObjectsArray));

  cout << "Created VRPN server." << endl;
	//<-----
#ifdef USE_ARMM_VRPN_RECEIVER 	//----->Receiver part
	ARMM_sever_receiver = new vrpn_Tracker_Remote (ARMM_CLIENT_IP);
	ARMM_sever_receiver->register_change_handler(NULL, handle_object);
#endif 	//<----- 

#endif

#ifdef USE_SKIN_SEGMENTATION	//Skin color look up
	gHandRegion.LoadSkinColorProbTable();
#endif

#if USE_OSGMENU == 1
	AssignPhysics2Osgmenu();
	pOsgRoot->ResetAddModelMode();
	pOsgRoot->ResetPanelCond();
	ResetTextureTransferMode();
#endif

/////////////////////////////////////////////Main Loop////////////////////////////////////////////////
	while (running) 
	{
    //start kinect
#if SHOW_FPS_ON_WINDOW == 1
		TickCountAverageBegin();
#endif
		if (XnStatus rc = niContext.WaitAnyUpdateAll() != XN_STATUS_OK) 
		{
			printf("Read failed: %s\n", xnGetStatusString(rc));
			return rc;
		}

		//get image and depth data from Kinect
		g_depth.GetMetaData(niDepthMD);
		g_image.GetMetaData(niImageMD);

		colourIm = cvCreateImage(cvSize(niImageMD.XRes(), niImageMD.YRes()), IPL_DEPTH_8U, 3);
		memcpy(colourIm->imageData, niImageMD.Data(), colourIm->imageSize); cvCvtColor(colourIm, colourIm, CV_RGB2BGR);
		cvFlip(colourIm, colourIm, 1);

		depthIm = cvCreateImage(cvSize(niDepthMD.XRes(), niDepthMD.YRes()), IPL_DEPTH_16U, 1);
		transDepth160 = cvCreateImage(cvSize(MESH_SIZE.width, MESH_SIZE.height), IPL_DEPTH_32F, 1);
		transDepth320 = cvCreateImage(cvSize(ARMM::ConstParams::SKIN_X, ARMM::ConstParams::SKIN_Y), IPL_DEPTH_32F, 1);
		transColor320 = cvCreateImage(cvSize(ARMM::ConstParams::SKIN_X, ARMM::ConstParams::SKIN_Y), IPL_DEPTH_8U, 3);
		memcpy(depthIm->imageData, niDepthMD.Data(), depthIm->imageSize);
		//cvCircle(colourIm, cvPoint(marker_origin.x,marker_origin.y), 5, CV_BLUE, 3); //for debug to show marker origin
#if SHOWKINECTIMG == 1
		cvShowImage("Kinect View", colourIm);
#endif
		IplImage *arImage = capture->getFrame();
		cvWaitKey(1); 

		//check input device
		input_key = kc->check_input(pOsgRoot, m_world);
		ExecuteAction(input_key);

		//xc->check_input();
#ifdef USE_ARMM_VRPN_RECEIVER
		if( pass_key != 0){
			kc->check_input(pass_key);
			pass_key = 0;
		}
#endif

		// kinect transform as cvmat* for use
		if(kinectTransform) 
		{ 
			if( counter >= SIM_FREQUENCY) 
			{
#ifdef UPDATE_TRIMESH
				//TickCountAverageBegin();

				inpaintDepth(&niDepthMD, true); 
				memcpy(depthIm->imageData, niDepthMD.Data(), depthIm->imageSize);				
				TransformImage(depthIm, transDepth160, MARKER_DEPTH, MESH_SIZE, true);
				GenerateTrimeshGroundFromDepth(transDepth160, MARKER_DEPTH); /*Trimesh generation*/
				m_world->updateTrimeshRefitTree(ground_grid);//opencl?
				pOsgRoot->osg_UpdateHeightfieldTrimesh(ground_grid);//opencl?

				//TickCountAverageEnd();
#endif

				counter = 0;
			} 
			else 
			{
#ifdef USE_SKIN_SEGMENTATION /*Skin color segmentation*/ // may be reduce resolution first as well as cut off depth make processing faster
				// (2)Sphere representation
				//----->Transform both depth and color
				//TickCountAverageBegin();
				TransformImage(depthIm, transDepth320,	MARKER_DEPTH, cvSize(ARMM::ConstParams::SKIN_X, ARMM::ConstParams::SKIN_Y), true);
				TransformImage(colourIm, transColor320, MARKER_DEPTH, cvSize(ARMM::ConstParams::SKIN_X, ARMM::ConstParams::SKIN_Y), false);
				int num_hand_in_scene = gHandRegion.FindHands(depthIm, colourIm, transDepth320, transColor320);
				REP(i,num_hand_in_scene)
				{
					if(m_world->getTotalNumberHand() < num_hand_in_scene) 
					{
						CreateHand(curr_hands_corners[i].x, curr_hands_corners[i].y);
					}
				}
				//TickCountAverageEnd();

				//TickCountAverageBegin();
				UpdateAllHands();
				//TickCountAverageEnd();
#endif
				counter++;
			}
#if USE_OSGMENU == 1
			//(3) Checker for AR button input
			CheckerArInput();
			input_key = CheckerArModelButtonType(input_key);
#endif

			//do hand pose recognition
			//TickCountAverageBegin();
			m_world->Update();
			//TickCountAverageEnd();

			//(B)normal client only rendering
			RenderScene(arImage, capture);
		} /* if(kinectTransform) */
//		TickCountAverageEnd();

#ifdef USE_ARMM_VRPN
	//Send Car position+orientation			
	//TickCountAverageBegin();
	ARMM_server->mainloop();
	//TickCountAverageEnd();
#ifdef USE_ARMM_VRPN_RECEIVER
	ARMM_sever_receiver->mainloop();
#endif
	////Copy depth info
	//for (int i = 0; i < ARMM::ConstParams::GRID_SIZE;i++) {
	//	ARMM_img_buffer[i] = ground_grid[i];
	//}

	//Send depth grid info	
	//ARMM_img_server->send_begin_frame(0, MESH_SIZE.width-1, 0, MESH_SIZE.height-1);
 //   ARMM_img_server->mainloop();
 //   int nRowsPerRegion= ((int) vrpn_IMAGER_MAX_REGIONf32)/ MESH_SIZE.width;
 //   for(int y=0; y<MESH_SIZE.height; y+=nRowsPerRegion) {
 //     ARMM_img_server->send_region_using_base_pointer(channel_id,0,MESH_SIZE.width-1,y,min(MESH_SIZE.width,y+nRowsPerRegion)-1, ARMM_img_buffer, 1, MESH_SIZE.width, MESH_SIZE.height);
 //     ARMM_img_server->mainloop();
 //   }
 //   ARMM_img_server->send_end_frame(0, MESH_SIZE.width-1, 0, MESH_SIZE.height-1);
 //   ARMM_img_server->mainloop();


	//<--Exec data transmission
	//TickCountAverageBegin();
	m_Connection->mainloop();

#if SHOW_FPS_ON_WINDOW == 1
	TickCountAverageEnd();
#endif

#endif

		cvReleaseImage(&arImage);
		cvReleaseImage(&depthIm); 
		cvReleaseImage(&colourIm);
		cvReleaseImage(&transDepth160);
#ifdef USE_SKIN_SEGMENTATION
		cvReleaseImage(&transDepth320);
		cvReleaseImage(&transColor320);
#endif
	}

	//memory release
	delete m_world;
	delete kinectReg;
	cvReleaseMat(&RegistrationParams);
	delete kc;
	//delete xc;

	return 0;
}

void RenderScene(IplImage *arImage, Capture *capture) 
{
	float scale = 10;
	osg::Vec3d worldVec;

	std::vector <osg::Quat> quat_obj_array;
	std::vector <osg::Vec3d> vect_obj_array;

	//set the number of objects in the AR environment
	const int num_of_objects = pOsgRoot->mOsgObject->getVirtualObjectsCount();

	if(num_of_objects > 0) 
	{
		if(!objVectorDeletable.empty())
		{
			for(int i= objVectorDeletable.size()-1; i>=0; i--)
			{	
				DeleteVirtualObject(objVectorDeletable[i]);
			}
			objVectorDeletable.clear();
		}

		if( interact_state == TEXTUREGET)
		{
			interact_state = KEEP;
			cout << "Interaction state changed TEXTUREGET to KEEP" << endl;

			//TODO 別の場所で生成するように書き換え
			//add soft texture object into the environment
			pOsgRoot->osgAddObjectNode(m_world->CreateSoftTexture("Data/tex.bmp"));
		}

		for(int i = 0; i < num_of_objects; i++) 
		{
			btTransform trans2 = m_world->get_Object_Transform(i);
			btQuaternion quat2 = trans2.getRotation();
			quat_obj_array.push_back(osg::Quat(quat2.getX(), quat2.getY(), quat2.getZ(), quat2.getW())); 
			vect_obj_array.push_back(osg::Vec3d(trans2.getOrigin().getX()*scale, trans2.getOrigin().getY()*scale,trans2.getOrigin().getZ()*scale));		

			////for debug for collision
			//if(collisionInd >= 444){
			//	osg::Node * node = obj_node_array.at(0);		
			//	osg::NodePathList paths = node->getParentalNodePaths();
			//	osg::Matrix world2local = osg::computeWorldToLocal(paths.at(paths.size()-1));
			//	osg::Vec3d localVec = worldVec * world2local;
			//	//printf("(%d)%f,%f,%f -- %f,%f,%f\n",obj_node_array.size(),localVec.x(), localVec.y(), localVec.z(), vect_obj_array[0].x(), vect_obj_array[0].y(), vect_obj_array[0].z());
			//}
		}
	} 
	else // Virtual_Objects_Count <= 0
	{}

	pOsgRoot->osg_update(quat_obj_array, vect_obj_array);
	
	pOsgRoot->osg_render(arImage, RegistrationParams, capture->getDistortion());
}

bool loadKinectParams(char *filename, CvMat **params, CvMat **distortion) 
{
	CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );
	if (fs==0) return false; 

	CvFileNode* fileparams;
	//Read the Camera Parameters
	fileparams = cvGetFileNodeByName( fs, NULL, "camera_matrix" );
	*params = (CvMat*)cvRead( fs, fileparams );

	//Read the Camera Distortion 
	fileparams = cvGetFileNodeByName( fs, NULL, "distortion_coefficients" );
	*distortion = (CvMat*)cvRead( fs, fileparams );
	cvReleaseFileStorage( &fs );

	return true;
}

void loadKinectTransform(char *filename) 
{
	CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );

	if (fs !=0 ) 
	{
		CvSeq *s = cvGetFileNodeByName(fs, 0, "MarkerSize")->data.seq;
		markerSize.width = cvReadInt((CvFileNode*)cvGetSeqElem(s, 0));
		markerSize.height = cvReadInt((CvFileNode*)cvGetSeqElem(s, 1));
		printf("Marker Size = %d x %d\n", markerSize.width, markerSize.height);
		s = cvGetFileNodeByName(fs, 0, "MarkerOrigin")->data.seq;
		marker_origin.x = cvReadInt((CvFileNode*)cvGetSeqElem(s, 0));
		marker_origin.y = cvReadInt((CvFileNode*)cvGetSeqElem(s, 1));
		setWorldOrigin();
		WORLD_SCALE = cvReadRealByName(fs, 0, "WorldScale", 1);
		WORLD_ANGLE = cvReadRealByName(fs, 0, "WorldAngle", 0);
		MARKER_DEPTH = cvReadRealByName(fs, 0, "MARKER_DEPTH", 0);
		printf("WORLD_SCALE= %.4f \n", WORLD_SCALE);

		CvFileNode* fileparams = cvGetFileNodeByName( fs, NULL, "KinectTransform" );
		kinectTransform = (CvMat*)cvRead( fs, fileparams );
		cvReleaseFileStorage( &fs );

		if (niContext.WaitAnyUpdateAll() == XN_STATUS_OK) 
		{
			//Load in the marker for registration
			pOsgRoot->osg_inittracker(MARKER_FILENAME, 400, markerSize.width);

			m_world->setWorldDepth(MARKER_DEPTH);
			m_world->setWorldScale(WORLD_SCALE);
			pOsgRoot->setOSGTrimeshScale(WORLD_SCALE);

			g_depth.GetMetaData(niDepthMD);
			inpaintDepth(&niDepthMD, true);
			depthIm = cvCreateImage(cvSize(niDepthMD.XRes(), niDepthMD.YRes()), IPL_DEPTH_16U, 1);
			transDepth160 = cvCreateImage(cvSize(MESH_SIZE.width, MESH_SIZE.height), IPL_DEPTH_32F, 1);
			memcpy(depthIm->imageData, niDepthMD.Data(), depthIm->imageSize);	

			TransformImage(depthIm, transDepth160, MARKER_DEPTH, MESH_SIZE, true);
			GenerateTrimeshGroundFromDepth(transDepth160, MARKER_DEPTH);
			m_world->updateTrimesh(ground_grid);
			m_world->setMinHeight(MinHeight);
			m_world->setMaxHeight(MaxHeight);
			m_world->initPhysics();
		}
	}
}

void TransformImage(IplImage* src_img, IplImage* dst_img, float markerDepth, CvSize img_size, bool isDepth) {

	float ground_depth = (float)markerDepth/10;
	int Depth, Channels;
	if(isDepth) {
		Depth = IPL_DEPTH_32F;
		Channels = 1;
	} else {
		Depth = IPL_DEPTH_8U;
		Channels = 3;
	}
	IplImage *tmp_img = cvCreateImage(cvGetSize(src_img), Depth, Channels);//problem using cvthres with 64F
	IplImage *lowerResImg1 = cvCreateImage(cvSize(img_size.width,img_size.height), Depth, Channels);
	IplImage *lowerResImg2 = cvCreateImage(cvSize(img_size.width,img_size.height), Depth, Channels);
	if(isDepth) 
		cvConvertScale(src_img, tmp_img, .1);//convert img from 16U to 32F and mm to cm
	else
		cvCopyImage(src_img, tmp_img);
	cvResize(tmp_img, lowerResImg1, CV_INTER_NN);//use nearest neighbor interpolation
	if(isDepth) {
		cvSubRS(lowerResImg1, cvScalar(ground_depth), lowerResImg1);
		for (int i = 0; i < img_size.height; i++) {
			for (int j = 0; j < img_size.width; j++) {
				if (i < 5 || j < 3)
					CV_IMAGE_ELEM(lowerResImg1, float, i, j) = 0;
			}
		}
	}
	int scale =  (int) img_size.width/MESH_SIZE.width;
	cvSetZero(lowerResImg2);
	CvMat* rot_mat = cvCreateMat(2,3,CV_32FC1);
	cv2DRotationMatrix( cvPoint2D32f(WORLD_ORIGIN_X*scale,WORLD_ORIGIN_Y*scale), WORLD_ANGLE, 1, rot_mat );//correct back
	cvWarpAffine( lowerResImg1, lowerResImg2, rot_mat );
	//if(isDepth && img_size.width == MESH_SIZE.width) cvShowImage("Depth Transformed", lowerResImg2); else cvShowImage("Color Transformed", lowerResImg2);
	cvCopyImage(lowerResImg2, dst_img);
	cvReleaseImage(&tmp_img);
	cvReleaseImage(&lowerResImg1);
	cvReleaseImage(&lowerResImg2);		

}

void GenerateTrimeshGroundFromDepth(IplImage* depthIm, float markerDepth) 
{
	float ground_depth = (float)markerDepth/10;
	IplImage * depthtmp = cvCreateImage(MESH_SIZE, IPL_DEPTH_8U, 1);
	MaxHeight = ground_depth-40;
	MinHeight = -MaxHeight;

	//reverse depth mask
	cvThreshold(depthmask_for_mesh, depthtmp, 0, 255, CV_THRESH_BINARY_INV);
	cvDilate(depthtmp,depthtmp, NULL, 5);
	
	//set cvScalar(0) in hand region in depth image
	cvSet(depthIm, cvScalar(0), depthtmp);

	float val = 0;
	for (int i = 0; i < MESH_SIZE.width; i++) {
		for (int j = 0; j < MESH_SIZE.height; j++) {
			int index = j*MESH_SIZE.width+i;
			//val = CV_IMAGE_ELEM(lowerResDepth2, float, 119-j, i);
			val = CV_IMAGE_ELEM(depthIm, float, (MESH_SIZE.height-1)-j, i);
			if(val > MaxHeight){
				val = MaxHeight;
			}
			else if(val < 0.001 && -0.001 < val){
				val = ground_grid[index];
			}
			else if(val < MinHeight){
				val = MinHeight;
			}
			ground_grid[index] =  val;
		}
	}
	//cvShowImage("depth",depthtmp);
	//cvShowImage("depth2",depthIm);
	cvReleaseImage(&depthtmp);
}

void inpaintDepth(DepthMetaData *niDepthMD, bool halfSize)
{
	IplImage *depthIm, *depthImFull;

	const float div = 4.0;

	if (halfSize) {
		depthImFull = cvCreateImage(cvSize(niDepthMD->XRes(), niDepthMD->YRes()), IPL_DEPTH_16U, 1);
		depthImFull->imageData = (char*)niDepthMD->WritableData();
		depthIm = cvCreateImage(cvSize(depthImFull->width/div, depthImFull->height/div), IPL_DEPTH_16U, 1);
		cvResize(depthImFull, depthIm, 0);
	} else {
		depthIm = cvCreateImage(cvSize(niDepthMD->XRes(), niDepthMD->YRes()), IPL_DEPTH_16U, 1);
		depthIm->imageData = (char*)niDepthMD->WritableData();
	}
	
	IplImage *depthImMask = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
	for (int y=0; y<depthIm->height; y++) {
		for (int x=0; x<depthIm->width; x++) {
			CV_IMAGE_ELEM(depthImMask, char, y, x)=CV_IMAGE_ELEM(depthIm, unsigned short,y,x)==0?255:0;
		}
	}

	IplImage *depthImMaskInv = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
	cvNot(depthImMask, depthImMaskInv);

	double min, max; cvMinMaxLoc(depthIm, &min, &max, 0, 0, depthImMaskInv);
	
	IplImage *depthIm8 = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
	float scale = 255.0/(max-min);
	cvConvertScale(depthIm, depthIm8, scale, -(min*scale));

	IplImage *depthPaint = cvCreateImage(cvGetSize(depthIm8), IPL_DEPTH_8U, 1);
	cvInpaint(depthIm8, depthImMask, depthPaint, 3, CV_INPAINT_NS);
	
	IplImage *depthIm16 = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_16U, 1);
	cvConvertScale(depthPaint, depthIm16, 1/scale, min);

	if (halfSize) 
	{
		IplImage *depthPaintedFull = cvCreateImage(cvGetSize(depthImFull), IPL_DEPTH_16U, 1);
		cvResize(depthIm16, depthPaintedFull,0);
		IplImage *depthImMaskFull = cvCreateImage(cvGetSize(depthImFull), IPL_DEPTH_8U, 1);
		for (int y=0; y<depthImFull->height; y++) for (int x=0; x<depthImFull->width; x++)
			CV_IMAGE_ELEM(depthImMaskFull, char, y, x)=CV_IMAGE_ELEM(depthImFull, unsigned short,y,x)==0?255:0;
		cvCopy(depthPaintedFull, depthImFull, depthImMaskFull);
		cvReleaseImage(&depthPaintedFull); cvReleaseImage(&depthImMaskFull);
		cvReleaseImage(&depthImFull);
	} 
	else 
	{
		cvCopy(depthIm16, depthIm, depthImMask);
	}

	cvReleaseImage(&depthIm8); cvReleaseImage(&depthIm16);
	cvReleaseImage(&depthPaint);
	cvReleaseImage(&depthImMask); cvReleaseImage(&depthImMaskInv);
	cvReleaseImage(&depthIm);
}

void setWorldOrigin() 
{
	WORLD_ORIGIN_X = marker_origin.x / ARMM::ConstParams::WORLD_DIV; 
	WORLD_ORIGIN_Y = marker_origin.y / ARMM::ConstParams::WORLD_DIV; 
	center_trimesh_osg = cvPoint2D32f(WORLD_ORIGIN_X, WORLD_ORIGIN_Y);
	m_world->set_center_trimesh(WORLD_ORIGIN_X,WORLD_ORIGIN_Y);
}

void RegisterMarker() 
{
	if (calcKinectOpenGLTransform(colourIm, depthIm, &kinectTransform)) 
	{
		//Load in the marker for registration
		//original marker width = 40cm
		// A unit of markerSize.width is [pixel]
		pOsgRoot->osg_inittracker(MARKER_FILENAME, 400, markerSize.width);	
		printf("Reloaded Marker Size = %dx%d\n", markerSize.width, markerSize.height);
		//Set OSG Menu

		//In original ARMM, this code is uncommented out---> 
		//However, some segmentation faults cannot be removed, so i commented it out at the moment
		////Recreat world and controls
		//delete kc;

		////delete xc;
		//delete m_world;

		//m_world = new bt_ARMM_world();
		//kc = new KeyboardController();
		//xc = new XboxController(m_world);
		//<-----

		m_world->setWorldDepth(MARKER_DEPTH);
		m_world->setWorldScale(WORLD_SCALE);
		pOsgRoot->setOSGTrimeshScale(WORLD_SCALE);
		setWorldOrigin();
		transDepth160 = cvCreateImage(cvSize(MESH_SIZE.width, MESH_SIZE.height), IPL_DEPTH_32F, 1);
		TransformImage(depthIm, transDepth160, MARKER_DEPTH, MESH_SIZE, true);
		GenerateTrimeshGroundFromDepth(transDepth160, MARKER_DEPTH);

		m_world->updateTrimesh(ground_grid);
		m_world->setMinHeight(MinHeight);
		m_world->setMaxHeight(MaxHeight);

		//----->
		//m_world->initPhysics();
		//<-----

	}
	else
	{
		printf("Couldn't find marker, please try again!\n");
	}
}

#ifdef USE_SKIN_SEGMENTATION
int CreateHand(int lower_left_corn_X, int lower_left_corn_Y) 
{
	int index = m_world->getTotalNumberHand();

	ratio = curr_hands_ratio[0];
	m_world->createHand( lower_left_corn_X, lower_left_corn_Y, MIN_HAND_PIX, ratio);	
	pOsgRoot->osg_createHand(index, lower_left_corn_X, lower_left_corn_Y, WORLD_SCALE, ratio);
	return index; //return the index of current hand
}

void UpdateAllHands() 
{
	for(int i = 0; i < m_world->getTotalNumberHand();i++) 
	{
		//In this case, spheres are displayed on the bottom-left corner of the marker
		m_world->updateHandDepth(i, curr_hands_corners[i].x, curr_hands_corners[i].y, ratio, hand_depth_grids[i]);
		pOsgRoot->osg_UpdateHand(i, m_world->debugHandX(i), m_world->debugHandY(i), m_world->debugHandZ(i));
	}

}
#endif

//TODO osgクラスのメソッドに変更
void DeleteVirtualObject(const int & index)
{
	//vector<osg::ref_ptr<osg::PositionAttitudeTransform>>::iterator it = obj_transform_array.begin() + index;
	//vector<osg::ref_ptr<osg::Node>>::iterator it2 = obj_node_array.begin() + index;

	//if( shadowedScene->getNumChildren() <= index ||
	//	shadowedScene->getNumChildren() == 0)
	//{
	//	cerr << "Error: Out of range in ShadowedScene children arrays(DeleteVirtualObject)" << endl;
	//	return;
	//}
	//if(obj_transform_array.empty()) 
	//{
	//	cerr << "No object " << endl;
	//	return;
	//}
	//cout << shadowedScene->getNumChildren() << " " << index << endl;
	//shadowedScene->removeChild(obj_transform_array.at(index));
	//obj_transform_array.erase(it);
	//obj_node_array.erase(it2);
	//Virtual_Objects_Count--;
	//cout << index << "'s virtual objects LOST : Remain " << Virtual_Objects_Count << endl;
}

#if USE_OSGMENU == 1
void AssignPhysics2Osgmenu()
{
	m_world->CreateMenu(pOsgRoot->mOsgMenu);
	m_world->CreateModelButton(pOsgRoot->mOsgMenu);
}

void ResetTextureTransferMode()
{
	bTextureTransfer = false;
}

void CheckerArInput()
{
	if(gOsgArInputButton > 0)
	{
		string buttonStr = pOsgRoot->mOsgMenu->getObjMenuNodeArray().at(gOsgArInputButton)->getName(); 

		//set transmitted key input to client nodes
		//TODO : implementation network part
		input_key = kc->TransmitInput(gOsgArInputButton);

		//change panelInput state
		if(buttonStr.find("model.3ds") != string::npos)	//MODE: Add model
		{
			panelInput = ADDARMODEL;
			pOsgRoot->ModelButtonInput();
		}
		else if(buttonStr.find("reset.3ds") != string::npos)	//MODE: Reset virtual models
		{
			pOsgRoot->mOsgObject->osg_resetNodes();
			pOsgRoot->ResetPanelCond();
		}
		else if(buttonStr.find("Transfer") != string::npos)	//MODE: Texture Transfer
		{
			if(m_world->m_objectsBody.size() < 2)
			{
				cerr << "No enough model is found : need two at least" << endl;
				pOsgRoot->ResetPanelCond();

				//play some effect
				PlaySound(_T("jump02.wav"), NULL, SND_ASYNC);	

			}
			//having two models at least in AR env
			else
			{
				bTextureTransfer = true;
				pOsgRoot->ToggleMenuVisibility();	//menu should be disappeared
			}
		}

		cout << buttonStr.c_str() << endl;

		//reset
		gOsgArInputButton = -1;		
	}
}

int CheckerArModelButtonType(const int & v)
{
	if(gOsgArAddModelIndex > 0)
	{
		string touchStr = pOsgRoot->mOsgMenu->getMenuModelObjectArray().at(gOsgArAddModelIndex)->getName();
		cout << touchStr.c_str() << endl;

		int val = 0;
		//pushing cancel button
		if( touchStr.find("cancel") != string::npos )
		{
			MessageBeep(MB_OK);
			val = 301;
		}
		else
		{
			//add physics model
			int index = m_world->create_3dsmodel(touchStr.c_str());
			
			//add visual model
			osg::Vec3 pos = pOsgRoot->GetARMenuPos();
			val = pOsgRoot->AddNewObjFromARButton(index);

			//move this model to some pos
			m_world->ChangeAttribute(pos.x()/10, pos.y()/10, pos.z()/10, index);

			//play some effect
			PlaySound(_T("pickup04.wav"), NULL, SND_ASYNC);	
		}

		//reset
		pOsgRoot->ResetArModelButtonType();
		gOsgArInputButton = -1;		
		m_world->ResetARButtonInput();

		return val;
	}

	return v;
}

void ExecuteAction(const int & val)
{
	if(val == VK_SPACE)
	{
		RegisterMarker();
	}
}

#endif