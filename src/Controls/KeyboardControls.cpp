/*
 * KeyboardControls.cpp
 *
 *
 *  Created on: 2013/1/15
 *      Author: umakatsu
 */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "Controls\KeyboardControls.h"

#include "Physics/bt_ARMM_world.h"
#include "Rendering/osg_Root.h"
#include "Rendering\osg_Object.h"
#include "Rendering\osg_geom_data.h"
#include "constant.h"

#include <Windows.h>
#include <iostream>

//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
using namespace std;

extern int collide_counter;
extern double prev_collide_clock;
extern interaction interact_state;
extern bool running;
extern CvSize markerSize;
extern CvPoint2D32f marker_origin;
extern float WORLD_SCALE;
extern float WORLD_ANGLE;
extern float MARKER_DEPTH;
extern float WORLD_ORIGIN_X;
extern float WORLD_ORIGIN_Y;
extern CvMat *kinectTransform;

//for AR input button
extern int gOsgArInputButton;

double x = 0, y=0, z=0;
bool WIREFRAME_MODE = false;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
namespace
{
	inline bool LoadCheck(void * ptr, const char * filename)
	{
		if( ptr == NULL){
			std::cerr << "Error : No such file in the directory or incorrect path.  " << filename << std::endl;
			return false;
		}
		return true;
	}
}


KeyboardController::KeyboardController(bt_ARMM_world *m_world) 
	: Controller(m_world)
{
	mOsgGeom	= boost::shared_ptr<osg_geom>( new osg_geom());
}

KeyboardController::~KeyboardController()
{
	mKeyAssignment.clear();
}

int KeyboardController::TransmitInput(const int & input)
{
	const int transmitOffset = 200;
	return (input>0? (transmitOffset+input) : 0);
}


int KeyboardController::check_input(boost::shared_ptr<osg_Root> osgRoot)
{
	//key offset to use key assignment info
	const int offset = 79;

#if CAR_SIMULATION == 1
	//Car Number 1
	if (getKey(VK_UP)) {
		world->accelerateEngine(0);
	} else if (getKey(VK_DOWN)) {
		world->decelerateEngine(0);
	} else {
		world->resetEngineForce(0);
	}
	if (getKey(VK_LEFT)) {
		world->turnEngineLeft(0);
	} else if (getKey(VK_RIGHT)) {
		world->turnEngineRight(0);
	} else {
			world->turnReset(0);
	}
	//Car Number 2
	if (getKey(87)) {//W
		world->accelerateEngine(1);
	} else if (getKey(83)) {//S
		world->decelerateEngine(1);
	} else {
		world->resetEngineForce(1);
	}
	if (getKey(65)) {//A
		world->turnEngineLeft(1);
	} else if (getKey(68)) {//D
		world->turnEngineRight(1);
	} else {
		world->turnReset(1);
	}
#endif /* CAR_SIMULATION == 1 */

	//A 65 S 83 D 68 W 87 F 70 V 86
	if (getKey(VK_ESCAPE)) running = false;
#ifdef SIM_MICROMACHINE
	if (getKey(82)) world->resetCarScene(0); //R
	if (getKey(84)) world->resetCarScene(1); //T
#endif /*SIM_MICROMACHINE*/

	//about height field
	if (getKey(86)) { //V
		WIREFRAME_MODE = !WIREFRAME_MODE;
		if(WIREFRAME_MODE) 
		{
			osgRoot->ShowGroundGeometry();
		}
		else 
		{
			osgRoot->HideGroundGeometry();
		}
		printf("Wireframe Mode = %d \n",WIREFRAME_MODE);
		return 86;
	}

#ifdef SIM_MICROMACHINE
	if(osgRoot->mOsgObject->getVirtualObjectsCount() < ARMM::ConstParams::MAX_NUM_VIR_OBJECT) 
	{
		//for debug
		if (getKey(66)) { //B
			osgRoot->osgAddObjectNode(world->CreateSoftTexture("Data/tex.bmp"));
			return 66;
		}
		//for debug
		if (getKey(77)) { //M
			//interact_state = PINCH;
			gOsgArInputButton = 203;
			return 203;
		}

		if (getKey(79)) { //o
			//string modelname(DATABASEDIR);
			//modelname+="cube/cube.3ds";
			////string modelname = "HatuneMiku.3ds";
			//int index = world->create_3dsmodel(modelname.c_str());
			//osgAddObjectNode(osgNodeFrom3dsModel(world->GetModelName(), world->get3dsScale(), world->get_Object_Transform(index)));
			//Virtual_Objects_Count++;
			//world->ChangeAttribute(25, -5, 5, index);
			//				
			return 79;
		}

		if (getKey(80)) { //p
			
			string modelname(ARMM::ConstParams::DATABASEDIR);
			modelname+="keyboard/keyboard.3ds";
			//string modelname = "HatuneMiku.3ds";
			int index = world->create_3dsmodel(modelname.c_str());
			osgRoot->osgAddObjectNode(mOsgGeom->osgNodeFrom3dsModel(world->GetModelName(), world->get3dsScale(), world->get_Object_Transform(index)));
			osgRoot->mOsgObject->IncrementObjCount();
			world->ChangeAttribute(10, -5, 5, index);

			return 80;
		}
		if (getKey(81)) { //q
			string modelname(ARMM::ConstParams::DATABASEDIR);
			modelname+="cube/cube.3ds";
			int index = world->create_3dsmodel(modelname.c_str());
			osgRoot->osgAddObjectNode(mOsgGeom->osgNodeFrom3dsModel(world->GetModelName(), world->get3dsScale(), world->get_Object_Transform(index)));
			osgRoot->mOsgObject->IncrementObjCount();
			world->ChangeAttribute(25, -5, 5, index);
								
			return 81;
		}

		if (getKey(78) ) { //N
			int index = world->create_Sphere();
			osgRoot->osgAddObjectNode(mOsgGeom->osgNodeFromBtSphere(ARMM::ConstParams::SPHERE_SIZE, world->get_Object_Transform(index)));
			osgRoot->mOsgObject->IncrementObjCount();
			return 78;
		}
	}
#endif /*SIM_MICROMACHINE*/
	if (getKey(VK_SPACE)) 
	{
		return VK_SPACE;
	}

	if (getKey(VK_RETURN))
	{
		if (kinectTransform)
		{
			CvFileStorage *fs = cvOpenFileStorage(KINECT_TRANSFORM_FILENAME, 0, CV_STORAGE_WRITE);
			cvStartWriteStruct(fs, "MarkerSize", CV_NODE_MAP); 
				cvWriteInt(fs, "width", markerSize.width);
				cvWriteInt(fs, "height", markerSize.height);
			cvEndWriteStruct(fs);

			cvStartWriteStruct(fs, "MarkerOrigin", CV_NODE_MAP); 
				cvWriteInt(fs, "x", marker_origin.x);
				cvWriteInt(fs, "y", marker_origin.y);
			cvEndWriteStruct(fs);

			cvWriteReal(fs, "WorldScale", WORLD_SCALE);
			cvWriteReal(fs, "WorldAngle", WORLD_ANGLE);
			cvWriteReal(fs, "MARKER_DEPTH", MARKER_DEPTH);

			cvWrite(fs, "KinectTransform", kinectTransform);
			cvReleaseFileStorage( &fs );
			printf("Saved Kinect Transform\n");
		}
	}
	return 0;
}


//************************************************************************************
// Private functions
//************************************************************************************

bool KeyboardController::RegisteringObject(const char * filename, boost::shared_ptr<osg_Root> osg_Root)
{
	float scale = 10;
	ostringstream str;
	const char * format = ".3ds";

	str << ARMM::ConstParams::DATABASEDIR << filename << "/" << filename << format;
	osg::ref_ptr<osg::Node> obj = osgDB::readNodeFile(str.str().c_str());
	if(!LoadCheck(obj.get(), str.str().c_str()))
	{
		return false;
	}

	string tmpFilename(filename);
	tmpFilename += format;
	obj->setName(tmpFilename);

	osg_Root->osgAddObjectNode(obj.get());
	return true;
}

void KeyboardController::AssignmentKeyinput(const char * settingFilename)
{
	ostringstream setInput;
	setInput <<  ARMM::ConstParams::DATABASEDIR << settingFilename;

	std::ifstream input(setInput.str().c_str());

	if(!input.is_open())
	{
		cerr << "Setting file cannot be openned!!" << endl;
		cerr << "Filename is " << setInput.str().c_str() << endl;
		exit(EXIT_SUCCESS);
	}

	while(input)
	{
		char line[1024] ;
		input.getline(line, 1024) ;
		std::stringstream line_input(line) ;

		pair<unsigned int, string> tmpKeyAssignment;

		//first word means a value assignment
		unsigned int value;
		line_input >> value;
		tmpKeyAssignment.first = value;

		//second word means a name of model
		std::string keyword;
		line_input >> keyword;
		tmpKeyAssignment.second = keyword;

		mKeyAssignment.push_back(tmpKeyAssignment);
	}
}

inline bool KeyboardController::getKey(int key)
{
	return GetAsyncKeyState(key)& 0x8000; 
}
