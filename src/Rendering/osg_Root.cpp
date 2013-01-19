/*
 * osg_Root.cpp
 *
 *  Created on: 2012/12/20
 *      Author: umakatsu
 */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "Rendering\osg_Root.h"	//itself

#include "Rendering\osg_Object.h"
#include "Rendering\osg_Init.h"
#include "Rendering\osg_geom_data.h"
#include "Rendering\osg_Update.h"
#include "constant.h"

#include <osgShadow/ShadowMap>

//---------------------------------------------------------------------------
// Constant/Define
//---------------------------------------------------------------------------
const osg::Quat DEFAULTATTIDUTE = 
	osg::Quat(
		osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
		osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0),
		osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 0.0, 1.0)
	);

//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
vector<int> fingersIdx;
vector<int> fingerIndex;

extern bool panelCollisionLock;
extern panelinput panelInput;
extern int gOsgArInputButton;

namespace
{
	template< class T>
	bool VectorBoundChecker(std::vector<T> v, int idx)
	{
		return( v.size() > idx && idx >= 0);
	}
}
//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
osg_Root::osg_Root()
{
	mAddModelAnimation	= 0.0;
	mOsgArAddModelIndex	= -1;

	//Create Height field
	CreateGround();
}

osg_Root::~osg_Root()
{
	osg_uninit();
}

void osg_Root::osg_init(double *projMatrix) 
{
	//child objects of this root1
	mOsgInit	= boost::shared_ptr<osg_Init>( new osg_Init());
	mOsgGeom	= boost::shared_ptr<osg_geom>( new osg_geom());
	mOsgUpdater = boost::shared_ptr<osg_Update>( new osg_Update());

	//
	mVideoImage = new osg::Image();
	mGLImage = cvCreateImage(cvSize(512,512), IPL_DEPTH_8U, 3);

	mViewer.addEventHandler(new osgViewer::WindowSizeHandler());
	mViewer.setUpViewInWindow(100, 100, ARMM::ConstParams::WINDOW_WIDTH, ARMM::ConstParams::WINDOW_HEIGHT);
	
	mViewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
	mViewer.setKeyEventSetsDone(0);

	osg::ref_ptr<osg::Group> root = new osg::Group();
	mViewer.setSceneData(root);

	// ----------------------------------------------------------------
	// Video background
	// ----------------------------------------------------------------
	osg::ref_ptr<osg::Camera> bgCamera = new osg::Camera();
	bgCamera->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
	bgCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	bgCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	bgCamera->getOrCreateStateSet()->setMode(GL_LIGHTING, GL_TRUE);
	bgCamera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, GL_FALSE);
	bgCamera->setProjectionMatrixAsOrtho2D(0, ARMM::ConstParams::WINDOW_WIDTH, 0, ARMM::ConstParams::WINDOW_HEIGHT);
	
	osg::ref_ptr<osg::Geometry> geom = osg::createTexturedQuadGeometry(
		osg::Vec3(0, 0, 0), 
		osg::X_AXIS * ARMM::ConstParams::WINDOW_WIDTH, 
		osg::Y_AXIS * ARMM::ConstParams::WINDOW_HEIGHT, 
		0, 
		1, 
		1, 
		0);
	geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, new osg::Texture2D(mVideoImage));
	
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->addDrawable(geom.get());
	bgCamera->addChild(geode.get());
	root->addChild(bgCamera.get());
	
	// ----------------------------------------------------------------
	// Foreground 3D content
	// ----------------------------------------------------------------
	osg::ref_ptr<osg::Camera> fgCamera = new osg::Camera();
	fgCamera->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
	fgCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
	fgCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	fgCamera->setProjectionMatrix(osg::Matrix(projMatrix));
	root->addChild(fgCamera.get());

	arTrackedNode = new ARTrackedNode();
	fgCamera->addChild(arTrackedNode);

	//for input
	gOsgArInputButton	= -1;

};

void osg_Root::osg_uninit() 
{
	fingerIndex.clear();
	arTrackedNode->stop();
	cvReleaseImage(&mGLImage);
}

void osg_Root::osg_inittracker(string markerName, int maxLengthSize, int maxLengthScale) 
{
	static bool hasInit = false;

	if (hasInit) {
		arTrackedNode->removeChildren(0,arTrackedNode->getNumChildren());
		celicaIndex = arTrackedNode->addMarkerContent(markerName, maxLengthSize, maxLengthScale, mShadowedScene);
		arTrackedNode->setVisible(celicaIndex, true);
		return;
	}

	arTrackedNode->removeChildren(0,arTrackedNode->getNumChildren());

	//Create rendering world and set parameters of this world----->
	// Set shadow node
	//osg::ref_ptr<osgShadow::ShadowTexture> sm = new osgShadow::ShadowTexture; //V
	//osg::ref_ptr<osgShadow::MyShadowMap> sm = new osgShadow::MyShadowMap; //Adrian
	osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap; //Atsushi
	//sm->setLight(arTrackedNode->getLight()); //Atsushi
	sm->setTextureSize( osg::Vec2s(1024, 1024) ); //Adrian
	sm->setTextureUnit( 1 );

	mShadowedScene = new osgShadow::ShadowedScene;
	mShadowedScene->setShadowTechnique( sm.get() );
	mShadowedScene->setReceivesShadowTraversalMask( rcvShadowMask );
	mShadowedScene->setCastsShadowTraversalMask( castShadowMask );
	mShadowedScene->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON); //Adrian  

	//set param of lighting
	osg::ref_ptr<osg::LightSource> source = new osg::LightSource; //V
	source->getLight()->setPosition( osg::Vec4(4.0, 8.0, 10.0, 0.0) ); //V
	source->getLight()->setAmbient( osg::Vec4(0.1, 0.1, 0.1, 1.0) ); //V
	source->getLight()->setDiffuse( osg::Vec4(0.9, 0.9, 0.9, 1.0) ); //V
	mShadowedScene->addChild(source);

	//<-----

#if CAR_SIMULATION == 1
	mOsgInit->CreateCarUnit(mOsgObject);

	//Add these cars to rendering scene
	mShadowedScene->addChild( mOsgObject->car_transform.at(0) );
	mShadowedScene->addChild( mOsgObject->car_transform.at(1) );

	for(int i = 0 ; i < 2; i++)  { 
		for(int j = 0 ; j < 4; j++)  { 
			mShadowedScene->addChild(mOsgObject->wheel_transform[i].at(j));
		}
	} 
#endif /* CAR_SIMULATION == 1 */

	celicaIndex = arTrackedNode->addMarkerContent(markerName, maxLengthSize, maxLengthScale, mShadowedScene);
	arTrackedNode->setVisible(celicaIndex, true);

/*Adrian */
	{
		// Create the Heightfield Geometry
		mOsgInit->SetHeightfield(mOsgObject);

		// Create the containing geode
		osg::ref_ptr< osg::Geode > geode = new osg::Geode(); 
		geode->addDrawable(mOsgObject->mHeightFieldGeometry_quad);
		geode->addDrawable(mOsgObject->mHeightFieldGeometry_line);

		//Create the containing transform
		float scale = 10; float x = 0; float y = 0; float z = 0;
		osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
		mt->setScale(osg::Vec3d(scale,scale,scale));
		mt->setAttitude(osg::Quat(0,0,0,1));       
		mt->setPosition(osg::Vec3d(x, y, z)); 
		mt->addChild( geode.get() );

		//Set up the depth testing for the landscale
		osg::Depth * depth = new osg::Depth();
		depth->setWriteMask(true); 
		depth->setFunction(osg::Depth::Function::LEQUAL);
		mt->getOrCreateStateSet()->setAttributeAndModes(depth, osg::StateAttribute::ON);

		//Set up the shadow masks
		mt->setNodeMask( rcvShadowMask );
		mt->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
		mShadowedScene->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");

		//At the heightmap twice, once for shadowing and once for occlusion
		arTrackedNode->addModel(mt);
		mShadowedScene->addChild(mt);

#if CAR_SIMULATION == 1
		mOsgInit->SetCarRenderBin(mOsgObject);
#endif

	}/*Adrian*/

	hasInit = true;

	/* DEBUG code */
	//for confirmation about the direction of axis
	//osg::Node * axis = createMilestone();
	//axis->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");
	//mShadowedScene->addChild(axis);

	//Create OSG Menu
#if USE_OSGMENU==1
	mOsgMenu = new ARMM::osg_Menu();
	OsgInitMenu();
	OsgInitModelMenu();
#endif
}

void osg_Root::osg_render(IplImage *newFrame, osg::Quat *q,osg::Vec3d  *v, osg::Quat wq[][4], osg::Vec3d wv[][4], CvMat *cParams, CvMat *cDistort, std::vector <osg::Quat> q_array, std::vector<osg::Vec3d>  v_array) 
{
	if(mAddModelAnimation >0.001 || mAddModelAnimation < -0.001) //means "!= 0.0"
	{
		ModelButtonAnimation();
	}

	cvResize(newFrame, mGLImage);
	cvCvtColor(mGLImage, mGLImage, CV_BGR2RGB);
	mVideoImage->setImage(mGLImage->width, mGLImage->height, 0, 3, GL_RGB, GL_UNSIGNED_BYTE, (unsigned char*)mGLImage->imageData, osg::Image::NO_DELETE);

#ifdef USE_ARMM_SERVER_VIEW
#if CAR_SIMULATION == 1
	if(car_transform.at(0) && car_transform.at(1)) {
		for(int i = 0; i < 2; i++) {
			car_transform.at(i)->setAttitude(q[i]);
			car_transform.at(i)->setPosition(v[i]);
			for(int j = 0; j < 4; j++) {
				wheel_transform[i].at(j)->setAttitude(wq[i][j]);
				wheel_transform[i].at(j)->setPosition(wv[i][j]);
			}
		}
	}
#endif /* CAR_SIMULATION == 1 */

	//update the position of each virtual object
	for(unsigned int i = 0; i < q_array.size(); i++) 
	{
		obj_transform_array.at(i)->setAttitude(q_array.at(i));
		obj_transform_array.at(i)->setPosition(v_array.at(i));
		osg::Vec3 pos = obj_transform_array.at(i)->getPosition();
		//printf("%d,%s:%.2f  %.2f  %.2f \n", i, obj_node_array.at(i)->getName().c_str(), v_array.at(i).x(), v_array.at(i).y(), v_array.at(i).z());
		//printf("POS=(%.2f, %.2f, %.2f)\n",pos.x(), pos.y(), pos.z());
	}

	if (!mViewer.done()) 
	{
		if (CAPTURE_SIZE.width != REGISTRATION_SIZE.width || CAPTURE_SIZE.height != REGISTRATION_SIZE.height) 
		{
			double scale = double(REGISTRATION_SIZE.width)/double(CAPTURE_SIZE.width);
			IplImage *scaledImage = cvCreateImage(cvSize(newFrame->width*scale, newFrame->height*scale), newFrame->depth, newFrame->nChannels); cvResize(newFrame, scaledImage);
			arTrackedNode->processFrame(scaledImage, cParams, cDistort);
			cvReleaseImage(&scaledImage);
		} 
		else 
		{
			arTrackedNode->processFrame(newFrame, cParams, cDistort);
		}
		mViewer.frame();
	}
#endif

	//change the condition of regular state if lists of models is shown in AR space into invisible condition
	if( !mAddArModel && IsModelButtonVisibiilty())
	{
		ToggleModelButtonVisibility();
	}
}

void osg_Root::setOSGTrimeshScale(float scale)
{
	TrimeshScale = scale;
}

void osg_Root::ShowGroundGeometry()
{
	mOsgObject->mHeightFieldGeometry_line->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF);	
	mOsgObject->mGroundQuadColor->pop_back();
	mOsgObject->mGroundQuadColor->push_back(osg::Vec4(1,1,1,0.2));
	mOsgObject->mGroundLineColor->pop_back();
	mOsgObject->mGroundLineColor->push_back(osg::Vec4(1,0.2,0,1.0));
}

void osg_Root::HideGroundGeometry()
{
	mOsgObject->mHeightFieldGeometry_line->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);	
	mOsgObject->mGroundQuadColor->pop_back();
	mOsgObject->mGroundQuadColor->push_back(osg::Vec4(1,1,1,0.0));
	mOsgObject->mGroundLineColor->pop_back();
	mOsgObject->mGroundLineColor->push_back(osg::Vec4(1,1,1,0.0));
}

void osg_Root::osgAddObjectNode(osg::Node* node)
{
	mOsgGeom->osgAddObjectNode(mShadowedScene, mOsgObject, node);
}

//for hand object
void osg_Root::osg_createHand(int index, float x, float y, float world_scale, float ratio)
{
	mOsgGeom->CreateVirtualHand(mOsgObject, index, x, y, world_scale, ratio);

	//set the created hand as a child to osg scene
	mShadowedScene->addChild( mOsgObject->getHandObjectGlobalArray().at(index) );
}

void osg_Root::osg_UpdateHeightfieldTrimesh(float *ground_grid)
{
	int index =0;
	for(int i = 0; i < NUM_VERTS_X-1; i++) {
		for(int j = 0; j < NUM_VERTS_Y-1; j++) {
			float x = (float)(i- center_trimesh.x)*TrimeshScale; 
			float y = (float)(j- (120-center_trimesh.y))*TrimeshScale;
			mOsgObject->mHeightFieldPoints->at(index++) = osg::Vec3(x, y, ground_grid[j*NUM_VERTS_X+i]); 
			mOsgObject->mHeightFieldPoints->at(index++) = osg::Vec3(x+TrimeshScale, y, ground_grid[j*NUM_VERTS_X+i+1]);
			mOsgObject->mHeightFieldPoints->at(index++) = osg::Vec3(x+TrimeshScale, y+TrimeshScale, ground_grid[(j+1)*NUM_VERTS_X+i+1]); 
			mOsgObject->mHeightFieldPoints->at(index++) = osg::Vec3(x, y+TrimeshScale, ground_grid[(j+1)*NUM_VERTS_X+i]);
		}
	}
	mOsgObject->mHeightFieldGeometry_quad->dirtyDisplayList();
	mOsgObject->mHeightFieldGeometry_line->dirtyDisplayList();
}

void osg_Root::osg_UpdateHand(int index, float* x, float* y, float* grid) 
{
	mOsgUpdater->UpdateHand(mOsgObject, index, x, y, grid);
}

void osg_Root::OsgInitMenu()
{
	mOsgMenu->CreateMenuPane();

	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > pTransArray = 
		mOsgMenu->getObjMenuTransformArray();

	//add menu object into the AR scene
	osg::ref_ptr<osg::Group> menu = new osg::Group;
	REP(idx, pTransArray.size())
	{
		pTransArray.at(idx)->setNodeMask(castShadowMask);
		menu->addChild(pTransArray.at(idx));
	}

	osg::ref_ptr<osg::PositionAttitudeTransform> menuTrans = new osg::PositionAttitudeTransform;
	const osg::Vec3d BASEPOSITION(0,0,0);
	const osg::Quat BASEATTITUDE = osg::Quat(
			osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 0.0, 1.0)
	);

	menuTrans->setAttitude(BASEATTITUDE);
	menuTrans->setPosition(BASEPOSITION);
	menuTrans->addChild(menu.get());

	mShadowedScene->addChild(menuTrans.get());

	mOsgMenu->setObjMenuTransformArray(pTransArray);
}

void osg_Root::OsgInitModelMenu()
{
	mOsgMenu->CreateModelButtonCloud();

	std::vector< osg::ref_ptr<osg::PositionAttitudeTransform> > pTransArray = mOsgMenu->getMenuModelTransArray();
	//add menu object into the AR scene
	osg::ref_ptr<osg::Group> menu = new osg::Group;
	REP(idx, pTransArray.size())
	{
		pTransArray.at(idx)->setNodeMask(castShadowMask);
		menu->addChild(pTransArray.at(idx));
	}

	osg::ref_ptr<osg::PositionAttitudeTransform> menuTrans = new osg::PositionAttitudeTransform;
	const osg::Vec3d BASEPOSITION(0,0,0);
	const osg::Quat BASEATTITUDE = osg::Quat(
			osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 0.0, 1.0)
	);
	menuTrans->setAttitude(BASEATTITUDE);
	menuTrans->setPosition(BASEPOSITION);
	menuTrans->addChild(menu.get());

	mShadowedScene->addChild(menuTrans.get());

	mOsgMenu->setMenuModelTransArray(pTransArray);
}

void osg_Root::ModelButtonInput()
{
	//rendering list of models
	//rendering new button for cancal action
	ToggleModelButtonVisibility();
	mAddArModel = true;

	//disappearing all buttons and virtual objects temporary 
	ToggleMenuVisibility();
	ToggleVirtualObjVisibility();

	//play some effect
	PlaySound(_T("machine_call.wav"), NULL, SND_ASYNC);	
}

void osg_Root::ToggleMenuVisibility()
{
	std::vector< osg::ref_ptr<osg::PositionAttitudeTransform> > pMenuTransArray  = mOsgMenu->getObjMenuTransformArray();

	double shiftVal = 4;
	REP(idx, pMenuTransArray.size())
	{
		if(pMenuTransArray.at(idx)->getNodeMask() == castShadowMask)
		{
			pMenuTransArray.at(idx)->setNodeMask(invisibleMask);

			//appear ar model buttons
			mAddModelAnimation = shiftVal;
		}
		else
		{
			pMenuTransArray.at(idx)->setNodeMask(castShadowMask);

			//disappear ar model buttons
			//mAddModelAnimation = -1*shiftVal;
		}
	}

	mOsgMenu->setObjMenuTransformArray(pMenuTransArray);
	mOsgMenu->ToggleMenuButtonState();
}

void osg_Root::ToggleModelButtonVisibility()
{
	std::vector< osg::ref_ptr<osg::PositionAttitudeTransform> > pModelTransArray = mOsgMenu->getMenuModelTransArray();

	if(pModelTransArray.empty()) return;

	unsigned int nodeMask = pModelTransArray.at(0)->getNodeMask() == castShadowMask?
		invisibleMask : castShadowMask;

	REP(idx, pModelTransArray.size())
	{
		pModelTransArray.at(idx)->setNodeMask(nodeMask);
	}

	mOsgMenu->setMenuModelTransArray(pModelTransArray);
	mOsgMenu->ToggleModelButtonState();
}

void osg_Root::ToggleVirtualObjVisibility()
{
	//create a temporary object from osgObject
	std::vector< osg::ref_ptr<osg::PositionAttitudeTransform> > pTransArray = mOsgObject->getObjTransformArray();

	REP(i,pTransArray.size())
	{
		unsigned int mask = pTransArray.at(i)->getNodeMask() ^ (castShadowMask);
		pTransArray.at(i)->setNodeMask(mask);
	}

	//write back to the original object
	mOsgObject->setObjTransformArray(pTransArray);
}

bool osg_Root::IsMenuVisibiilty()
{
	return mOsgMenu->isMenuButtonState();
}

bool osg_Root::IsModelButtonVisibiilty()
{
	return mOsgMenu->isModelButtonState();
}

void osg_Root::ModelButtonAnimation()
{
	std::vector< osg::ref_ptr<osg::PositionAttitudeTransform> > pModelTransArray = mOsgMenu->getMenuModelTransArray();

	//check if valid models are found?
	if(pModelTransArray.empty())
	{
		cerr << "No model button is found in osg.h" << endl;
		return;
	}

	//set the action in current frame
	double posZ = pModelTransArray.at(0)->getPosition().z();
	const double zThreshold = 6.0;
	if(posZ > zThreshold)
	{
		mAddModelAnimation = 0;
		return;
	}

	//set the pos of each model in current frame
	REP(idx, pModelTransArray.size())
	{
		osg::Vec3 newPos = pModelTransArray.at(idx)->getPosition();
		newPos.set(newPos.x(), newPos.y(), newPos.z() + mAddModelAnimation);
		pModelTransArray.at(idx)->setPosition(newPos);		
	}

}

void osg_Root::ResetModelButtonPos()
{
	std::vector< osg::ref_ptr<osg::PositionAttitudeTransform> > pModelTransArray = mOsgMenu->getMenuModelTransArray();

	if(pModelTransArray.empty())
	{
		cerr << "No model button is found in osg.h" << endl;
		return;
	}

	REP(idx, pModelTransArray.size())
	{
		osg::Vec3 newPos = pModelTransArray.at(idx)->getPosition();
		newPos.set(newPos.x(), newPos.y(), mOsgMenu->GetInitPosZ());
		pModelTransArray.at(idx)->setPosition(newPos);		
	}

}

//add function for AR input
osg::Vec3 osg_Root::GetARMenuPos()
{

}

int osg_Root::AddNewObjFromARButton(const int & index)
{
	osg::Vec3 pos(mOsgMenu->getMenuModelTransArray().at(mOsgArAddModelIndex)->getPosition());

	int val = mOsgMenu->GetKeyAssignment(static_cast<unsigned int>(mOsgArAddModelIndex));

	//create model unit with osg::Node
	osg::ref_ptr<osg::Node> node = dynamic_cast<osg::Node*>
	(
		mOsgMenu->getMenuModelObjectArray().at(mOsgArAddModelIndex)->clone(osg::CopyOp::SHALLOW_COPY)
	);

	//register a new model as an osg unit to osg world
	float scale = 10;
	mOsgGeom->osgAddObjectNode(mShadowedScene, mOsgObject, node.get());
	mOsgObject->SetScale2ObjTrans(index, scale);
	mOsgObject->SetMask2ObjTrans(index, invisibleMask);
	mOsgObject->IncrementObjCount();

	return val; //set this value to "input_key"
}


void osg_Root::SetAddArModel(const bool & b){ mAddArModel = b; }
//void osg_Root::SetARAddModelButtton(const int & input){ mOsgArAddModelButton = input; }
//int  osg_Root::GetARModelButton(void) const { return mOsgArAddModelButton;}

void osg_Root::ResetArModelButtonType()
{
	ToggleMenuVisibility();
	ToggleModelButtonVisibility();
	ToggleVirtualObjVisibility();
	ResetAddModelMode();
	ResetPanelCond();
	setOsgArAddModelIndex(-1);
}

void osg_Root::ResetPanelCond()
{
	panelCollisionLock	= false;
	panelInput			= NOTHING;
}

//************************************************************************************
// Private functions
//************************************************************************************
void osg_Root::ResetAddModelMode()
{
	mAddArModel = false; //pOsgRoot->SetAddArModel(false);
	mOsgArAddModelIndex = -1;
	ResetModelButtonPos();
}


/** create quad at specified position. */
osg::Drawable* osg_Root::createSquare(const osg::Vec3& corner,const osg::Vec3& width,const osg::Vec3& height, osg::Image* image)
{
    // set up the Geometry.
    osg::Geometry* geom = new osg::Geometry;

    osg::Vec3Array* coords = new osg::Vec3Array(4);
    (*coords)[0] = corner;
    (*coords)[1] = corner+width;
    (*coords)[2] = corner+width+height;
    (*coords)[3] = corner+height;


    geom->setVertexArray(coords);

    osg::Vec3Array* norms = new osg::Vec3Array(1);
    (*norms)[0] = width^height;
    (*norms)[0].normalize();
    
    geom->setNormalArray(norms);
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    osg::Vec2Array* tcoords = new osg::Vec2Array(4);
    (*tcoords)[0].set(0.0f,0.0f);
    (*tcoords)[1].set(1.0f,0.0f);
    (*tcoords)[2].set(1.0f,1.0f);
    (*tcoords)[3].set(0.0f,1.0f);
    geom->setTexCoordArray(0,tcoords);
    
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));
    
    if (image)
    {
        osg::StateSet* stateset = new osg::StateSet;
        osg::Texture2D* texture = new osg::Texture2D;
        texture->setImage(image);
        stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
        geom->setStateSet(stateset);
    }
    
    return geom;
}

osg::Drawable* osg_Root::createAxis(const osg::Vec3& corner,const osg::Vec3& xdir,const osg::Vec3& ydir,const osg::Vec3& zdir)
{
    // set up the Geometry.
    osg::Geometry* geom = new osg::Geometry;

    osg::Vec3Array* coords = new osg::Vec3Array(6);
    (*coords)[0] = corner;
    (*coords)[1] = corner+xdir;
    (*coords)[2] = corner;
    (*coords)[3] = corner+ydir;
    (*coords)[4] = corner;
    (*coords)[5] = corner+zdir;

    geom->setVertexArray(coords);

    osg::Vec4 x_color(1.0f,.0f,.0f,1.0f);
    osg::Vec4 y_color(0.0f,1.0f,.0f,1.0f);
    osg::Vec4 z_color(.0f,0.0f,1.0f,1.0f);

    osg::Vec4Array* color = new osg::Vec4Array(6);
    (*color)[0] = x_color;
    (*color)[1] = x_color;
    (*color)[2] = y_color;
    (*color)[3] = y_color;
    (*color)[4] = z_color;
    (*color)[5] = z_color;
    
    geom->setColorArray(color);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,6));
    
    osg::StateSet* stateset = new osg::StateSet;
    osg::LineWidth* linewidth = new osg::LineWidth();
    linewidth->setWidth(14.0f);
    stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    geom->setStateSet(stateset);
    
    return geom;
}

osg::Node* osg_Root::createMilestone()
{
    // create the root node which will hold the model.
    osg::Group* milestone = new osg::Group();

    // add the drawable into a single goede to be shared...
    osg::Billboard* center = new osg::Billboard();
    center->setMode(osg::Billboard::POINT_ROT_EYE);
    center->addDrawable(
        createSquare(osg::Vec3(-0.5f,0.0f,-0.5f),osg::Vec3(1.0f,0.0f,0.0f),osg::Vec3(0.0f,0.0f,1.0f),osgDB::readImageFile("Images/reflect.rgb")),
        osg::Vec3(0.0f,0.0f,0.0f));
        
    osg::Geode* axis = new osg::Geode();
    axis->addDrawable(createAxis(osg::Vec3(0.0f,0.0f,0.0f),osg::Vec3(150.0f,0.0f,0.0f),osg::Vec3(0.0f,150.0f,0.0f),osg::Vec3(0.0f,0.0f,150.0f)));


    milestone->addChild(center);
    milestone->addChild(axis);

    return milestone;
}

void osg_Root::CreateGround()
{
	mOsgInit->CreateHeightfield(mOsgObject);
}
