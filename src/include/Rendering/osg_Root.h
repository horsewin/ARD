/*
 * osg.h
 *
 *  Created on: 2011/09/20
 *  Refactored on: 2012/12/20
 *      Author: umakatsu
 */

#ifndef OSG_H
#define OSG_H

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Texture2D>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/io_utils>
#include <osg/Depth>
#include <osg/Notify>
#include <osg/Billboard>
#include <osg/LineWidth>
#include <osgDB/ReadFile>
#include <osgShadow/ShadowedScene>
#include "UserConstant.h"

#include <map>
#include <sstream>
#include <iostream>

#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/Utils.h>

#include "OPIRALibraryMT.h"
#include "RegistrationAlgorithms\OCVSurf.h"

#include "opencv/highgui.h"
//#include "osg_geom_data.h"
#include "ARMM\Rendering\osg_Menu.h"

#include "UserConstant.h"

#include <boost\shared_ptr.hpp>

#include <tchar.h>

class ARTrackedNode : public osg::Group 
{

private:
	OPIRALibrary::Registration *r;
	typedef std::map<std::string, osg::ref_ptr<osg::MatrixTransform>> MarkerMap;
	MarkerMap mMarkers;
	osg::ref_ptr<osg::PositionAttitudeTransform> osgTrans;
	osg::ref_ptr<osg::Light> light;

public:
	ARTrackedNode(): osg::Group() {
		r = new RegistrationOPIRAMT(new OCVSurf());
		osg::ref_ptr<osg::Group> lightGroup = new osg::Group;  
		// Create a local light.
		light = new osg::Light();
		light->setLightNum(0);
		light->setAmbient(osg::Vec4(0.4f,0.4f,0.4f,1.0f));
		light->setDiffuse(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
		light->setSpecular(osg::Vec4(0.4f,0.4f,0.4f,1.0f));
		light->setConstantAttenuation(0.2f);
		light->setLinearAttenuation(0.05f);
		light->setQuadraticAttenuation(0.05f);
		light->setPosition(osg::Vec4(30, -5, 5, 0.0));

		osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource;	
		lightSource->setLight(light.get());
		lightSource->setLocalStateSetModes(osg::StateAttribute::ON); 
		lightSource->setStateSetModes(*this->getOrCreateStateSet(), osg::StateAttribute::ON);
		lightGroup->addChild(lightSource.get());
	
		this->addChild(lightGroup.get());
	}

	osg::Light* getLight() {
		return light.get();
	}

	void stop() 
	{
		delete r;
	}

	void setVisible(int index, bool visible)
	{
		osg::ref_ptr<osg::Switch> s = (osg::Switch*)this->getChild(index);
		if (visible) s->setAllChildrenOn(); 
		else s->setAllChildrenOff();
	}

	void processFrame(IplImage *mFrame, CvMat *cParams, CvMat *cDistort) 
	{
		for (MarkerMap::iterator iter = mMarkers.begin(); iter != mMarkers.end(); iter++) 
		{
			osg::ref_ptr<osg::Switch> s = 
				(osg::Switch*)iter->second->getChild(0); s.get()->setAllChildrenOff();
		}

		std::vector<MarkerTransform> mTransforms = r->performRegistration(mFrame, cParams, cDistort);

		for (std::vector<MarkerTransform>::iterator iter = mTransforms.begin(); iter != mTransforms.end(); iter++) {
			MarkerMap::iterator mIter = mMarkers.find(iter->marker.name);
			if (mIter != mMarkers.end()) {
				osg::ref_ptr<osg::MatrixTransform> m = mIter->second;
				m.get()->setMatrix(osg::Matrixd(iter->transMat));
				((osg::Switch*)m->getChild(0))->setAllChildrenOn();
			}
		}
		for (unsigned int i=0; i<mTransforms.size(); i++) { 
			free(mTransforms.at(i).viewPort); free(mTransforms.at(i).projMat); 	free(mTransforms.at(i).transMat);
		}
		mTransforms.clear();
	}

	int addMarkerContent(string imageFile, int maxLengthSize, int maxLengthScale, osg::Node *node) 
	{
		r->removeMarker(imageFile);
		if (r->addResizedScaledMarker(imageFile, maxLengthSize, maxLengthScale)) 
		{
			Marker m = r->getMarker(imageFile);

			osgTrans = new osg::PositionAttitudeTransform();
			osgTrans->setAttitude(osg::Quat(
						osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
						osg::DegreesToRadians(180.f), osg::Vec3d(0.0, 1.0, 0.0),
						osg::DegreesToRadians(180.f), osg::Vec3d(0.0, 0.0, 1.0)
						));
			osgTrans->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
			osgTrans->getOrCreateStateSet()->setMode(GL_NORMALIZE, GL_TRUE);
			osgTrans->addChild(node);

			osg::ref_ptr<osg::Switch> foundObject = new osg::Switch();
			foundObject->addChild(osgTrans.get());

			osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform();
			mt->addChild(foundObject.get());

			osg::ref_ptr<osg::Switch> trackObject = new osg::Switch();
			trackObject->addChild(mt);
			this->addChild(trackObject.get());

			mMarkers[imageFile] = mt;
			return(this->getChildIndex(trackObject.get()));
		} else {
			return -1;
		}
	}

	osg::Node* getOsgTrans() {
		return osgTrans.get();
	}

	void addModel(osg::Node *node) {
		osgTrans->addChild(node);
	}

	void removeModel(osg::Node *node) {
		osgTrans->removeChild(node);
	}
};

class osg_Object;
class osg_Init;
class osg_geom;
class osg_Update;
class osg_Root
{
public:
	
	ARMM::osg_Menu * mOsgMenu;
	boost::shared_ptr<osg_Object> mOsgObject;
	osg::ref_ptr<ARTrackedNode> arTrackedNode; 

	int celicaIndex;

private:
	osgViewer::Viewer mViewer;

	osg::ref_ptr<osgShadow::ShadowedScene> mShadowedScene; 	//for Shadowing

	boost::shared_ptr<osg_Init> mOsgInit;
	boost::shared_ptr<osg_geom> mOsgGeom;
	boost::shared_ptr<osg_Update> mOsgUpdater;

	//Background image
	IplImage	*mGLImage;
	osg::Image	*mVideoImage;

	double	mAddModelAnimation;
	bool	mAddArModel;

public:
	osg_Root();
	~osg_Root();

	//init parameter
	void osg_init(double *projMatrix);
	void osg_uninit();
	void osg_inittracker(std::string markerName, int maxLengthSize, int maxLengthScale);

	//update
	void osg_update(std::vector <osg::Quat> q_array, std::vector<osg::Vec3d>  v_array);

	//rendering window
	void osg_render(IplImage *newFrame, CvMat *cParams, CvMat *cDistort);

	//set ground parameter
	void osg_UpdateHeightfieldTrimesh(float *ground_grid);
	void setOSGTrimeshScale(float scale);
	void ShowGroundGeometry();
	void HideGroundGeometry();

	//add a new object to osg scene
	void osgAddObjectNode(osg::Node* node);

	//control virtual visual hand
	void osg_createHand(int index, float x, float y, float world_scale, float ratio);
	void osg_UpdateHand(int index, float *x, float *y, float *grid);

	//control AR menu
	void OsgInitMenu();
	void OsgInitModelMenu();
	void ModelButtonInput();
	void ToggleMenuVisibility();
	void ToggleModelButtonVisibility();
	void ToggleVirtualObjVisibility();
	bool IsMenuVisibiilty();
	bool IsModelButtonVisibiilty();
	void ModelButtonAnimation();
	void ResetModelButtonPos();
	void SetAddArModel(const bool & b);
	void SetARAddModelButtton(const int & input);
	int  GetARModelButton(void) const;

	//add function for AR input
	osg::Vec3 GetARMenuPos(); 
	int	 AddNewObjFromARButton(const int & index);

	
	bool isAddArModel() const {
		return mAddArModel;
	}

	void setAddArModel(bool addArModel) {
		mAddArModel = addArModel;
	}

	double getAddModelAnimation() const {
		return mAddModelAnimation;
	}

	void setAddModelAnimation(double addModelAnimation) {
		mAddModelAnimation = addModelAnimation;
	}

	void ResetArModelButtonType();
	void ResetPanelCond();
	void ResetAddModelMode();

private:
	osg::Drawable* createSquare(const osg::Vec3& corner,const osg::Vec3& width,const osg::Vec3& height, osg::Image* image=NULL);
	osg::Drawable* createAxis(const osg::Vec3& corner,const osg::Vec3& xdir,const osg::Vec3& ydir,const osg::Vec3& zdir);
	osg::Node* createMilestone();
	void CreateGround();
};
#endif