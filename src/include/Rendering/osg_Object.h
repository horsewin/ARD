/*
 * osg_Object.h
 *
 *  Created on: 2012/12/20
 *      Author: umakatsu
 */

#ifndef OSG_OBJECT_H_
#define OSG_OBJECT_H_

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>

#include <vector>

#include "UserConstant.h"

class osg_Object
{
public:
	osg_Object();
	~osg_Object();

	void SetObjTransformArrayIndex( const int & idx, osg::Quat quat, osg::Vec3d vec);
	void PushObjNodeArray(osg::Node* n){ obj_node_array.push_back(n); }
	osg::ref_ptr<osg::Node> GetObjNodeIndex(const int & idx) const;
	int SizeObjNodeArray(void) const ;
	int SizeObjTransformArray(void) const;

	void setObjectIndex(int objectIndex) {
		this->objectIndex = objectIndex;
	}

	int getVirtualObjectsCount() const {
		return Virtual_Objects_Count;
	}

	int getObjectIndex() const {
		return objectIndex;
	}

	osg::ref_ptr<osg::PositionAttitudeTransform> getObjTexturePosAtt() const {
		return mObjTexturePosAtt;
	}

	void setObjTexturePosAtt(
			osg::ref_ptr<osg::PositionAttitudeTransform> objTexture) {
		mObjTexturePosAtt = objTexture;
	}

	osg::ref_ptr<osg::Node> getObjTexture() const {
		return objTexture;
	}

	void setObjTexture(osg::ref_ptr<osg::Node> objTexture) {
		this->objTexture = objTexture;
	}

	void SetScale2ObjTrans(const int & index, const float & scale);
	void SetMask2ObjTrans(const int & index, short mask);

	void IncrementObjCount(void){ Virtual_Objects_Count++; }
	void DecrementObjCount(void){ Virtual_Objects_Count--; }
	void IncrementObjIndex(void){ objectIndex++; }
	void IncrementObjIndex(const int & val){ objectIndex+=val; }
	void DecrementObjIndex(void){ objectIndex--; }

	void osg_resetNodes();

	//------------------------------------------------------
	//setter and getter for virtual objects----->
	//------------------------------------------------------
	std::vector<osg::ref_ptr<osg::Node>> getObjNodeArray() const {
		return obj_node_array;
	}

	void setObjNodeArray(std::vector<osg::ref_ptr<osg::Node>> objNodeArray) {
		obj_node_array = objNodeArray;
	}

	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform>> getObjTransformArray() const {
		return obj_transform_array;
	}

	void setObjTransformArray(
			std::vector<osg::ref_ptr<osg::PositionAttitudeTransform>> objTransformArray) {
		obj_transform_array = objTransformArray;
	}

	//------------------------------------------------------
	//setter and getter for soft texture----->
	//------------------------------------------------------
	bool isSoftTexture() const {
		return softTexture;
	}

	void setSoftTexture(bool softTexture) {
		this->softTexture = softTexture;
	}

	//------------------------------------------------------
	//setter and getter for hand objects----->
	//------------------------------------------------------
	std::vector<osg::ref_ptr<osg::Node> > getHandObjectArray() const {
		return hand_object_array;
	}

	void setHandObjectArray(
			std::vector<osg::ref_ptr<osg::Node> > handObjectArray) {
		hand_object_array = handObjectArray;
	}

	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > getHandObjectGlobalArray() const {
		return hand_object_global_array;
	}

	void setHandObjectGlobalArray(
			std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > handObjectGlobalArray) {
		hand_object_global_array = handObjectGlobalArray;
	}

	std::vector<osg::ref_ptr<osg::ShapeDrawable> > getHandObjectShapeArray() const {
		return hand_object_shape_array;
	}

	void setHandObjectShapeArray(
			std::vector<osg::ref_ptr<osg::ShapeDrawable> > handObjectShapeArray) {
		hand_object_shape_array = handObjectShapeArray;
	}

public:

	//HeightField
	osg::ref_ptr<osg::Vec3Array>	mHeightFieldPoints;
	osg::ref_ptr<osg::Geometry>		mHeightFieldGeometry_quad;
	osg::ref_ptr<osg::Geometry>		mHeightFieldGeometry_line;
	osg::ref_ptr<osg::Vec4Array>	mGroundQuadColor;
	osg::ref_ptr<osg::Vec4Array>	mGroundLineColor;

	//TODO this variable should be private
	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform>> hand_object_transform_array[MAX_NUM_HANDS];

private:
	//osg_root‚©‚çˆÚ“®-->//
	//for virtual objects
	std::vector<osg::ref_ptr<osg::Node>> obj_node_array;
	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform>> obj_transform_array;

	std::vector<osg::ref_ptr<osg::Node>> hand_object_array;

	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform>> hand_object_global_array;
	std::vector<osg::ref_ptr<osg::ShapeDrawable>> hand_object_shape_array;
	//<--osg_root‚©‚çˆÚ“®//

	//for virtual objects
	int Virtual_Objects_Count;

	int objectIndex;

	//for soft texture
	osg::ref_ptr<osg::Node> objTexture;
	osg::ref_ptr<osg::PositionAttitudeTransform> mObjTexturePosAtt;
	bool softTexture;
};

#endif