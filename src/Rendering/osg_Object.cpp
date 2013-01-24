/*
 * osg_Object.cpp
 *
 *  Created on: 2012/12/20
 *      Author: umakatsu
 */
#include "Rendering\osg_Object.h"

#include <cassert>

osg_Object::osg_Object()
{
	Virtual_Objects_Count = 0;
}

osg_Object::~osg_Object()
{
	obj_node_array.clear();
	obj_transform_array.clear();

	hand_object_array.clear();
	hand_object_global_array.clear();
	REP(i,MAX_NUM_HANDS)
	{
		hand_object_transform_array[i];
	}
	hand_object_shape_array.clear();
}

void osg_Object::osg_resetNodes()
{
	obj_node_array.clear();
	obj_transform_array.clear();
}

void osg_Object::SetObjTransformArrayIndex( const int & idx, osg::Quat quat, osg::Vec3d vec)
{
	obj_transform_array.at(idx)->setAttitude(quat);
	obj_transform_array.at(idx)->setPosition(vec);
}

osg::ref_ptr<osg::Node>  osg_Object::GetObjNodeIndex(const int & idx) const
{
	assert( idx >= 0 && idx < this->obj_node_array.size());
	return obj_node_array[idx];
}

int osg_Object::SizeObjNodeArray(void) const
{
	return obj_node_array.size();
}

int osg_Object::SizeObjTransformArray(void) const
{
	return obj_transform_array.size();
}

void osg_Object::SetScale2ObjTrans(const int & index, const float & scale)
{
	obj_transform_array.at(index)->setScale(osg::Vec3d(scale,scale,scale));
}

void osg_Object::SetMask2ObjTrans(const int & index, short mask)
{
	obj_transform_array.at(index)->setNodeMask(mask);
}
