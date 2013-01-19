#ifndef OSGUPDATE_H
#define OSGUPDATE_H


#include "UserConstant.h"

#include "boost\shared_ptr.hpp"

class osg_Object;
class osg_Init;
class osg_geom;

class osg_Update
{
public:
	osg_Update();
	~osg_Update();

	//control virtual visual hand
	void UpdateHand(boost::shared_ptr<osg_Object> osgObject, int index, float *x, float *y, float *grid);

};

#endif