/*
 * osg_Init.cpp
 *
 *  Created on: 2012/12/20
 *      Author: umakatsu
 */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "Rendering\osg_Init.h"
#include "Rendering\osg_Object.h"

#include "UserConstant.h"

#include <osgDB/ReadFile>

#include <iostream>
#include <cassert>

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
osg_Init::osg_Init()
{}

osg_Init::~osg_Init()
{}

void osg_Init::CreateHeightfield(boost::shared_ptr<osg_Object> osgObject)
{
	if(osgObject == NULL)
	{
		std::cerr << "Error: No osg object is created in the stage of init!!" << std::endl;
		exit(EXIT_SUCCESS);
	}

	osgObject->mHeightFieldPoints			= new osg::Vec3Array;
	osgObject->mHeightFieldGeometry_quad	= new osg::Geometry;
	osgObject->mHeightFieldGeometry_line	= new osg::Geometry;
	osgObject->mGroundQuadColor				= new osg::Vec4Array;
	osgObject->mGroundLineColor				= new osg::Vec4Array;

	for (int i=0; i<159; i++) 
	{
		for (int j=0; j<119; j++) 
		{
			osgObject->mHeightFieldPoints->push_back(osg::Vec3(i-159, j-119, 0));
			osgObject->mHeightFieldPoints->push_back(osg::Vec3(i-158, j-119, 0));
			osgObject->mHeightFieldPoints->push_back(osg::Vec3(i-158, j-118, 0));
			osgObject->mHeightFieldPoints->push_back(osg::Vec3(i-159, j-118, 0));
		}
	}

}

void osg_Init::SetHeightfield(boost::shared_ptr<osg_Object> osgObject)
{
		osgObject->mHeightFieldGeometry_quad->setVertexArray(osgObject->mHeightFieldPoints); 
		osgObject->mHeightFieldGeometry_quad->addPrimitiveSet(new osg::DrawArrays( GL_QUADS, 0, osgObject->mHeightFieldPoints->size()));
		osgObject->mHeightFieldGeometry_quad->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

		osgObject->mHeightFieldGeometry_line->setVertexArray(osgObject->mHeightFieldPoints); 
		osgObject->mHeightFieldGeometry_line->addPrimitiveSet(new osg::DrawArrays( GL_LINES, 0, osgObject->mHeightFieldPoints->size()));
		osgObject->mHeightFieldGeometry_line->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);		

		//Set the Heightfield to be alpha invisible
		osgObject->mHeightFieldGeometry_quad->setColorBinding(osg::Geometry::BIND_OVERALL); 
		osgObject->mHeightFieldGeometry_line->setColorBinding(osg::Geometry::BIND_OVERALL); 

		//osg::Vec4Array* col = new osg::Vec4Array(); 
		osgObject->mHeightFieldGeometry_quad->setColorArray(osgObject->mGroundQuadColor); 
		osgObject->mHeightFieldGeometry_line->setColorArray(osgObject->mGroundLineColor);
		osgObject->mGroundQuadColor->push_back(osg::Vec4(1,1,1,0.0));
		osgObject->mGroundLineColor->push_back(osg::Vec4(1,1,1,0.0));
}
