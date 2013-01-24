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
#include "Rendering\osg_geom_data.h"

#include "Rendering\osg_Object.h"
#include "UserConstant.h"
#include "constant.h"

#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/StateSet>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/PositionAttitudeTransform>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>

//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
namespace{
	const osg::Quat DEFAULTATTIDUTE = 
		osg::Quat(
			osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 0.0, 1.0)
		);
}

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
osg_geom::osg_geom()
{}

osg_geom::~osg_geom()
{}

osg::Geode* osg_geom::create3dsModel() 
{
	osg::Vec3Array			*vertexArray	= new osg::Vec3Array();
	osg::DrawElementsUInt	*faceArray		= new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
	osg::Vec3Array			*normalArray	= new osg::Vec3Array();

	// normal index
	osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 24, 4> *normalIndexArray;
	normalIndexArray = new osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 24, 4>();

	//setting attributes of geometry
	osg::Geometry *geometry = new osg::Geometry();
	geometry->setVertexArray(vertexArray);
	geometry->setNormalArray(normalArray);
	geometry->setNormalIndices(normalIndexArray);
	geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	//geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geometry->addPrimitiveSet(faceArray);

	osg::Vec4Array* color = new osg::Vec4Array();     
	color->push_back( osg::Vec4( 1, 0, 0, 0.5 ) );    
	geometry->setColorArray( color );
	geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->addDrawable(geometry);
	return geode.release();
}

osg::Vec3 osg_geom::asOsgVec3( const btVector3& v )  
{
	return osg::Vec3( v.x(), v.y(), v.z() ); 
}  


osg::Node* osg_geom::osgNodeFromBtBoxShape(float cube_size, const btTransform& trans) {
	//osg::ref_ptr< osg::Geode > cube = createCube();
	float scale = 10;
	float x = trans.getOrigin().getX()*scale;
	float y = trans.getOrigin().getY()*scale;
	float z = trans.getOrigin().getZ()*scale;
	btQuaternion quat = trans.getRotation();
	osg::Quat q = osg::Quat(quat.getX(), quat.getY(), quat.getZ(), quat.getW());
	osg::ref_ptr< osg::Box > cube = new osg::Box(osg::Vec3d(0,0,0),cube_size);
	osg::ShapeDrawable * shape = new osg::ShapeDrawable( cube );
    osg::ref_ptr< osg::Geode> geode = new osg::Geode();
    geode->addDrawable( shape );
	osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
	mt->setScale(osg::Vec3d(scale,scale,scale));
	mt->setAttitude(q);
	mt->setPosition(osg::Vec3d(x, y, z));
	mt->addChild( geode.get() );
	return mt.release();
}

//Convert from btConvexHullShape into osg's node
osg::Node* osg_geom::osgNodeFromBtCollisionShape( const btConvexHullShape* hull, const btTransform& trans )  
{
	btShapeHull sh( hull );
	sh.buildHull( 0. );
	int nVerts( sh.numVertices () );
	int nIdx( sh.numIndices () ); 
	if( (nVerts <= 0) || (nIdx <= 0) )     
		return( NULL );       
	const btVector3* bVerts( sh.getVertexPointer() );
	const unsigned int* bIdx( sh.getIndexPointer() );
	osg::Vec3Array* v = new osg::Vec3Array();
	v->resize( nVerts );
	unsigned int idx;  
	for( idx = 0; idx < (unsigned int)nVerts; idx++ )        
		( *v )[ idx ] = asOsgVec3( bVerts[ idx ] );

	osg::DrawElementsUInt* deui = new osg::DrawElementsUInt( GL_TRIANGLES );   

	for( idx = 0; idx < (unsigned int)nIdx; idx++ )         
		deui->push_back( bIdx[ idx ] );      

	osg::Vec4Array* color = new osg::Vec4Array();     
	color->push_back( osg::Vec4( 1., 1., 1., 1. ) );    

	osg::Geometry* geom = new osg::Geometry;     
	geom->setVertexArray( v );
	geom->setColorArray( color );
	geom->setColorBinding( osg::Geometry::BIND_OVERALL );      
	geom->addPrimitiveSet( deui );
	osg::ref_ptr< osg::Geode > geode = new osg::Geode();   
	geode->addDrawable( geom ); 
	btQuaternion quat = trans.getRotation();
	osg::Quat q = osg::Quat(quat.getX(), quat.getY(), quat.getZ(), quat.getW());
	float scale = 10;
	float x = trans.getOrigin().getX()*scale;
	float y = trans.getOrigin().getY()*scale;
	float z = trans.getOrigin().getZ()*scale;
	osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
	mt->setScale(osg::Vec3d(scale,scale,scale));
	mt->setAttitude(q);
	mt->setPosition(osg::Vec3d(x, y, z));
	mt->addChild( geode.get() );
	return mt.release();
}  

osg::Node* osg_geom::osgNodeFromBtSphere(float sphere_size, const btTransform& trans) {
	float scale = 10;
	float x = trans.getOrigin().getX()*scale;
	float y = trans.getOrigin().getY()*scale;
	float z = trans.getOrigin().getZ()*scale;
	btQuaternion quat = trans.getRotation();
	osg::Quat q = osg::Quat(quat.getX(), quat.getY(), quat.getZ(), quat.getW());
	osg::ref_ptr< osg::Sphere > sphere = new osg::Sphere(osg::Vec3d(0,0,0), sphere_size);
	osg::ShapeDrawable * shape = new osg::ShapeDrawable( sphere );
    osg::ref_ptr< osg::Geode> geode = new osg::Geode();
    geode->addDrawable( shape );
	osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
	mt->setScale(osg::Vec3d(scale,scale,scale));
	mt->setAttitude(q);
	mt->setPosition(osg::Vec3d(x, y, z));
	mt->addChild( geode.get() );
	return mt.release();
}

osg::Node* osg_geom::osgNodeFrom3dsModel(std::string modelname, const double & scale_3ds, const btTransform& trans) 
{
	//cout << "object scale = " << scale_3ds << endl;
	float scale = 10;
	float x = trans.getOrigin().getX()*scale;
	float y = trans.getOrigin().getY()*scale;
	float z = trans.getOrigin().getZ()*scale;
	btQuaternion quat = trans.getRotation();
	osg::Quat q = osg::Quat(quat.getX(), quat.getY(), quat.getZ(), quat.getW());

	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(modelname.c_str());
	model->setName(modelname);

	//osg::ref_ptr< osg::Sphere > sphere = new osg::Sphere(osg::Vec3d(0,0,0), sphere_size);
	//osg::ShapeDrawable * shape = new osg::ShapeDrawable( sphere );
 //   osg::ref_ptr< osg::Geode> geode = new osg::Geode();
 //   geode->addDrawable( shape );
	osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
	//scale = scale_3ds * 10; //this scale set 10 times value against bullet scale(ref:bt_ARMM_world.cpp)
	mt->setScale(osg::Vec3d(scale,scale,scale));
	mt->setAttitude(q);
	mt->setPosition(osg::Vec3d(x, y, z));
	mt->addChild(model);

	return mt.release();
}

void osg_geom::osgAddObjectNode(osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene, 
								boost::shared_ptr<osg_Object> osgObject, 
								osg::Node* n)
{
	osgObject->PushObjNodeArray(n);

	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > pTransArray = osgObject->getObjTransformArray();
	pTransArray.push_back(new osg::PositionAttitudeTransform());

	//set default parameter to transform array
	int index = pTransArray.size()-1;
	pTransArray.at(index)->setAttitude( DEFAULTATTIDUTE );
	pTransArray.at(index)->setPosition( osg::Vec3d(0.0, 0.0, 0.0) );
	pTransArray.at(index)->setNodeMask( castShadowMask );

	pTransArray.at(index)->addChild(osgObject->GetObjNodeIndex(index));
	shadowedScene->addChild( pTransArray.at(index) );
	pTransArray.at(index)->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
	shadowedScene->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");

	osgObject->IncrementObjIndex();

	osgObject->setObjTransformArray(pTransArray);
}

void osg_geom::CreateVirtualHand(boost::shared_ptr<osg_Object> osgObject, int index, float x, float y, float world_scale, float ratio) 
{
//	float sphere_size = 0.5;
	float sphere_size = world_scale * ratio;
	//float sphere_size = world_scale; // test
	const int SPHERE_SCALE = ARMM::ConstParams::SPHERE_SCALE;

	printf("Hand%d Created : ws=%f, ratio=%f\n", index, world_scale, ratio);

	//create temporary osg contents from osgObject
	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > pHandObjGlobalArray	= osgObject->getHandObjectGlobalArray();
	std::vector<osg::ref_ptr<osg::ShapeDrawable> >pHandObjShapeArray				= osgObject->getHandObjectShapeArray();

	pHandObjGlobalArray.push_back(new osg::PositionAttitudeTransform());

	for(int i = 0; i < MIN_HAND_PIX; i++) 
	{
		for(int j = 0; j < MIN_HAND_PIX; j++) 
		{
			//create a part of hands 
			osg::ref_ptr< osg::Sphere > sphere = new osg::Sphere(osg::Vec3d(0,0,0), sphere_size);
		    osg::ref_ptr< osg::ShapeDrawable> shape = new osg::ShapeDrawable( sphere.get() );
			shape->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			shape->setColor(osg::Vec4(1, 1, 0, 1));
			osg::ref_ptr< osg::Geode> geode = new osg::Geode();
			pHandObjShapeArray.push_back(shape.get()); //<--- register
			geode->addDrawable( shape.get() );

			osgObject->hand_object_transform_array[index].push_back(new osg::PositionAttitudeTransform());

			int curr = osgObject->hand_object_transform_array[index].size()-1;
			osgObject->hand_object_transform_array[index].at(curr)->setScale(osg::Vec3d(SPHERE_SCALE,SPHERE_SCALE,SPHERE_SCALE));
			osgObject->hand_object_transform_array[index].at(curr)->setPosition(osg::Vec3d(j*SPHERE_SCALE, i*SPHERE_SCALE, 1000));
			osgObject->hand_object_transform_array[index].at(curr)->addChild( geode.get() );
			osgObject->hand_object_transform_array[index].at(curr)->setNodeMask(rcvShadowMask);

			pHandObjGlobalArray.at(index)->addChild( osgObject->hand_object_transform_array[index].at(curr));
		}
	}

	pHandObjGlobalArray.at(index)->setPosition(osg::Vec3d(0,0,0));

	//set rendering order
	pHandObjGlobalArray.at(index)->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");

	//write back to tmp variables
	osgObject->setHandObjectGlobalArray(pHandObjGlobalArray);
	osgObject->setHandObjectShapeArray(pHandObjShapeArray);
}