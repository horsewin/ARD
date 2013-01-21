#ifndef OSGGEOMDATA_H
#define OSGGEOMDATA_H

#include <osg/Geode>
#include <osgShadow/ShadowedScene>

#include <LinearMath\btVector3.h>
#include <LinearMath\btTransform.h>
#include <BulletCollision\CollisionShapes\btConvexHullShape.h>
#include <BulletCollision\CollisionShapes\btShapeHull.h>

#include <boost\shared_ptr.hpp>

class osg_Object;
class osg_geom
{
public:
	osg_geom();
	~osg_geom();

	osg::Geode*	create3dsModel();
	void osgAddObjectNode(osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene, boost::shared_ptr<osg_Object> osgObject, osg::Node* n);

	//hand object
	void CreateVirtualHand(boost::shared_ptr<osg_Object> osgObject, int index, float x, float y, float world_scale, float ratio);

	osg::Node*	osgNodeFromBtBoxShape(float cube_size, const btTransform& trans);
	osg::Node*	osgNodeFromBtCollisionShape( const btConvexHullShape* hull, const btTransform& trans ) ;//Convert from btConvexHullShape into osg's node
	osg::Node*	osgNodeFromBtSphere(float sphere_size, const btTransform& trans);
	osg::Node*	osgNodeFrom3dsModel(std::string modelname, const double & scale_3ds, const btTransform& trans) ;

private:
	osg::Vec3	asOsgVec3( const btVector3& v ) ;
};

#endif