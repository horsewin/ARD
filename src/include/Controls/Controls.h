#ifndef CONTROLS_H
#define CONTROLS_H

#include <boost\shared_ptr.hpp>

class osg_Root;
class bt_ARMM_world;

class Controller 
{
	public:
		Controller();
		int check_input(boost::shared_ptr<osg_Root> osgRoot, bt_ARMM_world *world);
};

#endif