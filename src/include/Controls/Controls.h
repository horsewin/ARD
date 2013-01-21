#ifndef CONTROLS_H
#define CONTROLS_H

#include <boost\shared_ptr.hpp>

class osg_Root;
class Controller 
{
	public:
		Controller();
		void check_input(boost::shared_ptr<osg_Root> osgRoot);
};

#endif