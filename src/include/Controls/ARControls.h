#ifndef ARCONTROLS_H
#define ARCONTROLS_H

#include "Controls.h"

//OpenCV
#include "opencv/cv.h"

class ARController: public Controller 
{
public:
	ARController(bt_ARMM_world *m_world) : Controller(m_world){};
	int check_input(boost::shared_ptr<osg_Root> osgRoot);

private:

};

#endif