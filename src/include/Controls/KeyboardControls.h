#ifndef KEYBOARD_CONTROLS_H
#define KEYBOARD_CONTROLS_H

#include "Controls.h"

//OpenCV
#include "opencv/cv.h"

class osg_geom;

class KeyboardController
{
public:
	KeyboardController();
	~KeyboardController();

	int	check_input(boost::shared_ptr<osg_Root> osgRoot, bt_ARMM_world *world);
	int TransmitInput(const int & input);

private:
	bool RegisteringObject(const char * filename, boost::shared_ptr<osg_Root> osg_Root);
	void AssignmentKeyinput(const char * settingFilename);
	inline bool getKey(int key);

private:
	std::vector<std::pair<unsigned int, std::string> > mKeyAssignment;
	boost::shared_ptr<osg_geom> mOsgGeom;
};
#endif