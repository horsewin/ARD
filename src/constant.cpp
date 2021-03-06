/*
 * constant.cpp
 *
 *  Created on: 2012/09/19
 *      Author: umakatsu
 */

#include "constant.h"
#include "UserConstant.h"

namespace ARMM
{
	const int	ConstParams::MARKER_WIDTH = 400; //400mm=40cm

	const char * ConstParams::DATABASEDIR = "C:/Users/Frontier2/Dropbox/Lab/ModelDatabase/";
	const char * ConstParams::MENUDATADIR = "C:/Users/Frontier2/Dropbox/Lab/MenuDatabase/";

	const float ConstParams::HAND_BOX_CM = 20.f; // 25 cm
	const double ConstParams::KINECT_PIX_PER_DEPTH = 0.4610778;
	const int	ConstParams::GRID_SIZE =19200;
	const int	ConstParams::WORLD_DIV = 4;

	const float ConstParams::SPHERE_SIZE	= 2;
	const float ConstParams::CUBE_SIZE		= 4;

	const int	ConstParams::SKIN_X = 640;
	const int	ConstParams::SKIN_Y = 480;	
	const int	ConstParams::HAND_MAX_TRANSMIT_SIZE = 120;

	//const float MIN_HAND_PIX = 15; // 11 pixels
	//const int HAND_GRID_SIZE = 225;// 15x15
	const int	ConstParams::NUM_OBJECTS = 10;

	const int ConstParams::COLLISION_PARAM	= 1;

	//window size
	const int ConstParams::WINDOW_WIDTH  = 1280;
	const int ConstParams::WINDOW_HEIGHT = 960;

	//for virtual objects
	const int ConstParams::MAX_NUM_VIR_OBJECT = 5;

	//for virtual hands
	const int ConstParams::SPHERE_SCALE = 10;
	const float ConstParams::HAND_THICKNESS = 1.5;

}
