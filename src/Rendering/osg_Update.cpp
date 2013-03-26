/*
 * osg_Update.cpp
 *
 *  Created on: 2013/1/18
 *      Author: umakatsu
 */
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "Rendering\osg_Update.h"

#include "Rendering\osg_Object.h"
#include "constant.h"

#include <osg/MatrixTransform>

#include <cassert>
#include <deque>
//---------------------------------------------------------------------------
// Constant/Define
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
extern std::vector<int> fingersIdx;
extern std::vector<float*> fingersPos;
std::deque<float> posDiff;
extern std::vector<int> fingerIndex;
extern bool  bFingerPinchState;
namespace
{
	template< class T>
	bool VectorBoundChecker(std::vector<T> v, int idx)
	{
		return( v.size() > static_cast<unsigned int>(idx) && idx >= 0);
	}

	float FloatNorm3(float a[3], float b[3])
	{
		return( std::sqrt( std::pow(a[0]-b[0], 2) + std::pow(a[1]-b[1], 2) + std::pow(a[2]-b[2], 2) ) );
	}
}

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
osg_Update::osg_Update()
{}

osg_Update::~osg_Update()
{}

void osg_Update::UpdateHand(boost::shared_ptr<osg_Object> osgObject, int index, float *x, float *y, float *grid)
{
	const int DEPTH_SCALE = 10; // to convert cm to mm scale

	//create temporary osg contents from osgObject
	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > pHandObjGlobalArray	= osgObject->getHandObjectGlobalArray();
	std::vector<osg::ref_ptr<osg::ShapeDrawable> >pHandObjShapeArray				= osgObject->getHandObjectShapeArray();

	if(!VectorBoundChecker(pHandObjGlobalArray, index)) return;

	//走査方向はKinect viewからみたときのtop-leftからtop-rightに向かう
	for(int i = 0; i < MIN_HAND_PIX; i++) 
	{
		//走査方向はKinect viewからみたときのbottom-leftからtop-leftに向かう
		for(int j = 0; j < MIN_HAND_PIX; j++) 
		{
			int curr = i*MIN_HAND_PIX+j;
			//osg::Vec3d pos = hand_object_transform_array[index].at(curr)->getPosition();
			if(grid[curr] > 0 && grid[curr]<100 )
			{
				osgObject->hand_object_transform_array[index].at(curr)->setPosition(osg::Vec3d(x[curr]*ARMM::ConstParams::SPHERE_SCALE, y[curr]*ARMM::ConstParams::SPHERE_SCALE, grid[curr]*DEPTH_SCALE));

				//printf("Pos, sensor %d = %f, %f, %f\n", curr, x[curr], y[curr], grid[curr]);
				pHandObjShapeArray[curr]->setColor(osg::Vec4d(0.9451, 0.7333, 0.5765, 1));
			}
			else
			{
				osgObject->hand_object_transform_array[index].at(curr)->setPosition(osg::Vec3d(x[curr]*ARMM::ConstParams::SPHERE_SCALE, y[curr]*ARMM::ConstParams::SPHERE_SCALE, -5000*DEPTH_SCALE));
				//if(curr%10 == 0)
				//	printf("Pos, sensor %d = %f, %f, %f\n", curr, x[curr], y[curr], grid[curr]);
			}
		}
	}

	//assign fingertips
	//OpenCVウィンドウで検出した指先位置とOSGで表現されている手の位置のマッチングをとる
	fingersIdx.clear();
	fingersPos.clear();
	REP(fingerTips, fingerIndex.size())
	{
		int idx =	fingerIndex.at(fingerTips);

		//if(grid[idx] > 0 && grid[idx]<100){}
		//else{
		bool fitting = false;
		int shift = 1;
		float *tmpPos = new float[3];
		int numDetectedTips = 0;
		do
		{
			int tmpIdx;
			for(int i=-shift; i<=shift; i++)
			{
				for(int j=-shift; j<=shift; j++)
				{
					tmpIdx = idx + (i*MIN_HAND_PIX+j);
					if(tmpIdx < 0 || tmpIdx >= HAND_GRID_SIZE) continue;
					if(grid[tmpIdx] > 0 && grid[tmpIdx]<100)
					{
						fingersIdx.push_back(tmpIdx);
						tmpPos[0] += x[tmpIdx];
						tmpPos[1] += y[tmpIdx];
						tmpPos[2] += grid[tmpIdx];
						numDetectedTips++;
						//fitting = true;
						pHandObjShapeArray[tmpIdx]->setColor(osg::Vec4(1, 0, 0, 1));
					}
				}
				//if(fitting) break;
			}
			shift++;
		//}while(!fitting || shift<=2);
		}while(shift<=2);

		tmpPos[0]/=3.0;
		tmpPos[1]/=3.0;
		tmpPos[2]/=3.0;

		fingersPos.push_back(tmpPos);
//		}
	}

	if(fingersPos.size()>=2)
	{
		float posCurrentDiff = FloatNorm3(fingersPos.at(0), fingersPos.at(1));
		bFingerPinchState = false;
		
		//3フレーム分の距離データがあるかどうか
		if(posDiff.size() == 3)
		{
			if(posDiff[0] > posCurrentDiff && 			
			   posDiff[1] > posCurrentDiff &&
			   posDiff[2] > posCurrentDiff
			)
			//printf("PINCH+%f\n",posCurrentDiff);
			bFingerPinchState = true;
			posDiff.pop_front();
		}

		posDiff.push_back(posCurrentDiff);
	}
	//write back to tmp variables
	osgObject->setHandObjectGlobalArray(pHandObjGlobalArray);
	osgObject->setHandObjectShapeArray(pHandObjShapeArray);
}

void osg_Update::UpdateObjects(boost::shared_ptr<osg_Object> osgObject, std::vector <osg::Quat> q_array, std::vector<osg::Vec3d>  v_array)
{
	//create temporary osg contents from osgObject
	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > pObjTransArray = osgObject->getObjTransformArray();

	assert(pObjTransArray.size() == v_array.size());
	assert(pObjTransArray.size() == q_array.size());

	for(unsigned int i = 0; i < pObjTransArray.size(); i++) 
	{
		pObjTransArray.at(i)->setAttitude(q_array.at(i));
		pObjTransArray.at(i)->setPosition(v_array.at(i));
		////for debug
		//osg::Vec3 pos = pObjTransArray.at(i)->getPosition();
		//printf("%d,%s:%.2f  %.2f  %.2f \n", i, obj_node_array.at(i)->getName().c_str(), v_array.at(i).x(), v_array.at(i).y(), v_array.at(i).z());
		//printf("POS=(%.2f, %.2f, %.2f)\n",pos.x(), pos.y(), pos.z());
	}

	//write back to tmp variables
	osgObject->setObjTransformArray(pObjTransArray);

}
