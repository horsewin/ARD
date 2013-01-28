#ifndef _HAND_REGION_H_
#define _HAND_REGION_H_

#ifndef OPENCV
#define OPENCV
	#include <opencv/cv.h>
	#include <opencv/highgui.h>
	//#include <opencv2/opencv.hpp>
#endif
#include "GMM.h"

#define ALPHA 0.9

class HandRegion
{
private:

    GMM     mSkinColor;
    GMM     mNonSkinColor;

    IplImage *      mImage;            // Hand Region Image ( binary )

	//for segmentation hand region
	std::vector <CvRect>	cont_boundbox; 
	std::vector <CvBox2D>	cont_boundbox2D; 
	std::vector <CvPoint>	cont_center;

public:
    HandRegion(void);
    ~HandRegion(void);

    bool LoadSkinColorProbTable();


    IplImage * GetHandRegion( IplImage * srcImage, int *cont_num, std::vector <CvRect> & cont_boundbox,  std::vector <CvBox2D> & cont_boundbox2D, std::vector <CvPoint> & cont_center);

    IplImage * QueryHandRegion() { return mImage; };

	int FindHands(IplImage *depthIm, IplImage *colourIm, IplImage *transDepth320, IplImage *transColor320);

private:
	void DetectFingertips(cv::Ptr<IplImage> handMask, std::vector< std::vector<cv::Point> > & fingerTips);
	int FindNumHandPixels(float depth);
};

#endif // _HAND_REGION_H_
