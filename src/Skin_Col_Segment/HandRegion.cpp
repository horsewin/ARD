/*
 * HandRegions.cpp
 *
 *
 *  Created on: 2011/11/11
 *      Author: umakatsu
 */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "Skin_Col_Segment\HandRegion.h"
#include "UserConstant.h"
#include "constant.h"

#include <stdio.h>
#include <vector>

using namespace std;

//---------------------------------------------------------------------------
// Constant/Define
//---------------------------------------------------------------------------
const int		CVCLOSE_ITR	 = 1;
const CvScalar	CV_CVX_WHITE = cvScalar(255);
const CvScalar	CV_CVX_BLACK = cvScalar(0);
const int		CVCONTOUR_APPROX_LEVEL = 1;

//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
extern float WORLD_SCALE;
extern IplImage *depthmask_for_mesh;
extern vector<int> fingerIndex;

#ifdef USE_SKIN_SEGMENTATION
extern CvPoint curr_hands_corners[MAX_NUM_HANDS];
extern CvPoint prev_hands_corners[MAX_NUM_HANDS];
extern float hand_depth_grids[MAX_NUM_HANDS][HAND_GRID_SIZE];
extern float curr_hands_ratio[MAX_NUM_HANDS];
extern double showFPS;
#endif

namespace
{
// Remove certain contour with area less than threshold
void removeNoise( IplImage* src, int size )
{    
	IplImage* tmp = cvCreateImage(cvSize(src->width, src->height),IPL_DEPTH_8U,1);
	cvCopyImage(src, tmp);
	CvMemStorage* storage   = cvCreateMemStorage( 0 );     
	CvSeq* contours         = NULL;    
	CvScalar black          = CV_RGB( 0, 0, 0 ); 
	CvScalar white          = CV_RGB( 255, 255, 255 ); 
    double area;   
	cvFindContours( tmp, storage, &contours, sizeof( CvContour ), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );  

	while( contours )   {  
		area = cvContourArea( contours, CV_WHOLE_SEQ );       
		if( fabs( area ) <= size )  {// if less than threshold then remove by paint over white by black           
			cvDrawContours( src, contours, black, black, -1, CV_FILLED, 8 );      
		}     
		contours = contours->h_next;   
	}   
	cvReleaseMemStorage( &storage );    
	cvReleaseImage(&tmp);
}

	//Connected component(Labeling) from opencv book example 
void getConnectedComponents(IplImage *mask, int poly1_hull0, float perimScale, int *cont_num, std::vector <CvRect> & cont_boundbox, std::vector <CvBox2D> & cont_boundbox2D, std::vector <CvPoint> & cont_center) {
		static CvMemStorage*	mem_storage	= NULL;
		static CvSeq*			contours	= NULL;

		//CLEAN UP RAW MASK
//		cvMorphologyEx( mask, mask, NULL, NULL, CV_MOP_OPEN, CVCLOSE_ITR );
//		cvMorphologyEx( mask, mask, NULL, NULL, CV_MOP_CLOSE, CVCLOSE_ITR );

//		removeNoise(mask,20);

		//FIND CONTOURS AROUND ONLY BIGGER REGIONS
		if( mem_storage==NULL ) mem_storage = cvCreateMemStorage(0);
		else cvClearMemStorage(mem_storage);

		CvContourScanner scanner = cvStartFindContours(mask,mem_storage,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
		CvSeq* c;
		int numCont = 0;
		while( (c = cvFindNextContour( scanner )) != NULL )
		{
			double len = cvContourPerimeter( c );
			double q = (mask->height + mask->width) /perimScale;   //calculate perimeter len threshold

			if( len < q ) //Get rid of blob if it's perimeter is too small
			{
				cvSubstituteContour( scanner, NULL );
			}
			else //Smooth it's edges if it's large enough
			{
				CvSeq* c_new;
				if(poly1_hull0) //Polygonal approximation of the segmentation
					c_new = cvApproxPoly(c,sizeof(CvContour),mem_storage,CV_POLY_APPROX_DP, CVCONTOUR_APPROX_LEVEL,0);
				else //Convex Hull of the segmentation
					c_new = cvConvexHull2(c,mem_storage,CV_CLOCKWISE,1);
				cvSubstituteContour( scanner, c_new );
				numCont++;
			}
		}
		contours = cvEndFindContours( &scanner );

		// PAINT THE FOUND REGIONS BACK INTO THE IMAGE
		cvZero( mask );
		IplImage *maskTemp;
		//CALC CENTER OF MASS AND OR BOUNDING RECTANGLES
		if(cont_num != NULL)
		{
			int N = *cont_num, numFilled = 0, i=0;
			CvMoments moments;
			double M00, M01, M10;
			maskTemp = cvCloneImage(mask);
			for(i=0, c=contours; c != NULL; c = c->h_next,i++ )
			{
				if(i < N) //Only process up to *num of them
				{
					cvDrawContours(maskTemp,c,CV_CVX_WHITE, CV_CVX_WHITE,-1,CV_FILLED,8);
					//Find the center of each contour
					cvMoments(maskTemp,&moments,1);
					M00 = cvGetSpatialMoment(&moments,0,0);
					M10 = cvGetSpatialMoment(&moments,1,0);
					M01 = cvGetSpatialMoment(&moments,0,1);
					cont_center.push_back(cvPoint((int)(M10/M00),(int)(M01/M00)));
					//centers[i].x = (int)(M10/M00);
					//centers[i].y = (int)(M01/M00);
					//Bounding rectangles around blobs
					cont_boundbox.push_back(cvBoundingRect(c));
					cont_boundbox2D.push_back(cvMinAreaRect2(c));
					cvZero(maskTemp);
					numFilled++;
				}
				//Draw filled contours into mask
				cvDrawContours(mask,c,CV_CVX_WHITE,CV_CVX_WHITE,-1,CV_FILLED,8); //draw to central mask
			} //end looping over contours
			*cont_num = numFilled;
			cvReleaseImage( &maskTemp);

		}
		//ELSE JUST DRAW PROCESSED CONTOURS INTO THE MASK
		else
		{
			for( c=contours; c != NULL; c = c->h_next )
			{
				cvDrawContours(mask,c,CV_CVX_WHITE, CV_CVX_BLACK,-1,CV_FILLED,8);
			}
		}
		cvReleaseMemStorage( &mem_storage );   
	}
}

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
HandRegion::HandRegion(void)

{
	mImage = 0;
}

HandRegion::~HandRegion(void)
{
    if ( mImage )
    {
        cvReleaseImage( &mImage );
    }
}

bool HandRegion::LoadSkinColorProbTable()
{
    if ( mSkinColor.LoadLookUpTable( "skin.dis" ) == false )
    {
        if ( mSkinColor.LoadFile( "skin.mgm" ) == false )
        {
            fprintf( stderr, "skin color distribution load error.\n" );
            return false;
        }
        printf("making a lookup table for skin color distribution ");
        mSkinColor.MakeLookUpTable();
        printf("done\n");
        if ( mSkinColor.SaveLookUpTable( "skin.dis" ) == false )
        {
            fprintf( stderr, "skin color distribution look up table save error.\n" );
            return false;
        }
    }
    if ( mNonSkinColor.LoadLookUpTable( "nonskin.dis" ) == false )
    {
        if ( mNonSkinColor.LoadFile( "nonskin.mgm" ) == false )
        {
            fprintf( stderr, "non-skin color distribution load error.\n" );
            return false;
        }
        printf("making a lookup table for non-skin color distribution ");
        mNonSkinColor.MakeLookUpTable();
        printf("done\n");
        if ( mNonSkinColor.SaveLookUpTable( "nonskin.dis" ) == false )
        {
            fprintf( stderr, "non-skin color distribution look up table save error.\n" );
            return false;
        }
    }

    return true;
}

IplImage * HandRegion::GetHandRegion( IplImage * srcImage, int *cont_num, std::vector <CvRect> & cont_boundbox,  std::vector <CvBox2D> & cont_boundbox2D, std::vector <CvPoint> & cont_center)
{
    //
    // Initialize Memory for Image
    //
    if ( !mImage )
    {
        mImage = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
        mImage->origin = srcImage->origin;
    }
	
    //
    // Segmentation by Color Distribution
    //
    for ( int i = 0 ; i < srcImage->height ; i ++ )
    {
        for ( int j = 0 ; j < srcImage->width ; j ++ )
        {
            unsigned char R = srcImage->imageData[ i * srcImage->widthStep + j*3 + 2];
            unsigned char G = srcImage->imageData[ i * srcImage->widthStep + j*3 + 1];
            unsigned char B = srcImage->imageData[ i * srcImage->widthStep + j*3 + 0];

            float P_Skin = mSkinColor.GetProbabilityByLookup( R, G, B );
            float P_NonSkin = mNonSkinColor.GetProbabilityByLookup( R, G, B );

            float P = P_Skin/P_NonSkin;
            if ( P < 0.4 ) 
			{
				CV_IMAGE_ELEM(mImage, char, i, j) = 0;		
				//mImage->imageData[ i * mImage->widthStep + j ] = 0;
            }
            else
            {
				CV_IMAGE_ELEM(mImage, char, i, j) = 1;		
                //mImage->imageData[ i * mImage->widthStep + j ] = 255;
			}
        }
    }
	// find connected components
	getConnectedComponents(mImage,1,4,cont_num,cont_boundbox, cont_boundbox2D,cont_center);//int *num, CvRect *bbs, CvPoint *centers
	
	return mImage;
}

#ifdef USE_SKIN_SEGMENTATION
int HandRegion::FindHands(IplImage *depthIm, IplImage *colourIm, IplImage *transDepth320, IplImage *transColor320)
{
	//<----(Hand Segmentation) Count up
	//TickCountAverageBegin();
	const CvSize SKINSIZE = cvSize(ARMM::ConstParams::SKIN_X, ARMM::ConstParams::SKIN_Y); 

	//Create image storages
	IplImage* depthTmp		  = cvCreateImage( SKINSIZE, IPL_DEPTH_8U, 1);
	IplImage* colourImResized = cvCreateImage( SKINSIZE, IPL_DEPTH_8U, 3);
	IplImage* depthImResized  = cvCreateImage( SKINSIZE, IPL_DEPTH_32F, 1);
	IplImage* colourImCapSize = cvCreateImage(CAPTURE_SIZE, IPL_DEPTH_8U, 3);
	cvResize(transDepth320, depthImResized);
	cvSmooth(depthImResized, depthImResized, CV_MEDIAN, 3);
	//REP(i,3){
	//	cvSmooth(depthImResized, depthImResized, CV_MEDIAN, 1);
	//}

	//----->Threshold the edge region from kinect
	const int edge_side = 50;
	for ( int i = 0 ; i < transDepth320->height; i ++ )
	{
		if( i>=0 && i<edge_side || i>=transDepth320->height-edge_side && i<transDepth320->height)
		{
			for ( int j = 0 ; j < transDepth320->width ; j ++ )
			{
				CV_IMAGE_ELEM(transDepth320, float, i, j) = 0;		
			}
		}
		else
		{
			for ( int j = 0 ; j < edge_side; j ++ )
			{
				CV_IMAGE_ELEM(transDepth320, float, i, j) = 0;		
			}
			for ( int j = transDepth320->width-edge_side; j < transDepth320->width; j ++ )
			{
				CV_IMAGE_ELEM(transDepth320, float, i, j) = 0;		
			}
		}
	}

	//----->Threshold too near region from Kinect
	for ( int i = 0 ; i < transDepth320->height ; i ++ )
	{
		for ( int j = 0 ; j < transDepth320->width ; j ++ )
		{
			float depth_ =  CV_IMAGE_ELEM(transDepth320, float, i, j);

			// dist from ground is larger than 30cm => removing the pixel
			if( depth_ > 30 || depth_ <= 1)
			{
				CV_IMAGE_ELEM(transDepth320, float, i, j) = 0;
			}
		}
	}

	//----->Threshold at marker's depth
	cvThreshold(transDepth320, depthTmp, 1, 255, CV_THRESH_BINARY_INV); //thres at 1cm above marker
	cvResize(transColor320, colourImResized);
	//if(SKINSIZE.width == CAPTURE_SIZE.width && SKINSIZE.height == CAPTURE_SIZE.height)
	//{
	//	cvSet(colourImCapSize, cvScalar(0), depthTmp);	//threshold to the colour image
	//}
	//else
	//{
		cvSet(colourImResized, cvScalar(0), depthTmp);	//threshold to the colour image
		cvResize(colourImResized, colourImCapSize, CV_INTER_NN);
	//}

	//----->Segment skin color
	int cont_num = MAX_NUM_HANDS;//up to MAX_NUM_HANDS contours
	cvCopyImage( GetHandRegion( colourImResized, &cont_num, cont_boundbox, cont_boundbox2D, cont_center), depthTmp);
	cvThreshold(depthTmp, depthTmp, 0, 255, CV_THRESH_BINARY_INV); //ネガポジ反転
	cvSet(colourImResized, cvScalar(0), depthTmp); //apply Hand mask image to colour image
	cvSet(depthImResized,  cvScalar(0), depthTmp); //apply Hand mask image to depth image
	cvResize(depthTmp, depthmask_for_mesh, CV_INTER_LINEAR);

  //----->Display image
	IplImage* col_640 = cvCreateImage(SKINSIZE, IPL_DEPTH_8U, 3);
	cvResize(colourImResized, col_640, CV_INTER_LINEAR);

	//----->Draw center of contour
	float *center_depth; 
	int numb_hands = cont_center.size();//tmp
	center_depth = new float[numb_hands];//tmp

	// correct skin image size to 160x120
	int skin_ratio = ARMM::ConstParams::SKIN_X / MESH_SIZE.width;
	assert( fabs( (ARMM::ConstParams::SKIN_Y / MESH_SIZE.height) - skin_ratio ) < 0.1); 

	// the corner of the bounding box
	CvPoint upperLeft = cvPoint(0,0);
	CvPoint bottomRight = cvPoint(0,0);

	REP(i,cont_center.size())
	{
		center_depth[i] = CV_IMAGE_ELEM(transDepth320, float, cont_center.at(i).y, cont_center.at(i).x);
		const int box_size = skin_ratio * FindNumHandPixels(center_depth[i]-ARMM::ConstParams::HAND_THICKNESS);
		const int offset = (box_size-1)/2;

		//  (x2,y2)-----(x1,y2)
		//	   |           | 
		//  (x2,y1)-----(x1,y1)
		int x1 = cont_center.at(i).x + offset;//lower right corner
		int y1 = cont_center.at(i).y + offset;
		int x2 = cont_center.at(i).x - offset;//upper left corner
		int y2 = cont_center.at(i).y - offset;					
		curr_hands_corners[i] = cvPoint(x2, ARMM::ConstParams::SKIN_Y-y1);
		bottomRight = cvPoint(x1,y1); 
		upperLeft   = cvPoint(x2,y2);

		//----->Sort all the points using nearest neighbor
	
		//temporary test 1 hand
		//----->Copy height info to grid and display
		//				for(int i = 0; i < num_hand_in_scene; i++) {
		IplImage* depth11	= cvCreateImage(cvSize(MIN_HAND_PIX, MIN_HAND_PIX), IPL_DEPTH_32F, 1);

		cvSetImageROI(depthImResized, cvRect(x2, y2 ,box_size,box_size));

		cvResize(depthImResized, depth11, CV_INTER_NN);
		//int resize_ratio_x = SKIN_SEGM_SIZE.width / MIN_HAND_PIX;
		//int resize_ratio_y = SKIN_SEGM_SIZE.height/ MIN_HAND_PIX;
		for(int j = 0; j < MIN_HAND_PIX; j++)
		{
			for(int k = 0; k < MIN_HAND_PIX; k++) 
			{
				int ind = j * MIN_HAND_PIX + k;
				//version 2013.1.28
				//hand_depth_grids[i][ind] = CV_IMAGE_ELEM(depth11, float, ((int)MIN_HAND_PIX-1)-k, j);
				//version 2013.1.29 considering the thin of a hand
				hand_depth_grids[i][ind] = CV_IMAGE_ELEM(depth11, float, ((int)MIN_HAND_PIX-1)-k, j)
											- ARMM::ConstParams::HAND_THICKNESS;
			}
		}
		cvReleaseImage(&depth11); 

		//----->Display
		//char center_depth_print[50];
		//sprintf(center_depth_print,"%.2lf cm", center_depth[i]);
		////for displaying distance from ground
		//CvFont font;//tmp
		//cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.3 * skin_ratio , 0.3 * skin_ratio , 0, 1);//tmp
		//cvPutText (colourIm160, center_depth_print,  cont_center.at(i), &font, cvScalar(0,255,0));

		//cvCircle(colourIm160, cont_center.at(i),2,cvScalar(255,0,0));
		//cvRectangle(colourIm160, cvPoint(x1,y1) ,cvPoint(x2,y2),cvScalar(0,0,255));
	    cvRectangle(transColor320, cvPoint(x1,y1) ,cvPoint(x2,y2),cvScalar(0,0,255));

		//HACK TODO you should change this ratio depended on hand depth
		curr_hands_ratio[0] = (float)FindNumHandPixels(0) / MIN_HAND_PIX;

		curr_hands_corners[i].x /= skin_ratio;
		curr_hands_corners[i].y /= skin_ratio;

		cvResetImageROI(transDepth320);
	}

	//for(int i = 0; i < num_hand_in_scene; i++) {
	//	cvCircle(col_640,cvPoint(curr_hands_corners[i].x, 160 - curr_hands_corners[i].y),5,cvScalar(0,255,255));
	//}
	//<-- (HAND SEGMENTATION) end
	//TickCountAverageEnd();

	//<---Finger detection
	//TickCountAverageBegin();
	vector< vector<cv::Point> > fingerTips;
	cv::Ptr<IplImage> grey_640 = cvCreateImage(cvGetSize(col_640), 8, 1);	
	cvCvtColor(col_640, grey_640, CV_BGR2GRAY);	
	cvThreshold(grey_640, grey_640, 1, 255, cv::THRESH_BINARY);
	DetectFingertips(grey_640, fingerTips);

	// init fingertips
	fingerIndex.clear();

	//draw fingertips
	const float diffX = abs(upperLeft.x - bottomRight.x)/static_cast<float>(MIN_HAND_PIX);
	const float diffY = abs(upperLeft.y - bottomRight.y)/static_cast<float>(MIN_HAND_PIX);

	REP(i,fingerTips.size())
	{
		REP(j,fingerTips[i].size())
		{
			//check pixels matching fingertips
			REP(dy,MIN_HAND_PIX) REP(dx,MIN_HAND_PIX)
			{
				CvPoint tmpFingertips;
				tmpFingertips.x = upperLeft.x+static_cast<int>(dx*diffX);
				tmpFingertips.y = upperLeft.y+static_cast<int>(dy*diffY);
				if(tmpFingertips.x < 0 
				|| tmpFingertips.x >= ARMM::ConstParams::SKIN_X
				|| tmpFingertips.y < 0
				|| tmpFingertips.y >= ARMM::ConstParams::SKIN_Y
				){
					continue;
				}

				if( abs(fingerTips[i][j].x-tmpFingertips.x) <= 2
				&&  abs(fingerTips[i][j].y-tmpFingertips.y) <= 2)
				{
					//OSGで関連付けているIndexの走査方向によってIndexは定まる
					fingerIndex.push_back( dx*MIN_HAND_PIX + (MIN_HAND_PIX-1)-dy);
				}
			}

#ifdef SHOWSEGMENTATION
			cvCircle(col_640, fingerTips[i][j] , 10, cv::Scalar(255,255,0), 4);
#endif
		}
	}
	//printf("Upper=(%d,%d) Bottom=(%d,%d)\n",upperLeft.x, upperLeft.y, bottomRight.x, bottomRight.y);
	//printf("DiffX=%f, DiffY=%f\n",diffX, diffY);
	//cout << "Finger=" << fingerIndex.size() << endl;
	//<--(FINGERTIPS DETECTION) end
	//TickCountAverageEnd();

#ifdef SHOWSEGMENTATION
	//for displaying FPS
	char fps_print[50];
	sprintf(fps_print,"%.2lf FPS", showFPS);
	CvPoint fpsPos =  cvPoint((int)(ARMM::ConstParams::SKIN_X*0.75), (int)(ARMM::ConstParams::SKIN_Y*0.9));
	CvFont font;//tmp
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.2 * skin_ratio , 0.2 * skin_ratio , 0, 1);//tmp
	cvPutText (col_640, fps_print, fpsPos, &font, cvScalar(0,255,255));

	cvShowImage("Op_Flow_640",col_640);
	//cvShowImage("transcolor",transColor320);
#endif

	int num_of_hands = static_cast<int>(cont_center.size());

	//memory release
	delete center_depth;

	cvReleaseImage(&col_640);
	cvReleaseImage(&depthTmp);
	cvReleaseImage(&colourImResized);
	cvReleaseImage(&depthImResized);
	cvReleaseImage(&colourImCapSize);
	cont_boundbox.clear();
	cont_boundbox2D.clear();
	cont_center.clear();

	return num_of_hands;
}
#endif /* USE_SKIN_SEGMENTATION */

void HandRegion::DetectFingertips(cv::Ptr<IplImage> handMask, 
	vector< vector<cv::Point> > & fingerTips)
{
	fingerTips.clear();
	vector< vector<cv::Point> > contours;

	//cv::findContours(src, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	CvMemStorage* storage   = cvCreateMemStorage( 0 );     
	CvSeq* cs = NULL;    
	CvScalar black          = CV_RGB( 0, 0, 0 ); 
	CvScalar white          = CV_RGB( 255, 255, 255 ); 
	int contours_count= cvFindContours( handMask, storage, &cs, sizeof( CvContour ), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );  

	int loop = 0;
	contours.resize(contours_count);
    for( ; cs!= 0; cs= cs->h_next, loop++)
    {
        REP(i,cs->total)
        {
			CvPoint tmpPoint = *CV_GET_SEQ_ELEM( CvPoint, cs, i );
			contours[loop].push_back(cv::Point(tmpPoint));
        }

		cv::Mat contourMat = cv::Mat(contours[loop]);
		double area = cv::contourArea(contourMat);

		if (area > 200)  { // possible hand
			vector<cv::Point> tmp_fingertips;
			tmp_fingertips.clear();

			cv::Scalar center = mean(contourMat);
			cv::Point centerPoint = cv::Point(static_cast<int>(center.val[0]), static_cast<int>(center.val[1]) );
			vector<cv::Point> approxCurve;
			CvSeq *approx = cvApproxPoly(cs, sizeof(CvContour), NULL, CV_POLY_APPROX_DP, 15);
			//cv::approxPolyDP(contourMat, approxCurve, 15, true); //error part

			REP(i,approx->total)
			{
				CvPoint tmpPoint = *CV_GET_SEQ_ELEM( CvPoint, approx, i );
				approxCurve.push_back(cv::Point(tmpPoint));
			}

			vector<int> hull;
			vector<cv::Point> hullShape;
			//cv::convexHull(cv::Mat(approxCurve), hull);
			CvSeq* h = cvConvexHull2(approx,0,CV_CLOCKWISE,0);
			//cout << "hogehoge4" << endl;


			REP(i,h->total)
			{
				//CvPoint tmpPoint = *CV_GET_SEQ_ELEM( CvPoint, h, i ); //this code occurs assertion
				CvPoint tmpPoint = **CV_GET_SEQ_ELEM( CvPoint*, h, i ); //this one is okay?
				hullShape.push_back(cv::Point(tmpPoint));
			}

			int approxPointer = 0;
			REP(i,hullShape.size())
			{
				for(;approxPointer<static_cast<int>(approxCurve.size());approxPointer++)
				{
					if(hullShape[i] == approxCurve[approxPointer]){
						hull.push_back(approxPointer);
						break;
					}
				}
			}
			//cout << "Hull size = " << hull.size() << endl;
			// find upper and lower bounds of the hand and define cutoff threshold (don't consider lower vertices as fingers)
			int upper = 600, lower = 0;
			for (unsigned int j=0; j<hull.size(); j++) {
				int idx = hull[j]; // corner index
				if (approxCurve[idx].y < upper) upper = approxCurve[idx].y;
				if (approxCurve[idx].y > lower) lower = approxCurve[idx].y;
			}

			float cutoff = lower - (lower - upper) * 0.1f;
			// find interior angles of hull corners
			for (unsigned int j=0; j<hull.size(); j++) {
				int idx = hull[j]; // corner index
				int pdx = idx == 0 ? approxCurve.size() - 1 : idx - 1; //  predecessor of idx
				int sdx = idx == approxCurve.size() - 1 ? 0 : idx + 1; // successor of idx
				cv::Point v1 = approxCurve[sdx] - approxCurve[idx];
				cv::Point v2 = approxCurve[pdx] - approxCurve[idx];
				double angle = acos( (v1.x*v2.x + v1.y*v2.y) / (cv::norm(v1) * cv::norm(v2)) );
				// low interior angle + within upper 90% of region -> we got a finger
				if ( angle < 1 && approxCurve[idx].y < cutoff) {
					tmp_fingertips.push_back(cv::Point( approxCurve[idx].x , approxCurve[idx].y) );
				}
			}
			fingerTips.push_back(tmp_fingertips);
		}
	}
}

int HandRegion::FindNumHandPixels(float depth) 
{
	float box_pix = (float)(ARMM::ConstParams::HAND_BOX_CM/WORLD_SCALE+depth*ARMM::ConstParams::KINECT_PIX_PER_DEPTH);
	int ans;
	if(((int) ceil(box_pix)%2) == 0)
		ans = static_cast<int>(floor(box_pix));
	else
		ans = static_cast<int>(ceil(box_pix));
	//printf("pixel width = %d \n", ans);
	return ans;
}
