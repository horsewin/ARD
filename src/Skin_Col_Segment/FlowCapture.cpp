//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <iostream>
#include <math.h>

#include "FlowCapture.h"
#include "VarFlow.h"  

//---------------------------------------------------------------------------
// Constant
//---------------------------------------------------------------------------
const float M_PI = 3.141592654;
const char * window_name = "Variational Optical Flow";

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
static double _PrevTickCount = 0;

int width = 640;
int height = 480;
int img_width	= 320;
int img_height	= 240;


namespace{
	void TickCountBegin()
	{
		_PrevTickCount = static_cast<double>(cv::getTickCount());
	}

	double TickCountEnd()
	{
		double currTickCount = static_cast<double>(cv::getTickCount());
		double timeSpent = ( currTickCount - _PrevTickCount ) / cv::getTickFrequency();
		_PrevTickCount = currTickCount;
		return( 1/timeSpent );
	}

	// Calculates RGB values from HSV color space
	void hsv2rgb(float h, float s, float v, uchar &r, uchar &g, uchar &b)
	{     
		if(h > 360){
			h = h - 360;
		}
  
		float c = v*s;   // chroma
		float hp = h / 60;
  
		float hpmod2 = hp - (float)((int)(hp/2))*2;
     
		float x = c*(1 - fabs(hpmod2 - 1));
		float m = v - c;
     
		float r1, g1, b1;
  
		if(0 <= hp && hp < 1){
			r1 = c;
			g1 = x;
			b1 = 0;
		}
		else if(1 <= hp && hp < 2){
			r1 = x;
			g1 = c;
			b1 = 0;
		}
		else if(2 <= hp && hp < 3){
			r1 = 0;
			g1 = c;
			b1 = x;
		}
		else if(3 <= hp && hp < 4){
			r1 = 0;
			g1 = x;
			b1 = c;
		}
		else if(4 <= hp && hp < 5){
			r1 = x;
			g1 = 0;
			b1 = c;
		}
		else{
			r1 = c;
			g1 = 0;
			b1 = x;
		}
  
		r = (uchar)(255*(r1 + m));
		g = (uchar)(255*(g1 + m));
		b = (uchar)(255*(b1 + m));   
	}

	// Draw a vector field based on horizontal and vertical flow fields
	void drawMotionField(IplImage* imgU, IplImage* imgV, IplImage* imgMotion, int xSpace, int ySpace, float cutoff, int multiplier, CvScalar color)
	{
		int x, y;
  
		CvPoint p0 = cvPoint(0,0);
		CvPoint p1 = cvPoint(0,0);
  
		float deltaX, deltaY, angle, hyp;
  
		for(y = ySpace; y < imgU->height; y+= ySpace ) {
			for(x = xSpace; x < imgU->width; x+= xSpace ){
				p0.x = x;
				p0.y = y;
      
				deltaX = *((float*)(imgU->imageData + y*imgU->widthStep)+x);
				deltaY = -(*((float*)(imgV->imageData + y*imgV->widthStep)+x));
      
				angle = atan2(deltaY, deltaX);
				hyp = sqrt(deltaX*deltaX + deltaY*deltaY);
      
				if(hyp > cutoff){
	
		p1.x = p0.x + cvRound(multiplier*hyp*cos(angle));
		p1.y = p0.y + cvRound(multiplier*hyp*sin(angle));
                   
		cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);
        
		p0.x = p1.x + cvRound(3*cos(angle-M_PI + M_PI/4));
		p0.y = p1.y + cvRound(3*sin(angle-M_PI + M_PI/4));
		cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);
        
		p0.x = p1.x + cvRound(3*cos(angle-M_PI - M_PI/4));
		p0.y = p1.y + cvRound(3*sin(angle-M_PI - M_PI/4));
		cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);
				}    
			}
		}
	}

	// Draws the circular legend for the color field, indicating direction and magnitude
	void drawLegendHSV(IplImage* imgColor, int radius, int cx, int cy)
	{
		int width = radius*2 + 1;
		int height = width;
     
		IplImage* imgLegend = cvCreateImage( cvSize(width, height), 8, 3 );
		IplImage* imgMask = cvCreateImage( cvSize(width, height), 8, 1 );
		IplImage* sub_img = cvCreateImageHeader(cvSize(width, height),8,3);
  
		uchar* legend_ptr;
		float angle, h, s, v, legend_max_s;
		uchar r,g,b;
		int deltaX, deltaY;
  
		legend_max_s = static_cast<float>(radius*sqrt(2.0));
  
		for(int y=0; y < imgLegend->height; y++){
			legend_ptr = (uchar*)(imgLegend->imageData + y*imgLegend->widthStep);         
			for(int x=0; x < imgLegend->width; x++){
				deltaX = x-radius;
				deltaY = -(y-radius);
				angle = atan2(static_cast<float>(deltaY),static_cast<float>(deltaX) );
      
				if(angle < 0){
					angle += static_cast<float>(2*M_PI);
				}      
				h = angle * static_cast<float>(180 / M_PI);
				s = sqrt(static_cast<float>(deltaX*deltaX + deltaY*deltaY) ) / legend_max_s;
				v = (float)0.9;
      
				hsv2rgb(h, s, v, r, g, b);
               
				legend_ptr[3*x] = b;
				legend_ptr[3*x+1] = g;
				legend_ptr[3*x+2] = r;
      
			}
		}
     
		cvZero(imgMask);
		cvCircle( imgMask, cvPoint(radius,radius) , radius, CV_RGB(255,255,255), -1,8,0 );
     
		sub_img->origin = imgColor->origin;
		sub_img->widthStep = imgColor->widthStep;
		sub_img->imageData = imgColor->imageData + (cy-radius) * imgColor->widthStep + (cx-radius) * imgColor->nChannels;
  
		cvCopy(imgLegend, sub_img, imgMask);
     
		cvCircle( imgColor, cvPoint(cx,cy) , radius, CV_RGB(0,0,0), 1,CV_AA,0 );
     
		cvReleaseImage(&imgLegend);
		cvReleaseImage(&imgMask);
		cvReleaseImageHeader(&sub_img);   
	}

	// Draws a color field representation of the flow field
	void drawColorField(IplImage* imgU, IplImage* imgV, IplImage* imgColor)  
	{
		IplImage* imgColorHSV = cvCreateImage( cvSize(imgColor->width, imgColor->height), IPL_DEPTH_32F, 3 );
		cvZero(imgColorHSV);
  
		float max_s = 0;
		float *hsv_ptr, *u_ptr, *v_ptr;
		uchar *color_ptr;
		float angle;
		float h,s,v;
		uchar r,g,b;
		float deltaX, deltaY;
  
		int x, y;
  
		// Generate hsv image
		for(y = 0; y < imgColorHSV->height; y++ ) {    
			hsv_ptr = (float*)(imgColorHSV->imageData + y*imgColorHSV->widthStep);
			u_ptr = (float*)(imgU->imageData + y*imgU->widthStep);
			v_ptr = (float*)(imgV->imageData + y*imgV->widthStep);
		
			for(x = 0; x < imgColorHSV->width; x++){
				deltaX = u_ptr[x];
				deltaY = v_ptr[x];
      
				angle = atan2(deltaY,deltaX);
      
				if(angle < 0){
					angle += (float)2*M_PI;
				}                   
				hsv_ptr[3*x] = angle * 180 / M_PI;
				hsv_ptr[3*x+1] = sqrt(deltaX*deltaX + deltaY*deltaY);
				hsv_ptr[3*x+2] = (float)0.9;	
      
				if(hsv_ptr[3*x+1] > max_s){
					max_s = hsv_ptr[3*x+1];
				}      
			}
		}
  
		// Generate color image
		for(y = 0; y < imgColor->height; y++ ) {
			hsv_ptr = (float*)(imgColorHSV->imageData + y*imgColorHSV->widthStep);
			color_ptr = (uchar*)(imgColor->imageData + y*imgColor->widthStep);
    
			for(x = 0; x < imgColor->width; x++){
				h = hsv_ptr[3*x];
				s = hsv_ptr[3*x+1] / max_s;
				v = hsv_ptr[3*x+2];
      
				hsv2rgb(h, s, v, r, g, b);
      
				color_ptr[3*x] = b;
				color_ptr[3*x+1] = g;
				color_ptr[3*x+2] = r;    
			}
		}    
		drawLegendHSV(imgColor, 25, 28, 28);    
		cvReleaseImage(&imgColorHSV);   
	}
}

using namespace std;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
FlowCapture::FlowCapture()
	:max_level(4), start_level(0), n1(2), n2(2), rho(2.8), alpha(1400), sigma(1.5)
{
	imgA = cvCreateImage(cvSize(img_width,img_height),IPL_DEPTH_8U,1);
  imgB = cvCreateImage(cvSize(img_width,img_height),IPL_DEPTH_8U,1);
  
  imgU = cvCreateImage(cvSize(img_width,img_height), IPL_DEPTH_32F, 1);
  imgV = cvCreateImage(cvSize(img_width,img_height), IPL_DEPTH_32F, 1);
  
	imgColor = cvCreateImage( cvSize(width,height), 8, 3 );
  imgMotion = cvCreateImage(cvSize(img_width,img_height), 8, 3);

	optical_flow = new VarFlow(img_width, img_height, max_level, start_level, n1, n2, rho, alpha, sigma);

	cvNamedWindow(window_name,CV_WINDOW_AUTOSIZE);
}

FlowCapture::~FlowCapture(){
	delete optical_flow;

	cvDestroyWindow(window_name);
  
  cvReleaseImage(&imgA);
  cvReleaseImage(&imgB);
  cvReleaseImage(&imgU);
  cvReleaseImage(&imgV);
  cvReleaseImage(&imgColor);
  cvReleaseImage(&imgMotion);
}

void FlowCapture::Init()
{  
  cvZero(imgA);
  cvZero(imgB);

	cvZero(imgU);
  cvZero(imgV);
  cvZero(imgMotion);
  cvZero(imgColor);
  
  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1.0, 1.0, 0, 1);
}

void FlowCapture::Run(IplImage* prev_img, IplImage* curr_img, IplImage* col_img){
	//Copy color image
//	memcpy(imgColor->imageData, imd.RGB24Data(), imgColor->imageSize);
	//cvResize(imgColor, imgMotion, 1);
	//cvCvtColor(imgMotion, imgMotion, CV_RGB2BGR);

	//// Add Finger detection part
	//g_fingertip_pose_estimation.OnCapture( imgMotion, cvGetTickCount() );
	//g_fingertip_pose_estimation.OnProcess();

	////imgB = g_fingertip_pose_estimation.QueryHandImage();
	//imgB = (IplImage *)cvClone(g_fingertip_pose_estimation.QueryHandImage());
	////cvCvtColor(src_img, imgB, CV_RGB2GRAY);

	// Calculate the flow
	optical_flow->CalcFlow(prev_img, curr_img, imgU, imgV, 0);
        
	// Draw motion field with grid spacing of 10, minimium displacement 1 pixel, arrow length multiplier of 5         
	drawMotionField(imgU, imgV, col_img, 5, 5, 2, 2, CV_RGB(255,0,0));
	//    drawColorField(imgU, imgV, imgColor);                 
    
	//displaying fps
	//char fps[50];
	//sprintf(fps,"%.2lf fps", TickCountEnd());
	//cvPutText (col_img, fps, cvPoint(50,40), &font, cvScalar(0,255,0));

//	cvShowImage(window_name, curr_img);
	//imgA = (IplImage *) cvClone (imgB);
	//char c = cvWaitKey(100);
	//if( c == 27 ) exit(1);
}