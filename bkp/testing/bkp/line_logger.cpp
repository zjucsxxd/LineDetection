#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <string.h>
#include<cstdlib>
#include<fstream>
//for opencv 3 #include<opencv/highgui.h>
#include<highgui.h>
// for opencv 3 #include<opencv2/opencv.hpp>
#include<cv.h>
#include<iostream>
#include<vector>
#define odroid 1
#include<stdio.h>
#include<stdlib.h>
using namespace cv;
using namespace std;

int hmin_value=40,smin_value=11,vmin_value=100,hmax_value=150,smax_value=255,vmax_value=255;

int canny_thres=50;
bool save_image=false;

Mat img,path,hsv,thres,edge,org_img;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

RNG rng(12345);
FILE *fp,*fp1;


void callback(int )
{
	inRange(hsv,Scalar(hmin_value,smin_value,vmin_value),Scalar(hmax_value,smax_value,vmax_value),thres);

	erode(thres, thres, Mat(), Point(-1, -1), 1, 1, 1);	     // opening  
	dilate(thres, thres, Mat(), Point(-1, -1), 1, 1, 1);        
	for(int k=1;k<=2;k++)
	{ 
		dilate(thres, thres, Mat(), Point(-1, -1), 1, 1, 1);             // closing 
		erode(thres, thres, Mat(), Point(-1, -1), 1, 1, 1);
	}	
	Canny(thres,edge,canny_thres,canny_thres*2,3,true);                                    // canny edge detection

}

void contrast_streching(Mat img)
{
	Mat ch[3];
	split(img,ch);
	normalize(ch[1],ch[1],255,0,NORM_MINMAX);
	normalize(ch[2],ch[2],255,0,NORM_MINMAX);
	normalize(ch[0],ch[0],255,0,NORM_MINMAX);
	merge(ch,3,img);
}

void smoothing(Mat img)
{
	Mat img_new;
	img.convertTo(img_new,CV_8U);
	bilateralFilter(img_new,img,5,150,10);
}


float rho(int x1,int y1,int x2,int y2)
{           //returns the perpendicular distance of line
	float m=(y2-y1)/(float)(x2-x1);
	float numer=y2-m*x2;
	float den=sqrt((m*m)+1);
	return (numer/den);
}

int main(int argc,char **argv)
{
	//sleep(5);
	FILE *fp2;	
	if(!odroid){
		 fp=fopen("ipdata.txt","a");}
		fp2=fopen("login.txt","a");
	
	//VideoCapture inputVideo("Pipe.avi");
	VideoCapture inputVideo(1);
	
//CvSize size = cvSize((int)cvGetCaptureProperty(inputVideo,CV_CAP_PROP_FRAME_WIDTH),
//		        (int)cvGetCaptureProperty(inputVideo,CV_CAP_PROP_FRAME_HEIGHT));  
    CvSize size=cvSize(640,480);

	int frames=1;
	// VideoWriter vid_rgb("rgb.avi", CV_FOURCC('X','V','I','D'), frames, size);
	    

	while(1)
	{ 
		inputVideo.read(org_img);

		if( !inputVideo.isOpened() )
			break;
		img=org_img.clone();
		smoothing(img);                            //  smoothing of rgb image.
		

		contrast_streching(img);
		cvtColor(img,hsv,CV_RGB2HSV);
		/*erode(hsv, hsv, Mat(), Point(-1, -1), 1, 1, 1);*/
		smoothing(hsv);
		     
		//contrast_streching(hsv);
		
		callback(0);
		
			
		
		dilate(edge, edge, Mat(), Point(-1, -1), 1, 1, 1);                   // dilation

		cvtColor(edge, path, CV_GRAY2BGR);

		vector<Vec4i> lines;
		 
		 
		HoughLinesP(edge, lines, 1, CV_PI/180, 50, 100, 6.1);                // line detection
		  //lines is a 2d mat with row=no. of lines detected and columns=4.
		 
		  float angle[lines.size()];
		  float p[lines.size()];
		  int num=lines.size();
		  float set1[lines.size()];
		  float set2[lines.size()];
		  if(!odroid) fprintf(fp,"%d ",num);
		  int count5 =1;int flag =0;int count6 = 1;int count1,count2;count1=count2=1;float angle_f1,angle_f2;angle_f1=angle_f2 =0; float rho_1,rho_2;rho_1= rho_2 = 0;
		  for( size_t i = 0; i < lines.size(); i++ )
		  {
			Vec4i l = lines[i];
			line( path, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 1, CV_AA);
			line( img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 1, CV_AA);
			angle[i]=(l[3]-l[1])/(float)(l[2]-l[0]);
			if((57.2957795*tanh(angle[i]))>0)
				angle[i]=(90-(57.2957795*tanh(angle[i])));
				
			else		
				angle[i]=(57.2957795*tanh(angle[i]));
				p[i]=rho(l[0],l[1],l[2],l[3]);
		
			if (i==0) set1[0] = 0;
			if(i!= 0) {                                                                             // to sort the lines into two sets
				     if (abs (angle[i]-angle[0]) < 10 ) {set1[count5] = i;count5++;}
				     if ((abs (angle[i]-angle[0]) > 10) && (flag != 0)) {set2[count6] = i;count6++;}
				     if ((abs (angle[i]-angle[0]) > 10)  && (flag == 0)) {set2[0] = i;flag =98;}
	      	}
	        
	        if(!odroid) fprintf(fp," %.2f- %f ",angle[i],p[i]);  
	   
	      }
		  for( size_t i = 0; i < lines.size(); i++ ) 
		  {
		  	if (i== 0)  	angle_f1 += angle[(int)(set1[0])];
		  	if (i!= 0){ 
				if( 1.0<(int)(set1[i])<10.0) 
				 	{
						angle_f1 += angle[(int)(set1[i])];
		            	count1 +=1;
		            }
		   }  
		  
	       }  
	     for( size_t i = 0; i < lines.size(); i++ ) 
		 {
			if (i== 0) angle_f2 += angle[(int)(set2[0])];
			if (i!= 0)  
			if( 0.0<(int)(set2[i])<10.0) 
			{
				angle_f2 += angle[(int)(set2[i])];
				count2 +=1;  
			}
		 }  
	/*
	 for( size_t i = 0; i < lines.size(); i++ ) 
		{
		if (i== 0) rho_f1 += p[(int)(se12[0])];
	.	if (i!= 0)  rho( 1.0<(int)(set2[i])<10.0) rho_f1 += p[(int)(set1[i])];count3 +=1;  // if rho are to be averaged
	     rho_f1 = rho_f1/count3;  
		}  
	 */ 
	angle_f1 =  angle_f1/count1;
	angle_f2 =  angle_f2/count2;
	rho_1 = p[(int)set1[0]];
	rho_2 = p[(int)set2[0]];

	if(!odroid) {
		fprintf(fp,"Set 2 : %f,%f ",angle_f1,angle[(int)set1[2]]); 
		fprintf(fp,"\n \n"); 
	}
	
	if(!odroid) imshow("IMAGE1",img);  
	//RHO_1 AND RHO_2 ARE FINAL RHO OF BOTH SETS AND ANGLE_F1 AND ANGLE_F2 ARE THETA OF SETS
	//contour detection and drawing
	  findContours( edge, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	  Mat drawing = Mat::zeros( edge.size(), CV_8UC3 );
	  for( int i = 0; i< contours.size(); i++ )
		 {
		   Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		   drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
		 }
    if (angle_f1>1) {cout << "l" << endl;
    system("./mcntrl l"); fprintf(fp2,"%c",'l');}
    else if (angle_f1<-1) {cout << "r" << endl;
    system("./mcntrl r");fprintf(fp2,"%c",'r');}
    //if (rho_1 < -2) { cout << "b" << endl;
    //system("./mcntrl b");}
    else if (rho_1 > 2) {cout << "f" << endl;
    system("./mcntrl f");fprintf(fp2,"%c",'f');}	
	//vid_rgb << org_img;
 
    
 
	char c=cvWaitKey(20);
	if(c ==27)
	break;
}
		//saving the final parameters
		fp1=fopen("parameters_final","a");
	fprintf(fp1,"%d %d %d %d %d %d \n",hmin_value,smin_value,vmin_value,hmax_value,smax_value,vmax_value);
	system("./mcntrl stop");		 
}
