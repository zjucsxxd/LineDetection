#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <string.h>
#include<cstdlib>
#include<fstream>
#include<ctime>
#include<opencv/highgui.h>				//for opencv 3.0.0
#include<opencv2/opencv.hpp>			//for opencv 3.0.0
//#include<highgui.h>					//for opencv 2.4.9
//#include<cv.h>						//for opencv 2.4.9
#include<iostream>
#include<vector>
using namespace cv;
using namespace std;

//the code is shifted a tab to the right because I intend to write the whole code in a class

		int x=0;//for pause and play of video , 1 for run , 0 for pause
		int hmin_value=94,smin_value=94,vmin_value=145,hmax_value=155,smax_value=255,vmax_value=255;
		int canny_thres=50;
		bool save_image=false;
		Mat img,path,hsv,thres,edge,org_img;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		RNG rng(12345);
		FILE *logoutput, *ipdata;
		char c;
		String s;//for input from user
		
		double INF=3e38;
		long int DELAY=50000000;
		bool ifConstantSet=false;
		double kp, ki, kd;
		clock_t lastTime;
		double ITerm,  lastInput, tolerance;
		double Input, Output;
		double outMin=-INF, outMax=INF;
		double Setpoint;
		VideoCapture inputVideo;
		
		CvSize size=cvSize(640,480);
		int frames=1;
		VideoWriter vid_lines("test.avi", CV_FOURCC('X','V','I','D'), frames, size);
		int frame_c=1;

/*
 ***************************************************IMAGE PROCESSING STARTS******************************************************
*/

	void callback(int )
	{
		inRange(hsv,Scalar(hmin_value,smin_value,vmin_value),Scalar(hmax_value,smax_value,vmax_value),thres);
		erode(thres, thres, Mat(), Point(-1, -1), 1, 1, 1);//opening  
		dilate(thres, thres, Mat(), Point(-1, -1), 1, 1, 1);        
		for(int k=1;k<=2;k++)
		{ 
			dilate(thres, thres, Mat(), Point(-1, -1), 1, 1, 1);//closing 
			erode(thres, thres, Mat(), Point(-1, -1), 1, 1, 1);
		}
		Canny(thres,edge,canny_thres,canny_thres*2,3,true);//canny edge detection
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
	void imageInput()
	{
		inputVideo.read(org_img);
		if(org_img.empty())
		{
			cout<<"error reading video"<<endl;
			exit(0);
		}
		img=org_img.clone();
		smoothing(img);
		contrast_streching(img);
		cvtColor(img,hsv,CV_RGB2HSV);
		smoothing(hsv);
		imshow("HSV",hsv);
	}

/*
 ****************************************************IMAGE PROCESSING ENDS*******************************************************
 *****************************************************GUI FUNCTIONS START********************************************************
*/

	void tracks()
	{
		cvNamedWindow("settings",CV_WINDOW_AUTOSIZE);
		cvCreateTrackbar("HMIN","settings",&hmin_value,255,callback);
		cvCreateTrackbar("HMAX","settings",&hmax_value,255,callback);
		cvCreateTrackbar("SMIN","settings",&smin_value,255,callback);
		cvCreateTrackbar("SMAX","settings",&smax_value,255,callback);
		cvCreateTrackbar("VMIN","settings",&vmin_value,255,callback);
		cvCreateTrackbar("VMAX","settings",&vmax_value,255,callback);
		cvCreateTrackbar("CANNY_THRESHOLD","settings",&canny_thres,127,callback);
	}
	static void onMouse( int event, int x, int y, int f, void* )
	{
		//for printing hsv values of a point where we double click
		Vec3b pix=hsv.at<Vec3b>(y,x);
		int H=pix.val[0];
		int S=pix.val[1];
		int V=pix.val[2];
		if(event==7)
		{
			cout<<"\nhue\t"<<H<<"\nsatuation\t"<<S<<"\nvalue\t"<<V<<"\n"<<endl;
			cvWaitKey(0);
		}
	}
	void initializeGUI()
	{
		namedWindow("test",CV_WINDOW_AUTOSIZE);
		namedWindow("IMAGE1",CV_WINDOW_AUTOSIZE);
		namedWindow("HSV",CV_WINDOW_AUTOSIZE);
		setMouseCallback("HSV",onMouse,0);
		tracks();
	}

/*
 ***********************************************************GUI ENDS**************************************************************
 ***************************************CALCULATIONS WITH THE EXTRACTED DATA BEGINS***********************************************
*/

	float rho(int x1,int y1,int x2,int y2)
	{       
		//returns the perpendicular distance of line
		float m=(y2-y1)/(float)(x2-x1);
		float number=y2-m*x2;
		float den=sqrt((m*m)+1);
		return (number/den);
	}

/*
 *******************************************************CALCULATIONS END**********************************************************
 *********************************************************FILE HANDLING***********************************************************
*/

	void readParameter()
	{
		//reading values from file
		FILE *fp=fopen("parameter.txt","r");
		fscanf(fp,"%d %d %d %d %d %d",&hmin_value,&smin_value,&vmin_value,&hmax_value,&smax_value,&vmax_value);
		fclose(fp);//closing parameters_final 
	}
	void writeParameter()
	{
		//saving the final parameters and closing the files
		FILE *fp=fopen("parameter.txt","w");
		fprintf(fp,"%d %d %d %d %d %d",hmin_value,smin_value,vmin_value,hmax_value,smax_value,vmax_value);
		fclose(fp);
	}
	void openFiles()
	{
		ipdata=fopen("ipdata.txt","w");
		logoutput=fopen("log.txt","w");
	}
	void closeFiles()
	{
		fclose(ipdata);
		fclose(logoutput);
	}
/*
 ******************************************************FILE HANDLING DONE*********************************************************
 **********************************************INPUT AND OUTPUT FUNCTION FOR PID**************************************************
*/

	void outputfunc(double finalOutput)
	{
		cout<<"quack quack";
	}
	double inputfunc()
	{
		callback(0);
		dilate(edge, edge, Mat(), Point(-1, -1), 1, 1, 1);//dilation
		cvtColor(edge, path, CV_GRAY2BGR);
		vector<Vec4i> lines;
		HoughLinesP(edge, lines, 1, CV_PI/180, 50, 100, 4.1);//line detection
		//lines is a 2d mat with row=no. of lines detected and columns=4.
		int num=lines.size();
		float angle[num];
		float p[num];//perpendicular distance  
		float set1[num];
		float set2[num];
		fprintf(ipdata,"%d ",num);
		int count5 =1;int flag =0;int count6 = 1;
		int count1,count2;count1=count2=1;
		float angle_f1,angle_f2;angle_f1=angle_f2 =0;
		float rho_1,rho_2;rho_1= rho_2 = 0;
		for(size_t i=0;i<num;i++)
		{
			Vec4i l = lines[i];
			line(path,Point(l[0],l[1]),Point(l[2],l[3]),Scalar(255,0,0),1,CV_AA);
			line(img,Point(l[0],l[1]),Point(l[2],l[3]),Scalar(255,0,0),1,CV_AA);
			angle[i]=(l[3]-l[1])/(float)(l[2]-l[0]);
			if((57.2957795*tanh(angle[i]))>0)
				angle[i]=(90-(57.2957795*tanh(angle[i])));		
			else
				angle[i]=(57.2957795*tanh(angle[i]));
			p[i]=rho(l[0],l[1],l[2],l[3]);
			if (i==0)
				set1[0]=0;
			if(i!= 0)
			{       //to sort the lines into two sets
				if(abs (angle[i]-angle[0]) < 10 )
				{
					set1[count5] = i;
					count5++;
				}
				if((abs (angle[i]-angle[0]) > 10)&&(flag != 0))
				{
					set2[count6] = i;
					count6++;
				}
				if((abs (angle[i]-angle[0]) > 10)&&(flag == 0))
				{
					set2[0] = i;
					flag =98;
				}
			}
			fprintf(ipdata," %.2f- %f ",angle[i],p[i]);
		}
		for( size_t i = 0; i < lines.size(); i++ ) 
		{
			if (i== 0)
				angle_f1 += angle[(int)(set1[0])];
			if (i!= 0)
			{
				if( 1.0<(int)(set1[i])<10.0) 
				{
					angle_f1 += angle[(int)(set1[i])];
					count1 +=1;
				}
			}
		}
		for( size_t i = 0; i < lines.size(); i++ ) 
		{
			if (i==0)
				angle_f2 += angle[(int)(set2[0])];
			if (i!=0)
				if(0.0<(int)(set2[i])<10.0) 
				{
					angle_f2 += angle[(int)(set2[i])];
					count2 +=1;  
				}
		}
		angle_f1=angle_f1/count1;
		angle_f2=angle_f2/count2;
		rho_1=p[(int)set1[0]];
		rho_2=p[(int)set2[0]];
		fprintf(ipdata,"Set 2 : %f,%f ",angle_f1,angle[(int)set1[2]]); 
		fprintf(ipdata,"\n \n");
		imshow("IMAGE1",img);
		//RHO_1 AND RHO_2 ARE FINAL RHO OF BOTH SETS AND ANGLE_F1 AND ANGLE_F2 ARE THETA OF SETS
		//contour detection and drawing
		findContours(edge,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
		Mat drawing=Mat::zeros(edge.size(),CV_8UC3);
		for(int i=0;i<contours.size();i++)
		{
			Scalar color=Scalar(rng.uniform(0, 255),rng.uniform(0,255),rng.uniform(0,255));
			drawContours(drawing,contours,i,color,2,8,hierarchy,0,Point());
		}
		if(num>1)
		{
			vid_lines<<img;
			frame_c++;
			imshow("test",img);
			cout<<frame_c;
			cout<<" "<<num;
		}
		if(angle_f1>1)
		{
			cout << "r" << endl;
			fprintf(logoutput,"%c\n",'r');
			return angle_f1;
		}
		if(angle_f1<-1)
		{
			cout<<"l"<<endl;
			fprintf(logoutput,"%c\n",'l');
			return angle_f1;
		}
	}
	void setConstants(double Kp, double Ki, double Kd, double tol)
	{
		kp = Kp;
		ki = Ki;
		kd = Kd;
		tolerance = tol;
		ifConstantSet = true;
	}
	void initializePID()
	{
		lastInput=inputfunc();
		ITerm = 0;
		lastTime = clock();
	}
	void outputLimits(double Min, double Max)
	{
		if(Min > Max)
		{
			cout << "minimum cannot be greater than maximum!" <<endl;
			return;
		}
		outMin = Min;
		outMax = Max;
	}
	void execute(double setPt)
	{
		long int i;// this is for the delay loop , hoping this will be removed before mid semester break
		readParameter();
		if (s.length()==1)
			//check if the string is just the number 1 or 0 , then convert it to integer
			VideoCapture inputVideo(0);
		else
			VideoCapture inputVideo(s);
		if(!ifConstantSet)
		{
			cout<<"Parameters not set!"<<endl;
			return;
		}
		Setpoint = setPt;
		initializePID();
		initializeGUI();
		while(1)
		{
			for(i=0; i<DELAY; i++);			//replace this delay with sleep() function
			;
			clock_t now = clock();
			double timeChange = (double)(now - lastTime)/(CLOCKS_PER_SEC);
			if(x==0)
				imageInput();
			else ;//notice the semicolon here, it is for not reading the next image if x=1 i.e. video is paused
			Input = inputfunc();
			double error = Setpoint - Input;
			if ((error < tolerance) && (error > -tolerance))
				return;
			ITerm += ki*error*timeChange;
			if(ITerm > outMax) ITerm = outMax;
			else if(ITerm < outMin) ITerm = outMin;
			double dInput = (Input - lastInput)/timeChange;
			Output = kp*error + ITerm - kd*dInput;
			if(Output > outMax) Output = outMax;
			else if(Output < outMin) Output = outMin;
			outputfunc(Output);
			lastInput = Input;
			lastTime = now;
			c=cvWaitKey(100);
			if(c==27)
				break;
			else if(c==' ')
				x=1;
			else if(c=='r')
				x=0;
		}
	}

/*
 ****************************************************PID FUNCTIONS END HERE*******************************************************
*/

	int main(int argc,char **argv)
	{
		if(argc<2)
		{
			cout<<"usage: ./linetuning xyz.avi"<<endl;
			return 0;
		}
		double kp=100, ki=100, kd=100, tolerance=100, min=100, max=100,finalstate=9990;
		s=strdup(argv[1]);//for input
		openFiles();
		setConstants(kp, ki, kd, tolerance);
		outputLimits(min, max);
		execute(finalstate);//main function in which the loop runs
		closeFiles();
	}