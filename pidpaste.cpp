#include<ctime>
#include<iostream>

	double INF;
	long int DELAY;
	bool ifConstantSet;
	double kp, ki, kd;
	clock_t lastTime;
	double ITerm,  lastInput, tolerance;
	double Input, Output;
	double outMin, outMax;
	double Setpoint;
	void setConstants(double, double, double, double);
	void outputLimits(double , double);
	void execute(double);
	
void setConstants(double Kp, double Ki, double Kd, double tol)
{
	kp = Kp;
	ki = Ki;
	kd = Kd;
	tolerance = tol;
	ifConstantSet = true;
}

void Initialize()
{
	lastInput = inputfunc();
	ITerm = 0;
	lastTime = clock();
}

void outputLimits(double Min, double Max)
{
	if(Min > Max)
	{
		std::cout << "minimum cannot be greater than maximum!" << std::endl;
		return;
	}
	
	outMin = Min;
	outMax = Max;
}

void execute(double setPt)
{
	long int i;
	
	if(!ifConstantSet)
	{
		std::cout << "Parameters not set!" << std::endl;
		return;
	}
	
	Setpoint = setPt;
	Initialize();
	
	while(1)
	{
		
		for(i=0; i<DELAY; i++);
		
		clock_t now = clock();
		double timeChange = (double)(now - lastTime)/(CLOCKS_PER_SEC);
		
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
	}
}

