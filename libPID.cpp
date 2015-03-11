#include<ctime>
#include<iostream>

class pid
{
  private:

    double INF;
    long int DELAY;

    bool ifConstantSet;

    double kp, ki, kd;
    clock_t lastTime;

    double (*finput)();
    void (*foutput)(double);

    double ITerm,  lastInput, tolerance;
    double Input, Output;
    double outMin, outMax;

    double Setpoint;
    void Initialize();

  public:

    pid(double (*inpt_func)(), void (*outpt_func)(double));
    void setConstants(double, double, double, double);
    void outputLimits(double , double);
    void execute(double);

};

pid::pid(double (*inpt_func)(), void (*outpt_func)(double))
{
    INF = 3e38;
    DELAY = 50000000;

    finput = inpt_func;
    foutput = outpt_func;

    outMin = -INF;
    outMax = INF;

    ifConstantSet = false;
}

void pid::setConstants(double Kp, double Ki, double Kd, double tol)
{
    kp = Kp;
    ki = Ki;
    kd = Kd;
    tolerance = tol;
    ifConstantSet = true;
}

void pid::outputLimits(double Min, double Max)
{
   if(Min > Max)
   {
	std::cout << "minimum cannot be greater than maximum!" << std::endl;
	return;
   }

   outMin = Min;
   outMax = Max;
}

void pid::execute(double setPt)
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

    Input = (*finput)();
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

    (*foutput)(Output);

    lastInput = Input;
    lastTime = now;
    }
}

void pid::Initialize()
{
   lastInput = (*finput)();
   ITerm = 0;
   lastTime = clock();
}
