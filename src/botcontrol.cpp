#include "botcontrol.h"
#include "main.h"
#include "robot-config.h"
using namespace vex;

  // if (fabs(input)<width){
  //   return(0);
  // }
  // return(input);
int pxpos = 480/2;
int pypos = 240/2;
int turningCurve = 5;
int x = 1;
int push = 0;
int y = 1;
int push1 = 0;
bool turningRed = false;

int forwardCurve = 10;
bool forwardRed = false;

enum curve {linear=0, exponential=1, logarithmic=2, s_curve=3,parabolic=4,cubic=5,quadratic=6,step=7,sensitivity=8};

// Linear Curve with gradient
double f_linear(double x, double a = 1, double b = 0) {
    return a * x + b;
}

// Exponential Curve with base and scale
double f_exponential(double x, double a = M_E, double b = 1) {
    return b * (std::pow(a, x) - 1);
}

// Logarithmic Curve with base and scale
double f_logarithmic(double x, double a = M_E, double b = 1) {
    return b * (std::log(x + 1) / log(a));
}

// S-Curve (Sigmoid Curve) with steepness and midpoint
double f_s_curve(double x, double a = 1, double b = 0) {
    return 1 / (1 + std::exp(-a * (x - b)));
}

// Parabolic Curve with coefficient and offset
double f_parabolic(double x, double a = 1, double b = 0) {
    return a * std::pow(x, 2) + b;
}

// Cubic Curve with coefficient and offset
double f_cubic(double x, double a = 1, double b = 0) {
    return a * std::pow(x, 3) + b * std::pow(x, 2);
}

// Quadratic Curve with coefficients
double f_quadratic(double x, double a = 1, double b = 0) {
    return a * std::pow(x, 2) + b * x;
}

// Step Curve with number of steps and scale
double f_step(double x, double a = 10, double b = 1) {
    double step_size = 1.0 / a;
    return b * std::floor(x / step_size) * step_size;
}

// Sensitivity Curve with sensitivity level and offset
double f_sensitivity(double x, double a = 2.0, double b = 0) {
    return std::pow(x, a) + b;
}
double f_custom(double x, double a = 2.0, double b = 0) {
    return std::pow(x, a) + b;
}

int curveJoystick(int set_curve , int input,int max_input, double param_1, double param_2){
  double input_norm = input/max_input;
  switch(set_curve){
      case linear:
         return f_linear(input_norm, param_1, param_2);
      case exponential:
         return  f_exponential(input_norm, param_1, param_2);
      case logarithmic:
        return  f_logarithmic(input_norm, param_1, param_2);
      case s_curve:
         return f_s_curve(input_norm, param_1, param_2);
      case parabolic:
         return f_parabolic(input_norm, param_1, param_2);
      case cubic:
         return f_cubic(input_norm, param_1, param_2);
      case quadratic:
         return f_quadratic(input, param_1, param_2);
      case step:
         return f_step(input_norm, param_1, param_2);
      case sensitivity:
         return f_sensitivity(input_norm, param_1, param_2);
      
      default:
          return 0;
  }
  /*
  if(red){
    val = (std::exp(-t/10)+std::exp((std::abs(input)-100)/10)*(1-std::exp(-t/10))) * input;
  }else{
    //blue
    val = std::exp(((std::abs(input)-100)*t)/1000) * input;
  }
  return val;
  */
}

void driver(){
  while(1){
    // double turnVal = curveJoystick(false, con.Axis1.position(percent), turningCurve); //Get curvature according to settings [-100,100]
    // double forwardVal = curveJoystick(false, con.Axis3.position(percent), forwardCurve); //Get curvature according to settings [-100,100]

    // double turnVolts = turnVal * 0.12; //Converts to voltage
    // double forwardVolts = forwardVal * 0.12; //Converts to voltage

    // leftmo.spin(forward, forwardVolts + turnVolts, voltageUnits::volt); //Apply Via Voltage
    // rightmo.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

    // vex::task::sleep(20);
    //double speed = curveJoystick(int set_curve , int input, double param_1, double param_2) * 100;
    double input_axis_1=con.Axis1.position(pct);
    double speed = curveJoystick(2 , input_axis_1,100, 0.02, 0.04) * 100;
    /////////////////////////////////////
    if(con.ButtonR1.pressing()){
      inta.spin(reverse, 100, pct);
    } else if(con.ButtonR2.pressing()){
      inta.spin(fwd, 100, pct);
    } else{
      inta.stop(hold);
    }

      //elevation button need to change
    if(con.ButtonL2.pressing() == true)
      {
        x++;
      }
    else
      {
        x = 0 ;
      }
    if(x == 1)
      {
        if(push == 0)
        {
          elevation.set(true);
          push = 1;
        }
        else if (push == 1)
        {
          
          elevation.set(false);
          
          push = 0;
        }
      }
    //wing
    if(con.ButtonL1.pressing() == true)
      {
        y++;
      }
    else
      {
        y = 0 ;
      }
    if(y == 1)
      {
        if(push1 == 0)
        {
          intawings.set(true);
          push1 = 1;
        }
        else if (push1 == 1)
        {
          intawings.set(false);
          push1 = 0;
        }
      }

/////////////////////////////////////////////////////////////////

    double axis3 = con.Axis3.position(pct);
    double axis1 = -con.Axis1.position(pct);
    double leftVolt = axis3 - axis1;
    double rightVolt = axis3 + axis1;
    double scale = 12.0 / fmax(12.0, fmax(fabs(leftVolt), fabs(rightVolt)));
    leftVolt *= scale;
    rightVolt *= scale;
    if (fabs(leftVolt) < 0.1){
        leftmo.stop(brake);
    } 
    else{
        leftmo.spin(forward, leftVolt, volt);
    }
    if(fabs(rightVolt) < 0.1){
        rightmo.stop(brake);
    }
    else{
        rightmo.spin(forward, rightVolt, volt);
    }

//////////////////////////////////////////////////////////////////////////////


    wait(20, msec);
  }
}
