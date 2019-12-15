
#ifndef CONTROLLER_H
#define CONTROLLER_H

struct PI_Reg{
   double   Kp;
   double   Ti;
   double   Ki; // Ki = Kp/Ti*TS
   double   i_state;
   double   i_limit;
};
double PI(struct PI_Reg *r, double err);

struct ControllerForExperiment{

    double timebase;

    double ual;
    double ube;

    double rs;
    double rreq;
    double Lsigma;
    double alpha;
    double Lmu;
    double Lmu_inv;

    double Tload;
    double rpm_cmd;

    double Js;
    double Js_inv;
};
extern struct ControllerForExperiment CTRL;

struct SofterStarter{
    double scale;    //(IM_TS*DOWN_FREQ_EXE) 1=2.5e-4s MCU控制周期
    double rpm_now;
    double acc;   //1s
    double dcc;   //1s
};
extern struct SofterStarter SORFTSTARTER;


void CTRL_init();
void SoftStarter();


#endif
