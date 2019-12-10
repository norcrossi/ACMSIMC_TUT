#include "ACMSim.h"
#include "controller.h"

struct ControllerForExperiment CTRL;

/* Initialization */
void CTRL_init(){
    int i=0,j=0;

    CTRL.timebase = 0.0;

        /* Parameter (including speed) Adaptation */ 
        CTRL.rs     = IM.rs;
        CTRL.rreq   = IM.rreq;
        CTRL.Lsigma = IM.Lsigma;
        CTRL.alpha  = IM.alpha;
        CTRL.Lmu    = IM.Lmu;
        CTRL.Lmu_inv = 1.0/IM.Lmu;
        CTRL.Js     = IM.Js;
        CTRL.Js_inv = 1.0/IM.Js;    

    CTRL.ual = 0.0;
    CTRL.ube = 0.0;

    CTRL.rpm_cmd = 0.0;
}
void SoftStarter(double rpm_limit, double acctime)
{
    SORFTSTARTER.scale = rpm_limit/(SORFTSTARTER.acc/TS); //计算每个MCU控制周期增加的RPM步长 //TS (IM_TS*DOWN_FREQ_EXE) //2.5e-4 
    if (SORFTSTARTER.rpm_now <= rpm_limit)
    {
      SORFTSTARTER.rpm_now += SORFTSTARTER.scale;                    
    } 
}
