#include "ACMSim.h"

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

    if (CTRL.timebase < SORFTSTARTER.acc)
    { 
        SORFTSTARTER.rpm_now += SORFTSTARTER.scale;                   
    } 

}
double PI(struct PI_Reg *r, double err){
    #define I_STATE r->i_state
    #define I_LIMIT r->i_limit
    double output;
    I_STATE += err * r->Ki;    // 积分
    if( I_STATE > I_LIMIT)     // 添加积分饱和特性
        I_STATE = I_LIMIT; 
    else if( I_STATE < -I_LIMIT)
        I_STATE = -I_LIMIT;

    output = I_STATE + err * r->Kp;

    if(output > I_LIMIT)
        output = I_LIMIT;
    else if(output < -I_LIMIT)
        output = -I_LIMIT;
    return output;
    #undef I_STATE
    #undef I_LIMIT
}
void IMCLV(double spd, double torq，double)
{
    
}


