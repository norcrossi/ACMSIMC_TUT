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
    CTRL.rotor_flux_cmd = 0.5;

    CTRL.omg_ctrl_err = 0.0;
    CTRL.speed_ctrl_err = 0.0;

    CTRL.omg_fb = 0.0;
    CTRL.ial_fb = 0.0;
    CTRL.ibe_fb = 0.0;
    CTRL.psi_mu_al_fb = 0.0;
    CTRL.psi_mu_be_fb = 0.0;

    CTRL.theta_M = 0.0;
    CTRL.cosT = 0.0;
    CTRL.sinT = 1;

    CTRL.uMs_cmd = 0.0;
    CTRL.uTs_cmd = 0.0;
    CTRL.iMs_cmd = 0.0;
    CTRL.iTs_cmd = 0.0;

    CTRL.omega_syn = 0.0;
    CTRL.omega_sl = 0.0;

    // ver. IEMDC
    CTRL.pi_speed.Kp = 0.5; 
    CTRL.pi_speed.Ti = 5;
    CTRL.pi_speed.Ki = (CTRL.pi_speed.Kp*4.77) / CTRL.pi_speed.Ti * (TS*VC_LOOP_CEILING*DOWN_FREQ_EXE_INVERSE);
    CTRL.pi_speed.i_state = 0.0;
    CTRL.pi_speed.i_limit = 8;

    printf("Kp_omg=%g, Ki_omg=%g\n", CTRL.pi_speed.Kp, CTRL.pi_speed.Ki);

    CTRL.pi_iMs.Kp = 15; // cutoff frequency of 1530 rad/s
    CTRL.pi_iMs.Ti = 0.08;
    CTRL.pi_iMs.Ki = CTRL.pi_iMs.Kp/CTRL.pi_iMs.Ti*TS; // =0.025
    CTRL.pi_iMs.i_state = 0.0;
    CTRL.pi_iMs.i_limit = 350; //350.0; // unit: Volt

    CTRL.pi_iTs.Kp = 15;
    CTRL.pi_iTs.Ti = 0.08;
    CTRL.pi_iTs.Ki = CTRL.pi_iTs.Kp/CTRL.pi_iTs.Ti*TS;
    CTRL.pi_iTs.i_state = 0.0;
    CTRL.pi_iTs.i_limit = 650; // unit: Volt, 350V->max 1300rpm

    printf("Kp_cur=%g, Ki_cur=%g\n", CTRL.pi_iMs.Kp, CTRL.pi_iMs.Ki);
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
void IMCLV(double spdcmd, double torqcmd)
{
    signalprocess();
    foc();
    currentPI();
    //departcontrol();
    //protection();
}

void signalprocess(double spdcmd, double torqcmd)
{    

    //速度指令
    CTRL.omg_cmd = CTRL.rpm_cmd*(2 * M_PI * IM.npp)/60;

    //速度反馈
    //IM.x[4]为微分方程里电机的omega
    CTRL.omg_fb = IM.x[4];

    //电流指令
    //=============================d轴电流指令===================================//
    // Flux (linkage) command
    CTRL.rotor_flux_cmd = 1.5; // f(speed, dc bus voltage, last torque current command)
    // CTRL.rotor_flux_cmd = 3;
        // 1. speed is compared with the base speed to decide flux weakening or not
        // 2. dc bus voltage is required for certain application
        // 3. last torque current command is required for loss minimization

    // M-axis current command
    CTRL.iMs_cmd = CTRL.rotor_flux_cmd*CTRL.Lmu_inv + M1*OMG1*cos(OMG1*CTRL.timebase) / CTRL.rreq;
    // printf("%g, %g, %g\n", CTRL.Lmu_inv, CTRL.iMs_cmd, CTRL.iTs_cmd);
    //=============================d轴电流指令end===================================//

    //=============================Q轴电流指令===================================//
    CTRL.omg_ctrl_err = CTRL.omg_cmd - CTRL.omg_fb;

    //速度PI输出转矩iT指令
    CTRL.iTs_cmd = - PI(&CTRL.pi_speed, CTRL.omg_ctrl_err);
    //=============================Q轴电流指令end=================================//

    //电流反馈
    CTRL.ial_fb = IS_C(0);
    CTRL.ibe_fb = IS_C(1);


}

void foc(double spdcmd, double torqcmd)
{   

    //磁链OB
    FLUXOB(CTRL.iMs,CTRL.iTs,CTRL.omg_fb);

    //帕克变换角度正余弦值 theta_M控制轴角度=磁链角度
    CTRL.cosT = cos(CTRL.theta_M); 
    CTRL.sinT = sin(CTRL.theta_M);
    
    // Measured current in M-T frame
    CTRL.iMs = AB2M(CTRL.ial_fb, CTRL.ibe_fb, CTRL.cosT, CTRL.sinT);
    CTRL.iTs = AB2T(CTRL.ial_fb, CTRL.ibe_fb, CTRL.cosT, CTRL.sinT);

}


void currentPI()
{      
    //PI输出MT电流
    double vM, vT;
    vM = - PI(&CTRL.pi_iMs, CTRL.iMs-CTRL.iMs_cmd);
    vT = - PI(&CTRL.pi_iTs, CTRL.iTs-CTRL.iTs_cmd);

    // Current loop decoupling (skipped for now)
    CTRL.uMs_cmd = vM;
    CTRL.uTs_cmd = vT;

    // Voltage command in alpha-beta frame
    CTRL.ual = MT2A(CTRL.uMs_cmd, CTRL.uTs_cmd, CTRL.cosT, CTRL.sinT);
    CTRL.ube = MT2B(CTRL.uMs_cmd, CTRL.uTs_cmd, CTRL.cosT, CTRL.sinT);
}



void SoftStarter(double rpm_limit, double acctime)
{
    SORFTSTARTER.scale = rpm_limit/(SORFTSTARTER.acc/TS); //计算每个MCU控制周期增加的RPM步长 //TS (IM_TS*DOWN_FREQ_EXE) //2.5e-4

    if (CTRL.timebase < SORFTSTARTER.acc)
    { 
        SORFTSTARTER.rpm_now += SORFTSTARTER.scale;                   
    } 

}