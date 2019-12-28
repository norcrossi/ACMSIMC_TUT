
#ifndef CONTROLLER_H
#define CONTROLLER_H
//======================= PI=============================//
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

    double rpm_cmd;
    double omg_cmd;

    double theta_M; //磁链角度 控制轴角度
    double omg_M; //磁链角速度 控制轴角速度

    double ids_fb;
    double iqs_fb;
    double omg_fb;


    double err_ids;
    double err_iqs;

    double err_spd;

    double pi_spd;
    double pi_iMs; //M轴控制电流
    double pi_iTs; //T轴控制电流

    struct PI_Reg pi_speed;
    struct PI_Reg pi_iMs;
    struct PI_Reg pi_iTs;

 
};
extern struct ControllerForExperiment CTRL;

//======================= PI=============================//

void CTRL_init();

void IMCLV();


#endif
