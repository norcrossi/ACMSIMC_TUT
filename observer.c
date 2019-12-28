#include "ACMSim.h"

struct InductionMachine im;


void FLUXOB(double iMs, double iTs, double omega_r) //输入定子MT电流以及转子的角速度，推算出磁链角度
{   
	im.iMr_ob += (iMs-im.iMr_ob)*IM.alpha;  //通过转子常数预测出的转子M轴电流大小   
	CTRL.omg_M = omega_r + IM.alpha*(iTs/im.iMr_ob) //转子M轴电流推算出滑差再加上转子反馈速度，推算出磁链的角速度
	CTRL.theta_M += CTRL.omg_M; //控制轴的角速度积分即为磁链的角度
}