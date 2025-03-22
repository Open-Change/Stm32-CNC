/********************************************************************
* Description: kinematics for corexy
* Adapted from trivkins.c
* ref: http://corexy.com/theory.html
********************************************************************/

#include "motion.h"
#include "hal.h"
#include "rtapi.h"
#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_math.h"
#include "rtapi_string.h"
#include "kinematics.h"

static struct data {
    hal_s32_t joints[EMCMOT_MAX_JOINTS];
    hal_float_t motor1; // Birinci motorun hareket değeri
    hal_float_t motor2; // İkinci motorun hareket değeri
} *data;

// HAL veri yapısı: Motor hareketlerini saklamak için kullanılır

#define Motor1 (data->motor1) // Birinci motorun hareket değerine erişim
#define Motor2 (data->motor2) // İkinci motorun hareket değerine erişim



int kinematicsForward(const double *joints
                     ,EmcPose *pos
                     ,const KINEMATICS_FORWARD_FLAGS *fflags
                     ,KINEMATICS_INVERSE_FLAGS *iflags
                     ) {
    (void)fflags;
    (void)iflags;
    pos->tran.x = 0.5 * (joints[0] + joints[1]);
    pos->tran.y = 0.5 * (joints[0] - joints[1]);
    pos->tran.z = joints[2];
    pos->a      = joints[3];
    pos->b      = joints[4];
    pos->c      = joints[5];
    pos->u      = joints[6];
    pos->v      = joints[7];
    pos->w      = joints[8];

    return 0;
}

int kinematicsInverse(const EmcPose *pos
                     ,double *joints
                     ,const KINEMATICS_INVERSE_FLAGS *iflags
                     ,KINEMATICS_FORWARD_FLAGS *fflags
                     ) {
    (void)iflags;
    (void)fflags;
    joints[0] = pos->tran.x + pos->tran.y;
    joints[1] = pos->tran.x - pos->tran.y;
    joints[2] = pos->tran.z;
    joints[3] = pos->a;
    joints[4] = pos->b;
    joints[5] = pos->c;
    joints[6] = pos->u;
    joints[7] = pos->v;
    joints[8] = pos->w;

    return 0;
}

int kinematicsHome(EmcPose *world
                  ,double *joint
                  ,KINEMATICS_FORWARD_FLAGS *fflags
                  ,KINEMATICS_INVERSE_FLAGS *iflags
                  ) {
    *fflags = 0;
    *iflags = 0;
    return kinematicsForward(joint, world, fflags, iflags);
}

KINEMATICS_TYPE kinematicsType() { return KINEMATICS_BOTH; }

KINS_NOT_SWITCHABLE
EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

static int comp_id;
int rtapi_app_main(void) {
	 int res = 0;
    comp_id = hal_init("3dkinamtil");
    if(comp_id < 0) return comp_id;

    data = hal_malloc(sizeof(struct data));
    


    // Motor hareket değerlerini başlangıç durumuna getir
    Motor1 = 0.0;
    Motor2 = 0.0;

    // HAL parametrelerini oluştur
    if((res = hal_param_float_new("3dkinamtil.Motor1", HAL_RW, &data->motor1, comp_id)) < 0) goto error;
    if((res = hal_param_float_new("3dkinamtil.Motor2", HAL_RW, &data->motor2, comp_id)) < 0) goto error;

    hal_ready(comp_id);
    return 0;
    
    error:
    // Hata durumunda HAL bileşenini kapat
    hal_exit(comp_id);
    return res;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
