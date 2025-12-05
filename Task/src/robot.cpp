#include "robot.h"

static motor::MotorParameter wheelMotorParameter[4] =
    {{
         .id = 1,
         .dir = motor::kDirFwd,
         .ex_redu_rat = 1,
         .remove_build_in_reducer = true,
         .need_limit_vel = true,
         .max_vel = 500,
         .limitVel_CurrProportion = 0.8,
     },
     {
         .id = 2,
         .dir = motor::kDirFwd,
         .ex_redu_rat = 1,
         .remove_build_in_reducer = true,
         .need_limit_vel = true,
         .max_vel = 500,
         .limitVel_CurrProportion = 0.8,
     },
     {
         .id = 3,
         .dir = motor::kDirFwd,
         .ex_redu_rat = 1,
         .remove_build_in_reducer = true,
         .need_limit_vel = true,
         .max_vel = 500,
         .limitVel_CurrProportion = 0.8,
     },
     {
         .id = 4,
         .dir = motor::kDirFwd,
         .ex_redu_rat = 1,
         .remove_build_in_reducer = true,
         .need_limit_vel = true,
         .max_vel = 500,
         .limitVel_CurrProportion = 0.8,

     }};

static motor::MotorParameter dribblerMotorParameter = {
    .id = 5,
    .dir = motor::kDirRev,
    .ex_redu_rat = 20,
    .remove_build_in_reducer = false,
};

static pid::parameter pidParameter[4] = {
    {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .outputLimit = motor::MotorInfo_DM3519.broad_curr_limit,
        .integLimit = motor::MotorInfo_DM3519.broad_curr_limit,
        .dt = 1.0 / 1000.0,
    },
    {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .outputLimit = motor::MotorInfo_DM3519.broad_curr_limit,
        .integLimit = motor::MotorInfo_DM3519.broad_curr_limit,
        .dt = 1.0 / 1000.0,
    },
    {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .outputLimit = motor::MotorInfo_DM3519.broad_curr_limit,
        .integLimit = motor::MotorInfo_DM3519.broad_curr_limit,
        .dt = 1.0 / 1000.0,
    },
    {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .outputLimit = motor::MotorInfo_DM3519.broad_curr_limit,
        .integLimit = motor::MotorInfo_DM3519.broad_curr_limit,
        .dt = 1.0 / 1000.0,
    }};

static pid::parameter velPIDParameter[4] = {
    {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .outputLimit = 4,
        .integLimit = 4,
        .dt = 1.0 / 1000.0,
    },
    {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .outputLimit = 4,
        .integLimit = 4,
        .dt = 1.0 / 1000.0,
    },
    {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .outputLimit = 4,
        .integLimit = 4,
        .dt = 1.0 / 1000.0,
    },
    {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .outputLimit = 4,
        .integLimit = 4,
        .dt = 1.0 / 1000.0,
    }};

static pid::parameter dribblerPIDParameter = {

    .kp = 0.001,
    .ki = 0.005,
    .kd = 0,
    .outputLimit = motor::MotorInfo_M2006.broad_curr_limit,
    .integLimit = motor::MotorInfo_M2006.broad_curr_limit,
    .dt = 1.0 / 1000.0,

};

static td::parameter dribblerTDParameter = {
    .r = 2000,
    .h = 0.01,
    .dt = 1.0 / 1000.0,
    .is_cycle = false,
    .cycle_low = -180.0,
    .cycle_high = 180.0,
};

static td::parameter wheelTDParameter[4] = {
    {
        .r = 200000,
        .h = 0.01,
        .dt = 1.0 / 1000.0,
        .is_cycle = false,
        .cycle_low = -180.0,
        .cycle_high = 180.0,
    },
    {
        .r = 200000,
        .h = 0.01,
        .dt = 1.0 / 1000.0,
        .is_cycle = false,
        .cycle_low = -180.0,
        .cycle_high = 180.0,
    },
    {
        .r = 200000,
        .h = 0.01,
        .dt = 1.0 / 1000.0,
        .is_cycle = false,
        .cycle_low = -180.0,
        .cycle_high = 180.0,
    },
    {
        .r = 200000,
        .h = 0.01,
        .dt = 1.0 / 1000.0,
        .is_cycle = false,
        .cycle_low = -180.0,
        .cycle_high = 180.0,
    }};

static float wheelAngleForward = 65.0 / 180.0 * PI;
static float wheelAngleBackward = 37.0 / 180.0 * PI;
static float sinWheelAngleForward = sinf(wheelAngleForward);   
static float cosWheelAngleForward = cosf(wheelAngleForward);   
static float sinWheelAngleBackward = sinf(wheelAngleBackward); 
static float cosWheelAngleBackward = cosf(wheelAngleBackward); 
static float robot_M = 2196.92 / 1000.0;
static float robot_J = 2295873.57 / 1e9f;
static float robot_D = (85.0 + 61.0) / 2.0 / 1000.0;
static float wheel_M __attribute__((unused)) = 220.83 / 1000.0;
static float wheel_J = 83800 / 1e9f;
static float wheel_D = 57.0 / 1000.0 / 2.0;
static float FtMax __attribute__((unused)) = 10; 
static float ikSolveA[3][4] = {
    {sinWheelAngleForward, -sinWheelAngleForward, -sinWheelAngleBackward, sinWheelAngleBackward},
    {cosWheelAngleForward, cosWheelAngleForward, -cosWheelAngleBackward, -cosWheelAngleBackward},
    {1, 1, 1, 1}};

static float ikSolvePinvA[4][3] = {
    {0.3829, 0.4094, 0.3270},
    {-0.3829, 0.4094, 0.3270},
    {-0.2542, -0.4094, 0.1730},
    {0.2542, -0.4094, 0.1730}};

const float wheel_vx_angle[4] = {sinWheelAngleForward, -sinWheelAngleForward, -sinWheelAngleBackward, sinWheelAngleBackward};
const float wheel_vy_angle[4] = {cosWheelAngleForward, cosWheelAngleForward, -cosWheelAngleBackward, -cosWheelAngleBackward};
const float wheel_vw_angle[4] = {cosf(PI / 4 - (PI / 2 - wheelAngleForward)), cosf(PI / 4 - (PI / 2 - wheelAngleForward)), cosf(PI / 4 - (PI / 2 - wheelAngleBackward)), cosf(PI / 4 - (PI / 2 - wheelAngleBackward))};

static float wheelParallelResistanceTorq = 0.016;
static float verticalResistance = 0.0;
// static float motorCompensationTorq = 0.05;

static float accThreshold[3] = {2.0, 2.0, 20.0};

robot_::robot_()
{

    for (int i = 0; i < 4; i++)
    {
        wheelMotor[i] = new motor::Motor_DM3519(wheelMotorParameter[i]);
        wheelPID[i] = new pid::PID(pidParameter[i]);
        wheelVelPID[i] = new pid::PID(velPIDParameter[i]);
        wheelFilter[i] = new td::td(wheelTDParameter[i], 0);
    }

    dribbler = new motor::Motor_M2006(dribblerMotorParameter);
    dribblerPID = new pid::PID(dribblerPIDParameter);
    dribblerFilter = new td::td(dribblerTDParameter, 0);

    imu = new imu::Imu();
}

robot_::~robot_()
{
    for (int i = 0; i < 4; i++)
    {
        delete wheelMotor[i];
    }

    delete dribbler;
}

void robot_::piDecodeUart(void)
{
}

void robot_::piEncodeUart(void)
{
}

void robot_::piDecodeSpi(void)
{

    memcpy(&SpiRx, spiRxData, sizeof(SpiRx));

    for (uint8_t i = 0; i < 2; i++)
    {
        // robot_vel[i] = SpiRx.vel[i] / 1000.0;
    }
    // robot_vel[2] = SpiRx.vel[2] / 100.0;

    for (uint8_t i = 0; i < 3; i++)
    {
        // wheel_PID[i] = SpiRx.wheel_pid[i] / 1000.0;
    }
}

float imu_t[9] = {0};

void robot_::piEncodeSpi(void)
{
    SpiTx.infrare_flag = (INFRA_ADC1_Value > infraredThreshold) ? 1 : 0;
    SpiTx.getBall = false;
    SpiTx.imu_online = true;
    SpiTx.battery_vol = (int16_t)(BATVOL_ADC2_Value * 5);
    SpiTx.cap_vol = (int16_t)(CAPVOL_ADC3_Value * 100);

    for (uint8_t i = 0; i < 4; i++)
    {
        SpiTx.wheel[i] = (int16_t)(wheelMotor[i]->vel() * 10);
        SpiTx.wheel_ref[i] = (int16_t)(motor_vel[i] * 10);
    }

    imu->getData(imu_t);

    for (uint8_t i = 0; i < 9; i++)
    {
        SpiTx.imu_data[i] = (int16_t)(imu_t[i] * 100);
    }

    memcpy(spiTxData, &SpiTx, sizeof(SpiTx));
}

void robot_::ik_solve(void)
{

    float x[3] = {0};

    x[0] = robot_acc[0] * robot_M + 2 * verticalResistance * (cosWheelAngleForward + cosWheelAngleBackward);
    x[1] = robot_acc[1] * robot_M + 2 * verticalResistance * (sinWheelAngleForward + sinWheelAngleBackward);
    x[2] = robot_acc[2] * robot_J / robot_D;

    for (uint8_t i = 0; i < 4; i++)
    {
        motor_F[i] = 0;
        for (uint8_t j = 0; j < 3; j++)
        {
            motor_F[i] += ikSolvePinvA[i][j] * x[j];
        }
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        

        motor_acc[i] = (robot_acc[0] * wheel_vx_angle[i] + robot_acc[1] * wheel_vy_angle[i] + robot_acc[2] * wheel_vw_angle[i] * robot_D) / wheel_D + robot.wheelVelPID[i]->calc(robot.motor_vel[i], robot.wheelMotor[i]->vel());
        motor_vel[i] = ((robot_real_vel[0] * wheel_vx_angle[i] + robot_real_vel[1] * wheel_vy_angle[i] + robot_real_vel[2] * wheel_vw_angle[i] * robot_D) / wheel_D) * 30.0 / PI;
    }

    for (int i = 0; i < 4; i++)
    {
        float resistance = 0;
        // if (robot.wheelMotor[i]->vel() > 20.0)
        // {
        //     resistance = wheelParallelResistanceTorq;
        // }
        // else if (robot.wheelMotor[i]->vel() < -20.0)
        // {
        //     resistance = -wheelParallelResistanceTorq;
        // }
        // else
        // {
        //     resistance = 0;
        // }

        if (motor_vel[i] > 0.001)
        {
            resistance = wheelParallelResistanceTorq;
        }
        else if (motor_vel[i] < -0.001)
        {
            resistance = -wheelParallelResistanceTorq;
        }
        else
        {
            resistance = 0;
        }
        motor_torq[i] = wheel_J * motor_acc[i] + motor_F[i] * wheel_D + resistance;
        // motor_torq[i] = wheel_J * motor_acc[i] + motor_F[i] * wheel_D;
        // motor_torq[i] = motor_F[i] * wheel_D;
    }
}

// dt in us
void robot_::motion_planner(const double _dt)
{

    for (uint8_t i = 0; i < 3; i++)
    {
        robot_acc[i] = (robot_vel[i] - last_robot_real_vel[i]) / (_dt / 1000000.0);
        robot_acc[i] = limit<float>(robot_acc[i], accThreshold[i]);
        robot_real_vel[i] = last_robot_real_vel[i] + robot_acc[i] * _dt / 1000000.0;
        last_robot_real_vel[i] = robot_real_vel[i];
    }
}

robot_ robot{};
