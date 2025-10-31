/*************************************************************************************************************
 *  Template project for Extended Kalman Filter library
 * 
 * See https://github.com/pronenewbits for more!
 ************************************************************************************************************/
#include <Wire.h>
#include <elapsedMillis.h>
#include "konfig.h"
#include "matrix.h"
#include "ekf.h"


/* ============================================ EKF Variables/function declaration ============================================ */
/* Just example; in konfig.h: 
 *  SS_X_LEN = 2
 *  SS_Z_LEN = 1
 *  SS_U_LEN = 1 
 */
/* EKF initialization constant -------------------------------------------------------------------------------------- */
#define P_INIT      (10.)
#define Q_INIT      (1e-6)
#define R_INIT      (0.0015)
/* P(k=0) variable -------------------------------------------------------------------------------------------------- */
float_prec EKF_PINIT_data[SS_X_LEN*SS_X_LEN] = {P_INIT, 0,
                                                0,      P_INIT};
Matrix EKF_PINIT(SS_X_LEN, SS_X_LEN, EKF_PINIT_data);
/* Q constant ------------------------------------------------------------------------------------------------------- */
float_prec EKF_QINIT_data[SS_X_LEN*SS_X_LEN] = {Q_INIT, 0,
                                                0,      Q_INIT};
Matrix EKF_QINIT(SS_X_LEN, SS_X_LEN, EKF_QINIT_data);
/* R constant ------------------------------------------------------------------------------------------------------- */
float_prec EKF_RINIT_data[SS_Z_LEN*SS_Z_LEN] = {R_INIT};
Matrix EKF_RINIT(SS_Z_LEN, SS_Z_LEN, EKF_RINIT_data);
/* Nonlinear & linearization function ------------------------------------------------------------------------------- */
bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U);
/* EKF variables ---------------------------------------------------------------------------------------------------- */
Matrix X(SS_X_LEN, 1);
Matrix Y(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
/* EKF system declaration ------------------------------------------------------------------------------------------- */
EKF EKF_IMU(X, EKF_PINIT, EKF_QINIT, EKF_RINIT, 
            Main_bUpdateNonlinearX, Main_bUpdateNonlinearY, Main_bCalcJacobianF, Main_bCalcJacobianH);



/* ========================================= Auxiliary variables/function declaration ========================================= */
elapsedMillis timerLed, timerEKF;
uint64_t u64compuTime;
char bufferTxSer[100];



void setup() {
    /* serial to display data */
    Serial.begin(115200);
    while(!Serial) {}
    
    X.vSetToZero();
    EKF_IMU.vReset(X, EKF_PINIT, EKF_QINIT, EKF_RINIT);
}


void loop() {
    if (timerEKF >= SS_DT_MILIS) {
        timerEKF = 0;
        
        
        /* ================== Read the sensor data / simulate the system here ================== */
        /* ... */
        /* ------------------ Read the sensor data / simulate the system here ------------------ */
        
        
        /* ============================= Update the Kalman Filter ============================== */
        u64compuTime = micros();
        if (!EKF_IMU.bUpdate(Y, U)) {
            X.vSetToZero();
            EKF_IMU.vReset(X, EKF_PINIT, EKF_QINIT, EKF_RINIT);
            Serial.println("Whoop ");
        }
        u64compuTime = (micros() - u64compuTime);
        /* ----------------------------- Update the Kalman Filter ------------------------------ */
        
        
        /* =========================== Print to serial (for plotting) ========================== */
        #if (1)
            /* Print: Computation time, X_Est[0] */
            snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "%.3f %.3f ", ((float)u64compuTime)/1000., EKF_IMU.GetX()[0][0]);
            Serial.print(bufferTxSer);
        #endif
        Serial.print('\n');
        /* --------------------------- Print to serial (for plotting) -------------------------- */
    }
}

// CONFIG ROBOT TOUR
#define MIN_DUTY_CYCLE
#define MAX_DUTY_CYCLE
#define CONVERSION_CONSTANT_WHEEL_CIRCUMFERENCE
#define DIST_BETWEEN_WHEELS

#define TOLERANCE 0.0001f
#define PI 3.14159f

bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear update transformation here
     *          x(k+1) = f[x(k), u(k)]

     * Check the below for unit equivalence
     */
    
    float_prec uleft_adj = U[0][0] > 0 ? map(abs(U[0][0]), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE, 0, 1) : map(abs(U[0][0]), -MAX_DUTY_CYCLE, -MIN_DUTY_CYCLE, -1, 0);
    float_prec uright_adj = U[1][0] > 0 ? map(abs(U[1][0]), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE, 0, 1) : map(abs(U[1][0]), -MAX_DUTY_CYCLE, -MIN_DUTY_CYCLE, -1, 0);

    X_Next[0][0] = 0.5f*(uleft_adj + uright_adj)(CONVERSION_CONSTANT_WHEEL_CIRCUMFERENCE);
    X_Next[1][0] = abs(uleft_adj - uright_adj) < TOLERANCE ? X[1][0] : X[1][0] + (180f*X_Next[0][0]*SS_DT / (PI*PI* (.5f*(uleft_adj + uleft_adj)*(DIST_BETWEEN_WHEELS)/(abs(uleft_adj - uright_adj))) ));

    return true;
}

bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear measurement transformation here
     *          y(k)   = h[x(k), u(k)]
     */
    
    return true;
}

bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U)
{
    /* Insert the linearized update transformation here (i.e. Jacobian matrix of f[x(k), u(k)]) */
    
    return true;
}

bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U)
{
    /* Insert the linearized measurement transformation here (i.e. Jacobian matrix of h[x(k), u(k)]) */
    
    return true;
}





void SPEW_THE_ERROR(char const * str)
{
    #if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
        cout << (str) << endl;
    #elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
        Serial.println(str);
    #else
        /* Silent function */
    #endif
    while(1);
}
