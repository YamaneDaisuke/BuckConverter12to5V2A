#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * Includes
 */
#include "mbed.h"

/**
 * define
 */
#define SFT 6 //num of shift

class PIDController
{
public :
    PIDController(float Kc, float tauI, float tauD);
    void setTunings(float Kc, float tauI, float tauD);
    void setTargetValue(float target);
    void setTargetValueFixedPoint(int target);
    void setLimitMax(float max);
    void setLimitMaxFixedPoint(int max);
    void setLimitMin(float min);
    void setLimitMinFixedPoint(int min);
    void setBias(float bias);
    void setBiasFixedPoint(int bias);
    void start(void);
    void reset(void);
    void setProcessValueFixedPoint(int pv);
    int calculate(void);
    
private :
    // Actual tuning parameters used in PID calculation.
    int _kc;
    int _tauI;
    int _tauD;

    // Raw tuning parameters
    float _pParam;
    float _iParam;
    float _dParam;

    // Target point (real value)
    float _targetPoint;
    // Target point (used in PID calculation)
    int _target;

    // Process Value
    int _processValue;
    int _preProcessValue;

    // Output value
    int _controllerOutput;
    int _preControllerOutput;

    // Accumulated error
    int _accError;

    // limit and bias (for processer)
    int _max;
    int _min;
    int _bias;

    // limit and bias(real value)
    float _maxRaw;
    float _minRaw;
    float _biasRaw;

    int _range;

};


#endif
