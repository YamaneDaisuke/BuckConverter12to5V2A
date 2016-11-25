#include "PIDController.h"
#include <math.h>
#include <stdio.h>

PIDController::PIDController(float Kc, float tauI, float tauD)
{
    reset();
    setTunings(1.0, 0.0, 0.0);
    setTunings(Kc, tauI, tauD);
    _targetPoint = 0.0;

    _pParam = Kc;
    _iParam = tauI;
    _dParam = tauD;

    setTargetValue(0.0);

    setLimitMax(1023);
    setLimitMin(0);
    _range = _max - _min;


    // Process Value
    _processValue = 0;
    _preProcessValue = 0;

    // Output value
    _controllerOutput = 0;
    _preControllerOutput = 0;
}

void PIDController::setTunings(float Kc, float tauI, float tauD)
{
    // invalid parameters
    if (Kc == 0.0 || tauI < 0.0 || tauD < 0.0) {
        return;
    }
    _kc = (int)floor(Kc * (1<<SFT));
    _tauI = (int)floor(tauI * (1<<SFT));
    _tauD = (int)floor(tauD  * (1<<SFT));
}

void PIDController::reset(void)
{
    _accError = 0;
}

void PIDController::setTargetValue(float target)
{
    _targetPoint = target;
    setTargetValueFixedPoint((int)floor(target * (1<<SFT)));
}

void PIDController::setTargetValueFixedPoint(int target)
{
    _target = target;
}

void PIDController::setProcessValueFixedPoint(int pv)
{
    _preProcessValue = _processValue;
    _processValue = pv;
}

void PIDController::setLimitMax(float max)
{
    _maxRaw = max;
    setLimitMaxFixedPoint((int)floor(max * (1<<SFT)));
}

void PIDController::setLimitMaxFixedPoint(int max)
{
    _max = max;
}

void PIDController::setLimitMin(float min)
{
    _minRaw = min;
    setLimitMinFixedPoint((int)floor(min * (1<<SFT)));
}

void PIDController::setLimitMinFixedPoint(int min)
{
    _min = min;
}

void PIDController::setBias(float bias)
{
    _biasRaw = bias;
    setBiasFixedPoint((int)floor(bias * (1<<SFT)));
}
void PIDController::setBiasFixedPoint(int bias)
{
    _bias = bias;
}

void PIDController::start(void)
{
    printf("PIDController::start is not available\n");
}

int  PIDController::calculate(void)
{
    int error = _target - _processValue;
    //int scaledPV = ((_processValue - _min) << SFT) / _range;
    //int scaledTV = ((_target - _min) << SFT) / _range;
    int scaledPV = _processValue - _min;
    int scaledTV = _target - _min;
    if(!(_preControllerOutput >= _max && error > 0) && !(_preControllerOutput <= _min && error < 0)) {
        _accError += error;
    }
    _controllerOutput = _bias + (_kc * (error + ((_tauI * _accError)>>SFT)) >> SFT);
    if(_controllerOutput > _max) {
        _controllerOutput = _max;
    } else if(_controllerOutput < _min) {
        _controllerOutput = _min;
    }
    _preControllerOutput = _controllerOutput;

    return _controllerOutput >> (SFT+2);
}
