/*
 * PID.c
 *
 *  Created on: Apr 26, 2022
 *      Author: Mathew Morrow
 *
 */

#include "pid_static.h"
#include "math.h"
#include "NOTCH_FILTER.h"
#include "IIR.h"


void PID_Init(structPID *PID, float initKp, float initKi, float initKd, float sampleTime)
{
	/* Set sample time first - need for setting tuning parameters */
	PID->sampleTime = sampleTime;
	/* Set tuning parameters - K terms multiplied/integrated by sampling time once */
	SetTunings(PID, initKp, initKi, initKd);
	/* Set output limits */
	SetOutputLimits(PID, 0.0, 0.0, 0.0, 0.0);
	/* Reset some values for predictable startup*/
	PID->hasNotchFilter = 0;
	PID_Reset(PID, 0.0);

	PID->isOn = 1;

	/* Initialize d-term filters */
	float filtGain1 = pt1FilterGain(100, sampleTime);
	float filtGain2 = pt2FilterGain(80, sampleTime);
	pt1FilterInit(&PID->dFilt1, filtGain1);
	pt2FilterInit(&PID->dFilt2, filtGain2);
}

float PID_Compute(structPID *PID, float newInput, float newSetpoint)
{
	float output = 0.0;

	if(PID->isOn)
	{
		if(!(PID->initialized))
		{
			PID_Reset(PID, newInput);
			PID->initialized = 1;
		}
		else
		{
			/* Calculate error from output to set-point */
			float error = newSetpoint - newInput;

			PID->Pterm = (PID->Kp * error);

			/*  Integrate Iterm
			 * Check if PID output is saturated. If so, don't integrate I-term anymore */
			if( (PID->output < PID->maxOutput) && (PID->output > PID->minOutput))
			{
				/* Calculate integral term, calculated and summed separately to prevent integral kick if Ki is changed at runtime */
				PID->Iterm += (PID->Ki * error);
				/* Bound integral term to prevent wind-up issues */
				if(PID->Iterm > PID->maxIntegral) PID->Iterm = PID->maxIntegral;
				else if(PID->Iterm < PID->minIntegral) PID->Iterm = PID->minIntegral;
			}

			/* Calculate derivative value - using derivative-on-input to prevent derivative kick from discrete setpoint changes */
			PID->DtermUnfilt = PID->Kd * (PID->lastInput - newInput);
			/* Filter D term */
			PID->dFilt2Output = pt2FilterApply(&PID->dFilt2, PID->DtermUnfilt);
			PID->dFilt1Output = pt1FilterApply(&PID->dFilt1, PID->dFilt2Output);
			if(PID->hasNotchFilter)
			{
				PID->Dterm = NotchFilter_Update(&PID->notchFilter, PID->dFilt1Output);
			}
			else
			{
				PID->Dterm = PID->dFilt1Output;
			}


			/* Compute PID output*/
			output = (PID->Pterm) + (PID->Iterm) + (PID->Dterm);
			/* Bound PID output to prevent wind-up issues*/
			if(output > PID->maxOutput) output = PID->maxOutput;
			else if(output < PID->minOutput) output = PID->minOutput;
		}
		/*Remember some variables for next time*/
		PID->output = output;
		PID->lastInput = newInput;
	}
    return output;
}

void PID_Reset(structPID *PID, float newInput)
{
	PID->Iterm = 0;
	PID->Dterm = 0.0;
	PID->DtermUnfilt = 0.0;
	PID->output = 0;
	PID->lastInput = newInput;
	PID->initialized = 0;
}

void SetTunings(structPID *PID, float newKp, float newKi, float newKd)
{
	PID->Kp = newKp;
	PID->Ki = newKi * PID->sampleTime;
	PID->Kd = newKd / PID->sampleTime ;
}

void SetSampleTime(structPID *PID, float newSampleTime)
{
	if (newSampleTime > 0.0)
	{
		float ratio = (float) newSampleTime / PID->sampleTime;
		PID->Ki *= ratio;
		PID->Kd /= ratio;
		PID->sampleTime = (float) newSampleTime;
	}
}

void SetOutputLimits(structPID *PID, float minOutput, float maxOutput, float minIntegral, float maxIntegral)
{
	/* Check and set output limits */
	if (minOutput > maxOutput) {
		PID->minOutput = 0;
		PID->maxOutput = 0;
	}
	else {
		PID->minOutput = minOutput;
		PID->maxOutput = maxOutput;
	}
	/* Check and set integral limits */
	if (minIntegral > maxIntegral) {
		PID->minIntegral = 0;
		PID->maxIntegral = 0;
	}
	else {
		PID->minIntegral = minIntegral;
		PID->maxIntegral = maxIntegral;
	}
	/* Don't allow integral term to be >< the output term */
	if(PID->minIntegral < PID->minOutput ) PID->minIntegral = PID->minOutput;
	if(PID->maxIntegral > PID->maxOutput ) PID->maxIntegral = PID->maxOutput;
	/* Limit output value based on new limits */
	if (PID->output > PID->maxOutput) PID->output = PID->maxOutput;
	else if (PID->output < PID->minOutput) PID->output = PID->minOutput;
	/* Limit integral term based on new limits */
	if (PID->Iterm > PID->maxIntegral) PID->Iterm = PID->maxIntegral;
	else if (PID->Iterm < PID->minIntegral) PID->Iterm = PID->minIntegral;
}

void PID_Set_On(structPID *PID)
{
	PID->isOn = 1;
}

void PID_Set_Off(structPID *PID)
{
	PID->isOn = 0;
	PID->initialized = 0;
}

void PID_SetNotch(structPID *PID, NotchFilter *notchFilt)
{
	PID->hasNotchFilter = 1;
	PID->notchFilter = *notchFilt;
}
