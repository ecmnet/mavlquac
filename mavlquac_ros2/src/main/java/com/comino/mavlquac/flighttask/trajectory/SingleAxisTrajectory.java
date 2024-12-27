/****************************************************************************
 *
 *   Copyright (c) 2021 Eike Mansfeld ecm@gmx.de. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * This is a port to Java from minimum_jerkt_trajectories in
 * https://zenodo.org/record/5517791#.YW_kBS-21B1
 * 
 * The algorithm is described in the following paper: 
 * M.W. Mueller, M. Hehn, and R. D'Andrea, "A computationally efficient motion 
 * primitive for quadrocopter trajectory generation," 
 * IEEE Transactions on Robotics Volume 31, no.8, pages 1294-1310, 2015.
 * 
 * The paper may be downloaded from 
 * http://muellerlab.berkeley.edu/publications/
 *
 ****************************************************************************/

package com.comino.mavlquac.flighttask.trajectory;

public class SingleAxisTrajectory {

	private float  _p0, _v0, _a0;		                                // The initial state (position, velocity, acceleration)
	private float  _pf, _vf, _af;                                      // The goal state (position, velocity, acceleration)
	private boolean _posGoalDefined, _velGoalDefined, _accGoalDefined;  // The components of the goal state defined to be fixed (position, velocity, acceleration)
	private float _a, _b, _g;                                          // The three coefficients that define the trajectory
	private float _cost;                                               // The trajectory cost

	private float _minAcc;
	private float _maxAcc;

	private float  _totalTime;
	private boolean _isPlanned;

	private class accPeakTimes {										
		float t[] = { 0, 0};
		boolean initialised = false;
	}

	public accPeakTimes _accPeakTimes = new accPeakTimes();             // The times at which the acceleration has minimum/maximum


	public float getJerk(float t)  									// Returns the jerk at time t
	{  return _g  + _b*t  + (1.0f/2.0f)*_a*t*t; }

	public float getAcceleration(float t)                             // Returns the acceleration at time t
	{ return _a0 + _g*t  + (1.0f/2.0f)*_b*t*t  + (1.0f/6.0f)*_a*t*t*t;	}

	public float getVelocity(float t)								    // Returns the velocity at time t
	{ return _v0 + _a0*t + (1.0f/2.0f)*_g*t*t  + (1.0f/6.0f)*_b*t*t*t + (1.0f/24.0f)*_a*t*t*t*t; }


	public float getPosition(float t)								    // Returns the Position at time t
	{ return _p0 + _v0*t + (1.0f/2.0f)*_a0*t*t + (1.0f/6.0f)*_g*t*t*t + (1.0f/24.0f)*_b*t*t*t*t + (1.0f/120.0f)*_a*t*t*t*t*t; }

	public float getMinAcc() 
	{ return _minAcc; }

	public float getMaxAcc() 
	{ return _maxAcc; }

	public float getCosts() 
	{ return _cost; }

	public void setInitialState(float pos0, float vel0, float acc0)
	{ reset(); _p0=pos0; _v0=vel0; _a0=acc0;  }

	public void setTargetState(float posf, float velf, float accf)
	{ 
		if(Float.isFinite(posf)) setGoalPosition(posf); 
		if(Float.isFinite(velf)) setGoalVelocity(velf);
		if(Float.isFinite(accf)) setGoalAcceleration(accf);

	}
	
	public float getGoalPosition() {
		return getPosition(_totalTime);
	}
	
	public float getGoalVelocity() {
		return getVelocity(_totalTime);
	}
	
	public float getGoalAcceleration() {
		return getAcceleration(_totalTime);
	}

	public void setGoalPosition(float posf)    
	{ if(Float.isFinite(posf)) { _posGoalDefined = true; _pf = posf; } }

	public void setGoalVelocity(float velf)    
	{ if(Float.isFinite(velf)) { _velGoalDefined = true; _vf = velf; } }

	public void setGoalAcceleration(float accf)
	{ if(Float.isFinite(accf)) {  _accGoalDefined = true; _af = accf; } }

	public void reset() {
		_isPlanned = false;
		_totalTime = 0;
		_posGoalDefined = _velGoalDefined = _accGoalDefined = false;
		_cost = Float.MAX_VALUE;
		_accPeakTimes.initialised = false;
		_minAcc = 0;
		_maxAcc = 0;
		_p0 = _v0 = _a0 = 0;
		_pf = _vf = _af = 0;
		_a  =  _b =  _g = 0;

	}

	public float generateTrajectory(float Tf) {

		_totalTime = Tf;

		final float delta_a = _af - _a0;
		final float delta_v = _vf - _v0 - _a0*Tf;
		final float delta_p = _pf - _p0 - _v0*Tf - 0.5f*_a0*Tf*Tf;

		final float T2 = Tf*Tf;
		final float T3 = T2*Tf;
		final float T4 = T3*Tf;
		final float T5 = T4*Tf;

		//solve the trajectories, depending on what's constrained:
		if(_posGoalDefined && _velGoalDefined && _accGoalDefined)
		{
			_a = ( 60*T2*delta_a - 360*Tf*delta_v + 720* 1*delta_p)/T5;
			_b = (-24*T3*delta_a + 168*T2*delta_v - 360*Tf*delta_p)/T5;
			_g = (  3*T4*delta_a -  24*T3*delta_v +  60*T2*delta_p)/T5;
		}
		else if(_posGoalDefined && _velGoalDefined)
		{
			_a = (-120*Tf*delta_v + 320*   delta_p)/T5;
			_b = (  72*T2*delta_v - 200*Tf*delta_p)/T5;
			_g = ( -12*T3*delta_v +  40*T2*delta_p)/T5;
		}
		else if(_posGoalDefined && _accGoalDefined)
		{
			_a = (-15*T2*delta_a + 90*   delta_p)/(2*T5);
			_b = ( 15*T3*delta_a - 90*Tf*delta_p)/(2*T5);
			_g = (- 3*T4*delta_a + 30*T2*delta_p)/(2*T5);
		}
		else if(_velGoalDefined && _accGoalDefined)
		{
			_a = 0;
			_b = ( 6*Tf*delta_a - 12*   delta_v)/T3;
			_g = (-2*T2*delta_a +  6*Tf*delta_v)/T3;
		}
		else if(_posGoalDefined)
		{
			_a =  20*delta_p/T5;
			_b = -20*delta_p/T4;
			_g =  10*delta_p/T3;
		}
		else if(_velGoalDefined)
		{
			_a = 0;
			_b =-3*delta_v/T3;
			_g = 3*delta_v/T2;
		}
		else if(_accGoalDefined)
		{
			_a = 0;
			_b = 0;
			_g = delta_a/Tf;
		}
		else
		{ 
			_a = _b = _g = 0;
		}

		_cost =  _g*_g + _b*_g*Tf + _b*_b*T2/3.0f + _a*_g*T2/3.0f + _a*_b*T3/4.0f + _a*_a*T4/20.0f;
		_isPlanned = true;

		return (float)_totalTime;
	}

	public boolean isPlanned() {
		return _isPlanned;
	}
	
	public void setAsPlanned(boolean isPlanned) {
		this._isPlanned = isPlanned;
	}

	public float getTotalTime() {
		return _totalTime;
	}

	public void calcMinMaxAcc(float t1, float t2) {
		if(!_accPeakTimes.initialised)
		{
			//calculate the roots of the polynomial
			if(_a != 0)
			{//solve a quadratic for t
				float det = _b*_b - 2*_g*_a;
				if(det<0)
				{//no real roots
					_accPeakTimes.t[0] = 0;
					_accPeakTimes.t[1] = 0;
				}
				else
				{
					_accPeakTimes.t[0] = (-_b + (float)Math.sqrt(det))/_a;
					_accPeakTimes.t[1] = (-_b - (float)Math.sqrt(det))/_a; 
				}
			}
			else
			{//solve linear equation: _g + _b*t == 0:
				if(_b != 0) 
					_accPeakTimes.t[0] = -_g/_b;
				else         
					_accPeakTimes.t[0] = 0;
				_accPeakTimes.t[1] = 0;
			}

			_accPeakTimes.initialised = true;
		}

		//Evaluate the acceleration at the boundaries of the period:
		_minAcc = Math.min(getAcceleration(t1), getAcceleration(t2));
		_maxAcc = Math.max(getAcceleration(t1), getAcceleration(t2));

		//Evaluate at the maximum/minimum times:
		for(int i=0; i<2; i++)
		{
			if(_accPeakTimes.t[i] <= t1) continue;
			if(_accPeakTimes.t[i] >= t2) continue;

			_minAcc = Math.min(_minAcc, getAcceleration(_accPeakTimes.t[i]));
			_maxAcc = Math.max(_maxAcc, getAcceleration(_accPeakTimes.t[i]));
		}
	}

	public float getMaxJerkSquared(float t1, float t2)
	{
		float jMaxSqr = Math.max((float)Math.pow(getJerk(t1),2),(float)Math.pow(getJerk(t2),2));

		if(_a != 0)
		{
			float tMax = -1;
			tMax = -_b/_a;
			if(tMax>t1 && tMax<t2)	   
				jMaxSqr = Math.max((float)Math.pow(getJerk(tMax),2),jMaxSqr);
		}

		return jMaxSqr;
	}

	public float getParamAlpha() { return _a; }
	public float getParamBeta()  { return _b; }
	public float getParamGamma() { return _g; }
	public float getInitialAcc() { return _a0; }
	public float getInitialVel() { return _v0; }
	public float getInitialPos() { return _p0; }
	public float getCost()       { return _cost; }

	public void set(SingleAxisTrajectory planner) {

		_a  = planner._a;
		_b  = planner._b;
		_g  = planner._g;

		_p0 = planner._p0;
		_v0 = planner._v0;
		_a0 = planner._a0;

		_pf = planner._pf;
		_vf = planner._vf;
		_af = planner._af;

		_cost   = planner._cost;
		_minAcc = planner._minAcc;
		_maxAcc = planner._maxAcc;

		_totalTime = planner._totalTime;
		_isPlanned = planner._isPlanned;

		_accPeakTimes.initialised = planner._accPeakTimes.initialised;
		_accPeakTimes.t[0] = planner._accPeakTimes.t[0];
		_accPeakTimes.t[1] = planner._accPeakTimes.t[1];

		_posGoalDefined = planner._posGoalDefined;
		_velGoalDefined = planner._velGoalDefined;
		_accGoalDefined = planner._accGoalDefined;

	}


}
