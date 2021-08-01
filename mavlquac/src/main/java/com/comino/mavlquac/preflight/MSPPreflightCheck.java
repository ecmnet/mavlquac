package com.comino.mavlquac.preflight;

import org.mavlink.messages.ESTIMATOR_STATUS_FLAGS;

/****************************************************************************
*
*   Copyright (c) 2020 Eike Mansfeld ecm@gmx.de. All rights reserved.
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
****************************************************************************/

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.param.PX4Parameters;

public class MSPPreflightCheck {

	public static final int   OK     = 0;
	public static final int   INFO   = 1;
	public static final int   WARN   = 2;
	public static final int   FAILED = 3;

	private static MSPPreflightCheck instance;
	private IMAVMSPController control = null;

	private int                maxLevel  = OK;
	private DataModel model;

	public static MSPPreflightCheck getInstance(IMAVMSPController control) {
		if(instance == null)
			instance = new MSPPreflightCheck(control);
		return instance;
	}

	private MSPPreflightCheck(IMAVMSPController control) {
		this.control  = control;
		this.model    = control.getCurrentModel();

	}

	public int performArmCheck(PX4Parameters params) {

		maxLevel  = OK;
		
		MSPLogger.getInstance().writeLocalMsg("[msp] Performing preflight checks", MAV_SEVERITY.MAV_SEVERITY_NOTICE);

		// Is LIDAR available ?
		if(!model.sys.isSensorAvailable(Status.MSP_LIDAR_AVAILABILITY) && !model.sys.isSensorAvailable(Status.MSP_SONAR_AVAILABILITY))
			if(control.isSimulation())
			 checkFailed("[msp] No distance sensor available", WARN);
			else
			 checkFailed("[msp] No distance sensor available", WARN);

		// Is GPS with Fix available ?
        if(model.sys.isSensorAvailable(Status.MSP_GPS_AVAILABILITY) && model.gps.fixtype < 3)
			checkFailed("[msp] No GPS fix available ", INFO);

        // Is Vision available ?
        if(!model.sys.isSensorAvailable(Status.MSP_OPCV_AVAILABILITY)) {
        	if(control.isSimulation())
			  checkFailed("[msp] No odometry available ", WARN);
        	else
        	  checkFailed("[msp] No odometry available ", WARN);
        }

        // Is LPOS available
        if(!model.sys.isStatus(Status.MSP_LPOS_VALID))
     		checkFailed("[msp] LPOS not available", FAILED);
        
        if(Math.abs(model.state.l_z) > 0.3)
        	checkFailed("[msp] Local position not on ground", WARN);

        // Is GPOS available
        if(!model.sys.isStatus(Status.MSP_GPOS_VALID))
     		checkFailed("[msp] GPOS not available", WARN);

        // check Alt.amsl
     	if(Float.isNaN(model.hud.ag))
     		checkFailed("[msp] Altitude amsl not available", FAILED);

     	// Is IMU available ?
     	if(!model.sys.isSensorAvailable(Status.MSP_IMU_AVAILABILITY))
     		checkFailed("[msp] IMU not available", FAILED);

        // Check if kill switch is disabled
        if(params.getParam("CBRK_IO_SAFETY")!=null && params.getParam("CBRK_IO_SAFETY").value != 0)
     		checkFailed("[msp] IO SafetyBreaker set", INFO);

        // Check if RTL altitude is set
     	if(params.getParam("RTL_RETURN_ALT")!=null && params.getParam("RTL_RETURN_ALT").value != 1.0)
     		checkFailed("[msp] Return altitude not set to 1.0m", WARN);
     	
     	if(params.getParam("EKF2_AID_MASK")!=null && ((short)params.getParam("EKF2_AID_MASK").value & 0x0008) != 0x0008)
     		checkFailed("[msp] Vision not fused in EKF2", WARN);

//     	if(model.est.isFlagSet(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ACCEL_ERROR))
//     		checkFailed("[msp] EKF2 not ready", FAILED);
     		

     	// ...more

		return maxLevel;
	}

	private void checkFailed(String r, int level) {

		LogMessage m = new LogMessage();
		m.text = r;
		switch(level) {
		case INFO:
			m.severity = MAV_SEVERITY.MAV_SEVERITY_NOTICE;
			control.writeLogMessage(m);
			break;
		case WARN:
			m.severity = MAV_SEVERITY.MAV_SEVERITY_WARNING;
			control.writeLogMessage(m);
			break;
		case FAILED:
			if(control.isSimulation())
				level = MAV_SEVERITY.MAV_SEVERITY_WARNING;
			m.severity = MAV_SEVERITY.MAV_SEVERITY_CRITICAL;
			control.writeLogMessage(m);
			break;
		default:
			return;

		}
		if(level > maxLevel)
			maxLevel = level;

	}

}
