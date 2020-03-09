package com.comino.mavlquac.preflight;

import java.util.ArrayList;
import java.util.List;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.param.PX4ParamReader;

public class MSPPreflightCheck {

	public static final int   OK     = 0;
	public static final int   WARN   = 1;
	public static final int   FAILED = 2;

	private static MSPPreflightCheck instance;
	private IMAVMSPController control = null;

	private int                maxLevel  = OK;

	public static MSPPreflightCheck getInstance(IMAVMSPController control) {
		if(instance == null)
			instance = new MSPPreflightCheck(control);
		return instance;
	}

	private MSPPreflightCheck(IMAVMSPController control) {
		this.control  = control;

	}

	public int performCheck(DataModel model, PX4ParamReader params) {

		maxLevel  = OK;

		// LIDAR CHECK
		if(!model.sys.isSensorAvailable(Status.MSP_LIDAR_AVAILABILITY) && !model.sys.isSensorAvailable(Status.MSP_SONAR_AVAILABILITY))
			checkFailed("[msp] No distance sensor available", WARN);

		// Is GPS with Fix available ?
        if(model.sys.isSensorAvailable(Status.MSP_GPS_AVAILABILITY) && model.gps.fixtype < 3)
						checkFailed("[msp] No GPS fix available ", WARN);

        // Is GPOS available
        if(!model.sys.isStatus(Status.MSP_LPOS_VALID))
     		checkFailed("[msp] LPOS not available", FAILED);

        // check Alt.amsl
     	if(Float.isNaN(model.hud.ag))
     		checkFailed("Altitude amsl not available", WARN);


     	// Is IMU available ?
     	if(!model.sys.isSensorAvailable(Status.MSP_IMU_AVAILABILITY))
     		checkFailed("IMU not available", FAILED);

		return maxLevel;
	}

	private void checkFailed(String r, int level) {
		LogMessage m = new LogMessage();
		m.text = r;
		switch(level) {
		case WARN:
			m.severity = MAV_SEVERITY.MAV_SEVERITY_WARNING;
			control.writeLogMessage(m);
			break;
		case FAILED:
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
