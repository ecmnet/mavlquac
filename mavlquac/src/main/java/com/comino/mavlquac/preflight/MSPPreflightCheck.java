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

		// Is LIDAR available ?
		if(!model.sys.isSensorAvailable(Status.MSP_LIDAR_AVAILABILITY) && !model.sys.isSensorAvailable(Status.MSP_SONAR_AVAILABILITY))
			checkFailed("[msp] No distance sensor available", WARN);

		// Is GPS with Fix available ?
        if(model.sys.isSensorAvailable(Status.MSP_GPS_AVAILABILITY) && model.gps.fixtype < 3)
			checkFailed("[msp] No GPS fix available ", WARN);

        // Is Flow available ?
        if(!model.sys.isSensorAvailable(Status.MSP_PIX4FLOW_AVAILABILITY))
			checkFailed("[msp] No Flow data available ", WARN);

        // Is Vision available ?
        if(!model.sys.isSensorAvailable(Status.MSP_OPCV_AVAILABILITY))
			checkFailed("[msp] No vision data available ", WARN);


        // Is LPOS available
        if(!model.sys.isStatus(Status.MSP_LPOS_VALID))
     		checkFailed("[msp] LPOS not available", FAILED);

        // Is GPOS available
        if(!model.sys.isStatus(Status.MSP_GPOS_VALID))
     		checkFailed("[msp] LPOS not available", WARN);

        // check Alt.amsl
     	if(Float.isNaN(model.hud.ag))
     		checkFailed("[msp] Altitude amsl not available", WARN);

     	// Is IMU available ?
     	if(!model.sys.isSensorAvailable(Status.MSP_IMU_AVAILABILITY))
     		checkFailed("[msp] IMU not available", FAILED);

        // Is relative altitude < 20 ?
     	if(model.hud.ar > 0.2)
     		checkFailed("[msp] Vehicle not on ground", FAILED);

        // Check if kill switch is disabled
        if(params.getParam("CBRK_IO_SAFETY")!=null && params.getParam("CBRK_IO_SAFETY").value != 0)
     		checkFailed("[msp] IO SafetyBreaker set", WARN);

        // Check if RTL altitude is set
     	if(params.getParam("RTL_RETURN_ALT")!=null && params.getParam("RTL_RETURN_ALT").value != 1.0)
     		checkFailed("[msp] Return altitude not set to 1.0m", WARN);


     	// ...more

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
