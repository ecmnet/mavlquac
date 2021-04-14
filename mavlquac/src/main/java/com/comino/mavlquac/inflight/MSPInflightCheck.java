package com.comino.mavlquac.inflight;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavutils.hw.HardwareAbstraction;
import com.comino.mavutils.hw.upboard.UpLEDControl;


public class MSPInflightCheck implements Runnable {

	public static final int        OK   = 0;
	public static final int        WARN = 1;
	public static final int   EMERGENCY = 2;
	public static final int        INIT = 9;

	private IMAVMSPController control = null;

	private final DataModel model;
	private final HardwareAbstraction hw;

	private int   warnLevel = Integer.MAX_VALUE;
	private int   result    = OK;

	private long  lastMessage_tms = 0;
	private long  tms = 0;

	public MSPInflightCheck(IMAVMSPController control,HardwareAbstraction hw) {
		this.control  = control;
		this.model    = control.getCurrentModel();
		this.hw       = hw;
		reset();
		
		if(hw.getArchId() == HardwareAbstraction.UPBOARD)
			UpLEDControl.clear();
	}

	public int getCheckResult() {
		return result;
	}

	public void reset() {
		warnLevel = Integer.MAX_VALUE;
		lastMessage_tms = 0;
	}

	public void run() {
		result = performChecks();
		
		if(hw.getArchId() != HardwareAbstraction.UPBOARD) 
			return;

		if(model.sys.isStatus(Status.MSP_ACTIVE)) {

			switch(result) {
			case MSPInflightCheck.EMERGENCY:
				UpLEDControl.flash("red", 50);
				break;
			case MSPInflightCheck.WARN:
				UpLEDControl.flash("yellow", 10);
				break;
			case MSPInflightCheck.INIT:
				UpLEDControl.flash("yellow", 30);
				break;
			default:
				UpLEDControl.flash("green", 10);
			}
			return;
		}
		
		UpLEDControl.flash("yellow", 10);	
	}

	private int performChecks() {
		

		// Set to init phase until CV is initialized the first time
		if(model.sys.t_boot_ms < 20000 && !model.sys.isSensorAvailable(Status.MSP_OPCV_AVAILABILITY)) {
			reset();
			return INIT;
		}

		if(!model.sys.isStatus(Status.MSP_ARMED)) {
			reset();
			return OK;
		}



		if(model.sys.bat_state > 1)
			notifyCheck("PX4 battery warning.", MAV_SEVERITY.MAV_SEVERITY_WARNING);

		if(hw.getBatteryTemperature() > 45.0f )
			notifyCheck("MSP battery warning: Temperature too high.", MAV_SEVERITY.MAV_SEVERITY_CRITICAL);

		if(model.battery.b0 < 12.3f)
			notifyCheck("MSP battery warning: Voltage low.", MAV_SEVERITY.MAV_SEVERITY_WARNING);

		if(model.battery.b0 < 12.0f)
			notifyCheck("MSP battery warning: Voltage critical low.", MAV_SEVERITY.MAV_SEVERITY_CRITICAL);


		if(!model.sys.isSensorAvailable(Status.MSP_PIX4FLOW_AVAILABILITY))
			notifyCheck(null, MAV_SEVERITY.MAV_SEVERITY_WARNING);

		if(!model.sys.isSensorAvailable(Status.MSP_LIDAR_AVAILABILITY))
			notifyCheck(null, MAV_SEVERITY.MAV_SEVERITY_WARNING);

		// Todo: Register actions and executes them

		if(warnLevel <=  MAV_SEVERITY.MAV_SEVERITY_CRITICAL)
			return EMERGENCY;
		if(warnLevel <=  MAV_SEVERITY.MAV_SEVERITY_WARNING)
			return WARN;
		return OK;
	}


	private void notifyCheck(String message, int level) {
		if(level < warnLevel) warnLevel = level;
		if((System.currentTimeMillis() - lastMessage_tms) > 2000 && message != null) {
			lastMessage_tms = System.currentTimeMillis();
			control.writeLogMessage(new LogMessage(message,level));	
		}
	}

}
