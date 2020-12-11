package com.comino.mavlquac.inflight;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavutils.hw.HardwareAbstraction;


public class MSPInflightCheck {
	
	private static MSPInflightCheck instance;
	private IMAVMSPController control = null;

	private final DataModel model;
	private final HardwareAbstraction hw;
	
	private int   warnLevel = 0;
	
	private long  lastMessage_tms = 0;

	public MSPInflightCheck(IMAVMSPController control,HardwareAbstraction hw) {
		this.control  = control;
		this.model    = control.getCurrentModel();
		this.hw       = hw;
		
		reset();

	}
	
	public boolean performChecks() {
		
		if(model.sys.bat_state > 1)
			notifyCheck("PX4 battery warning.", MAV_SEVERITY.MAV_SEVERITY_WARNING);
		
		if(hw.getBatteryTemperature() > 45.0f)
			notifyCheck("MSP battery warning: Temerature too high.", MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
		
		if(model.battery.b0 < 13.0f)
			notifyCheck("MSP battery warning: Voltage low.", MAV_SEVERITY.MAV_SEVERITY_WARNING);
		
		if(model.battery.b0 < 12.5f)
			notifyCheck("MSP battery warning: Voltage critical low.", MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
		
		return warnLevel < MAV_SEVERITY.MAV_SEVERITY_WARNING;
	}
	
	private void notifyCheck(String message, int level) {
		if(level < warnLevel) warnLevel = level;
		if((System.currentTimeMillis() - lastMessage_tms) > 2000)
			control.writeLogMessage(new LogMessage(message,level));		
	}
	
	public void reset() {
		warnLevel = Integer.MAX_VALUE;
		lastMessage_tms = 0;
	}

}
