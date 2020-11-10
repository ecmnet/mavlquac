package com.comino.mavlquac.inflight;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavutils.hw.HardwareAbstraction;



public class MSPInflightCheck {
	
	private static final int   MAX_ERRORS = 20;

	private static final float MSP_MAX_TEMP = 80.0f;

	private static MSPInflightCheck instance;

	private IMAVMSPController control = null;
	private DataModel           model = null;
	private HardwareAbstraction    hw = null;

	private int error_count           = 0;

	public static MSPInflightCheck getInstance(IMAVMSPController control, PX4Parameters params, HardwareAbstraction hw) {
		if(instance == null)
			instance = new MSPInflightCheck(control, params, hw);
		return instance;
	}

	private MSPInflightCheck(IMAVMSPController control, PX4Parameters params, HardwareAbstraction hw) {
		this.control  = control;
		this.model    = control.getCurrentModel();
		this.hw       = hw;

		control.getStatusManager().addListener(Status.MSP_PARAMS_LOADED, (n) -> {
			if(n.isStatus(Status.MSP_PARAMS_LOADED)) {

			}		
		});

	}

	public void reset() {
		error_count = 0;
	}

	public void performChecks() {
		this.performChecks(null);
	}

	public void performChecks(InFlightFailureAction action) {


		// MSP Temperature check
		if(hw.getTemperature() > MSP_MAX_TEMP) {
			control.writeLogMessage(new LogMessage("Companion Temperature critical.", MAV_SEVERITY.MAV_SEVERITY_EMERGENCY));
			error_count++;
		}

		// Battery check


		// ....

		if(error_count > 0) {
			model.sys.setStatus(Status.MSP_ACTIVE, false);
		}
		
		if(error_count > MAX_ERRORS) {
			control.writeLogMessage(new LogMessage("InflightChecks failed. Emergency procedure triggered", MAV_SEVERITY.MAV_SEVERITY_EMERGENCY));
			if(action!=null)
			   action.run();
		}		
	}


}
