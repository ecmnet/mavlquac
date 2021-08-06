package com.comino.mavlquac.inflight;

import org.mavlink.messages.ESTIMATOR_STATUS_FLAGS;
import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavcom.status.StatusManager;
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
	private long  level_change_tms;


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
		lastMessage_tms  = 0;
		level_change_tms = 0;
	}

	public void run() {

		readParameters();

		result = performChecks();

		if(hw.getArchId() != HardwareAbstraction.UPBOARD) 
			return;

		if(model.sys.isStatus(Status.MSP_CONNECTED)) {

			switch(result) {
			case MSPInflightCheck.EMERGENCY:
				UpLEDControl.flash(UpLEDControl.RED, 50);
				break;
			case MSPInflightCheck.WARN:
				UpLEDControl.flash(UpLEDControl.YELLOW, 10);
				break;
			case MSPInflightCheck.INIT:
				UpLEDControl.flash(UpLEDControl.ORANGE, 30);
				break;
			default:
				UpLEDControl.flash(UpLEDControl.GREEN, 10);
			}
			return;
		}

		UpLEDControl.flash(UpLEDControl.ORANGE, 10);	

		if((System.currentTimeMillis() - level_change_tms) > 10000) {
			reset();
		}
	}


	private void readParameters() {

		PX4Parameters params = PX4Parameters.getInstance();
		if(params==null)
			return;
	}

	private int performChecks() {
		
		if(control.isSimulation())
			return OK;
	
		// Set to init phase until CV is initialized the first time
		if(model.sys.t_boot_ms < 20000 && !model.sys.isSensorAvailable(Status.MSP_OPCV_AVAILABILITY)) {
			reset();
			return INIT;
		}

		if(!model.sys.isStatus(Status.MSP_ARMED)) {
			reset();
			return OK;
		}

		if(!model.est.isFlagSet(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL)) 
			notifyCheck("[msp] EKF2 Position estimation failure.", MAV_SEVERITY.MAV_SEVERITY_ERROR);

		if(Math.abs(model.state.l_z - model.vision.z) > 0.3f && model.sys.isSensorAvailable(Status.MSP_OPCV_AVAILABILITY)) {
			notifyCheck("[msp] EKF2 altitude not aligned to EV.", MAV_SEVERITY.MAV_SEVERITY_WARNING);
		}

		if(model.sys.bat_state > 1)
			notifyCheck(" [msp] PX4 battery warning.", MAV_SEVERITY.MAV_SEVERITY_WARNING);

		if(hw.getBatteryTemperature() > 45.0f )
			notifyCheck("[msp] battery warning: Temperature too high.", MAV_SEVERITY.MAV_SEVERITY_CRITICAL);

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
		if(level < warnLevel) { 
			warnLevel = level;
			level_change_tms = System.currentTimeMillis();
		}
		System.out.println("Inflight: "+message);
		if((System.currentTimeMillis() - lastMessage_tms) > 5000 && message != null) {
			lastMessage_tms = System.currentTimeMillis();
			control.writeLogMessage(new LogMessage(message,level));	
		}
		if(warnLevel > WARN)
			model.sys.setStatus(Status.MSP_READY_FOR_FLIGHT, false);
	}

}
