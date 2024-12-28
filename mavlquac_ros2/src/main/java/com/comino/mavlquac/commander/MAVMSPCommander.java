package com.comino.mavlquac.commander;

import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.lquac.msg_msp_command;

import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavlquac.flighttask.offboard.OffboardManager;

import georegression.struct.point.Vector4D_F32;

public class MAVMSPCommander {
	
	private static MAVMSPCommander instance;
	private final  OffboardManager offboard;

	public static MAVMSPCommander getInstance(IMAVMSPController control) {
		if(instance == null)
			instance = new MAVMSPCommander(control);
		return instance;
	}
	
	public static MAVMSPCommander getInstance() {
		return instance;
	}
	
	private final IMAVMSPController control;
	
	private MAVMSPCommander(IMAVMSPController control) {
		this.control = control;
		this.offboard = OffboardManager.getInstance();
		
		this.registerMSPCommands();
		
	}
	
	private void registerMSPCommands() {
		control.registerListener(msg_msp_command.class,(o) -> {
			final msg_msp_command cmd = (msg_msp_command)o;
			switch(cmd.command) {
			   case MSP_CMD.MSP_CMD_OFFBOARD_SETLOCALPOS:
				   offboard.moveTo(new Vector4D_F32(cmd.param1, cmd.param2, Float.NaN, Float.NaN), 5.0f);
				   break;
			    
			}
		});
	}
	

}
