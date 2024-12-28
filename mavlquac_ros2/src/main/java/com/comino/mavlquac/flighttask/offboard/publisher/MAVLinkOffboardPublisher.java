package com.comino.mavlquac.flighttask.offboard.publisher;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.MAV_MODE_FLAG;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.lquac.msg_set_position_target_local_ned;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavlquac.flighttask.types.VehicleState4D_F32;

import georegression.struct.GeoTuple3D_F32;
import us.ihmc.log.LogTools;


public class MAVLinkOffboardPublisher extends MAVOffboardPublisher {

	private final msg_set_position_target_local_ned cmd;

	public MAVLinkOffboardPublisher(IMAVController control) {
		super(control);
		this.cmd  = new msg_set_position_target_local_ned(1,1);
	}

	@Override
	public void  publishToVehicle(VehicleState4D_F32 setpoints, int type_mask ) {

		cmd.type_mask        = type_mask;
		cmd.target_system    = 1;
		cmd.target_component = 1;

		cmd.x   = setpoints.pos().x;
		cmd.y   = setpoints.pos().y;
		cmd.z   = setpoints.pos().z;

		cmd.vx  = setpoints.vel().x;
		cmd.vy  = setpoints.vel().y;
		cmd.vz  = setpoints.vel().z;

		cmd.afx = setpoints.acc().x; 
		cmd.afy = setpoints.acc().y;
		cmd.afz = setpoints.acc().z;

		cmd.yaw      = setpoints.pos().w;
		cmd.yaw_rate = setpoints.vel().w;

		cmd.time_boot_ms = DataModel.getSynchronizedPX4Time_us()/1000L;
		cmd.isValid  = true;
		cmd.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
		control.sendMAVLinkMessage(cmd);
		//	System.out.println(cmd);

	}

	@Override
	public void enableOffboard() {
		
		if(control.getCurrentModel().sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD))
			return;

		control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd, result) -> {
			if(result != MAV_RESULT.MAV_RESULT_ACCEPTED) {
				control.writeLogMessage(new LogMessage("[msp] Switching to offboard failed ("+result+").", MAV_SEVERITY.MAV_SEVERITY_WARNING));
			} else {
				LogTools.info("Started successfully");
			}
		}, MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
				MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0 );
	}

	@Override
	public void switchToLoiter() {
		
		if(control.getCurrentModel().sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER))
			return;

		control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd,result) -> {
			if(result != MAV_RESULT.MAV_RESULT_ACCEPTED) 
				control.writeLogMessage(new LogMessage("Switching to hold failed. Continue offboard",MAV_SEVERITY.MAV_SEVERITY_DEBUG));
			else {
				LogTools.info("Stopped. Switched to Loiter");
			}
		},MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
				MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
	}

}
