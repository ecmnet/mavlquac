package com.comino.mavlquac.flighttask.offboard.publisher;

import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.lquac.msg_set_position_target_local_ned;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavlquac.flighttask.types.VehicleState4D_F32;

import georegression.struct.GeoTuple3D_F32;
import us.ihmc.log.LogTools;


public class MAVLinkOffboardPublisher implements IMAVOffboardPublisher {
	
	private final IMAVController                    control;
	private final msg_set_position_target_local_ned cmd;

	public MAVLinkOffboardPublisher(IMAVController control) {
		this.control = control;
		this.cmd     = new msg_set_position_target_local_ned(1,1);
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

}
