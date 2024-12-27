package com.comino.mavlquac.flighttask.types;

import com.comino.mavcom.model.DataModel;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point4D_F32;
import us.ihmc.log.LogTools;

public class VehicleStateCurrent4D_F32 extends VehicleState4D_F32 {

	protected final GeoTuple4D_F32<?> current_pos_setpoint = new Point4D_F32(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
	protected final GeoTuple4D_F32<?> current_vel_setpoint = new Point4D_F32(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
	protected final GeoTuple4D_F32<?> current_acc_setpoint = new Point4D_F32(Float.NaN, Float.NaN, Float.NaN, Float.NaN);


	private final DataModel model;

	public VehicleStateCurrent4D_F32() {
		this.model =null;
	}

	public VehicleStateCurrent4D_F32(DataModel model) {
		this.model = model;
		update();
	}

	public GeoTuple4D_F32<?> sep() {
		return current_pos_setpoint;
	}

	public GeoTuple4D_F32<?> sev() {
		return current_vel_setpoint;
	}
	
	public GeoTuple4D_F32<?> sea() {
		return current_acc_setpoint;
	}


	public void update() {

		if(model!=null) {
			
			pos.setTo(model.state.l_x,  model.state.l_y,  model.state.l_z,  model.attitude.y);
			vel.setTo(model.state.l_vx, model.state.l_vy, model.state.l_vz, model.attitude.yr);
			acc.setTo(0,0,0,0);
			
			maskZero(vel,0.1f);
		
			current_pos_setpoint.setTo(model.target_state.l_x,  model.target_state.l_y,  model.target_state.l_z,  model.attitude.sy);
			current_vel_setpoint.setTo(model.target_state.l_vx, model.target_state.l_vy, model.target_state.l_vz, model.attitude.syr);
			current_acc_setpoint.setTo(model.target_state.l_ax, model.target_state.l_ay, model.target_state.l_az,0.0f);

			
		} else {
			LogTools.error("Current state not updated. Model is NULL");
		}
	}
	
	private void maskZero(GeoTuple4D_F32<?> in, float limit) {
		if(Math.abs(in.x) < limit) in.x = 0f;
		if(Math.abs(in.y) < limit) in.y = 0f;
		if(Math.abs(in.z) < limit) in.z = 0f;
	}
}
