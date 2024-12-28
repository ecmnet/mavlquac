package com.comino.mavlquac.flighttask.offboard.publisher;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavlquac.flighttask.types.VehicleState4D_F32;

public class ROS2XrceOffboardPublisher extends MAVOffboardPublisher {

	public ROS2XrceOffboardPublisher(IMAVController control) {
		super(control);
	}

	@Override
	public void publishToVehicle(VehicleState4D_F32 setpoints, int type_mask) {
	
	}

	@Override
	public void enableOffboard() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void switchToLoiter() {
		// TODO Auto-generated method stub
		
	}



}
