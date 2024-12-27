package com.comino.mavlquac.flighttask.offboard.publisher;

import com.comino.mavlquac.flighttask.types.VehicleState4D_F32;

public interface IMAVOffboardPublisher {
	
	public void  publishToVehicle(VehicleState4D_F32 setpoints, int type_mask ) ;

}
