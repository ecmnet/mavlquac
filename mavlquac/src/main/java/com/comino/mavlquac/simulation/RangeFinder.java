package com.comino.mavlquac.simulation;

import java.util.concurrent.TimeUnit;

import org.mavlink.messages.MAV_SENSOR_TYPE;
import org.mavlink.messages.lquac.msg_distance_sensor;

import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.control.impl.MAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavutils.legacy.ExecutorService;

public class RangeFinder implements Runnable {
	
	private DataModel          model = null;
	private IMAVMSPController control = null;
	
	

	public RangeFinder(IMAVMSPController control2) {
		this.model = control2.getCurrentModel();
		this.control = control2;
		
		System.out.println("RangeFinder simulated");
		
		ExecutorService.get().scheduleAtFixedRate(this, 100, 20, TimeUnit.MILLISECONDS);
		
	}



	@Override
	public void run() {
		float distance = model.hud.ar + (float)(Math.random() - 0.5d)/30f + 0.12f;
		
		msg_distance_sensor msg = new msg_distance_sensor(1,1);
		msg.current_distance = (int)(distance * 100);
		msg.type = MAV_SENSOR_TYPE.MAV_SENSOR_TYPE_LIDAR;
		msg.min_distance = 20;
		msg.max_distance = 2500;
		control.sendMAVLinkMessage(msg);
		
	}
	

}
