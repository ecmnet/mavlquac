package com.comino.mavlquac.commander;

import com.comino.mavcom.control.IMAVController;

public class MSPCommander {
	
	private static MSPCommander instance;

	public static MSPCommander getInstance(IMAVController control) {
		if(instance == null)
			instance = new MSPCommander(control);
		return instance;
	}
	
	public static MSPCommander getInstance() {
		return instance;
	}
	
	private final IMAVController control;
	
	private MSPCommander(IMAVController control) {
		this.control = control;
	}
	

}
