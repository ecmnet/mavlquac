package com.comino.mavlquac.flighttask.offboard.publisher;

import org.mavlink.messages.lquac.msg_msp_trajectory;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavlquac.flighttask.trajectory.Rapid4DTrajectoryGenerator;
import com.comino.mavlquac.flighttask.types.VehicleState4D_F32;

public abstract class MAVOffboardPublisher {
	
	protected final IMAVController  control;
	protected final msg_msp_trajectory traj = new msg_msp_trajectory(2,1);
	
	public MAVOffboardPublisher(IMAVController control) {
		this.control = control;
	}
	
	public abstract void  publishToVehicle(VehicleState4D_F32 setpoints, int type_mask );
	
	public abstract void  enableOffboard();
	public abstract void  switchToLoiter();
	
	public void publishTrajectoryToGCL(Rapid4DTrajectoryGenerator xyzExecutor,float elapsed_time) {
		
		if(elapsed_time >= xyzExecutor.geTimeToFinish()) {
			traj.ls = xyzExecutor.geTimeToFinish();
			control.sendMAVLinkMessage(traj);
			return;
		}

		traj.ls = xyzExecutor.geTimeToFinish();
		traj.fs = elapsed_time;
		traj.ax = (float)xyzExecutor.getAxisParamAlpha(0);
		traj.ay = (float)xyzExecutor.getAxisParamAlpha(1);
		traj.az = (float)xyzExecutor.getAxisParamAlpha(2);

		traj.bx = (float)xyzExecutor.getAxisParamBeta(0);
		traj.by = (float)xyzExecutor.getAxisParamBeta(1);
		traj.bz = (float)xyzExecutor.getAxisParamBeta(2);

		traj.gx = (float)xyzExecutor.getAxisParamGamma(0);
		traj.gy = (float)xyzExecutor.getAxisParamGamma(1);
		traj.gz = (float)xyzExecutor.getAxisParamGamma(2);

		traj.sx = (float)xyzExecutor.getInitialPosition(0);
		traj.sy = (float)xyzExecutor.getInitialPosition(1);
		traj.sz = (float)xyzExecutor.getInitialPosition(2);

		traj.svx = (float)xyzExecutor.getInitialVelocity(0);
		traj.svy = (float)xyzExecutor.getInitialVelocity(1);
		traj.svz = (float)xyzExecutor.getInitialVelocity(2);

		traj.sax = (float)xyzExecutor.getInitialAcceleration(0);
		traj.say = (float)xyzExecutor.getInitialAcceleration(1);
		traj.saz = (float)xyzExecutor.getInitialAcceleration(2);

		traj.tms = DataModel.getSynchronizedPX4Time_us();
		
		control.sendMAVLinkMessage(traj);

	}

}
