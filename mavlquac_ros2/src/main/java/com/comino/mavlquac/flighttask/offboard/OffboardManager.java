package com.comino.mavlquac.flighttask.offboard;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavlquac.flighttask.offboard.publisher.MAVLinkOffboardPublisher;
import com.comino.mavlquac.flighttask.offboard.publisher.MAVOffboardPublisher;
import com.comino.mavlquac.flighttask.trajectory.Rapid4DTrajectoryGenerator;
import com.comino.mavlquac.flighttask.types.VehicleState4D_F32;
import com.comino.mavlquac.flighttask.types.VehicleStateCurrent4D_F32;
import com.comino.mavutils.workqueue.WorkQueue;

import georegression.struct.point.Vector4D_F32;
import us.ihmc.log.LogTools;

public class OffboardManager {
	
	private static final int     UPDATE_RATE  = 25;	
	private static final int     IDLE_MSGS    = 5;
	

	private static OffboardManager instance;
	
	private final DataModel             model;
	private final IMAVController        control;
	private final Worker                worker;
	private final WorkQueue             wq;
	private final MAVOffboardPublisher  pub;
	
	private final VehicleStateCurrent4D_F32 current;
	
	public static OffboardManager getInstance(IMAVController control) {
		if(instance == null)
			instance = new OffboardManager(control);
		return instance;
	}
	
	public static OffboardManager getInstance() {
		return instance;
	}

	private OffboardManager(IMAVController control) {
		this.model   = control.getCurrentModel();
		this.control = control;
		this.wq      = WorkQueue.getInstance();
		
		this.worker  = new Worker( );
		this.pub     = new MAVLinkOffboardPublisher(control);
		this.current = new VehicleStateCurrent4D_F32(model);
	}
	
	public void moveTo(Vector4D_F32 target, float timeToFinish) {
		current.update();
		var planner = new Rapid4DTrajectoryGenerator();
		planner.setInitialState(current);
		planner.setGoal(target);
		planner.generate(timeToFinish);
		worker.start(planner);
	}
	
	private class Worker implements Runnable {
		
		private Rapid4DTrajectoryGenerator  xyzwExecutor;
		
		private int   offboard_worker_id    = 0;
		private int   mask                  = 0;
		private float t_section_elapsed_s   = 0;
		private long  t_section_start_ms    = 0;
		
		private final VehicleState4D_F32   setpoints;
		private long    counter;
		
		public Worker() {
			
			this.setpoints = new VehicleState4D_F32();
		
		}
		
		public void start(Rapid4DTrajectoryGenerator xyzw) {
			
			if(!control.isConnected())
				return;
			
			if(model.sys.isStatus(Status.MSP_LANDED)) {
				control.writeLogMessage(new LogMessage("[msp] Landed. Offboard control rejected.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
				return;
			}
			
		    this.counter = 0;
			this.xyzwExecutor = xyzw;
			
			if(!wq.isInQueue("NP", offboard_worker_id)) {
				offboard_worker_id = wq.addCyclicTask("NP", UPDATE_RATE, this);
			}
		}
		
		public void stop( ) {
			this.counter = 0;
			wq.removeTask("NP", offboard_worker_id);
		}
		

		@Override
		public void run() {
			
			if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) && !model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD)) {
				this.stop();
				control.writeLogMessage(new LogMessage("[msp] Offboard externally stopped.", MAV_SEVERITY.MAV_SEVERITY_INFO));
				LogTools.info("Externally stopped");
				return;
			}
			
			current.update();
			
			counter++;
			if(counter < IDLE_MSGS) {
				// Send some idle setpoints before enabling offboard
				t_section_start_ms = System.currentTimeMillis();
				xyzwExecutor.getStateAt(0, setpoints);
				pub.publishToVehicle(setpoints, mask);
				return;
			}
			
			if(counter == IDLE_MSGS) {
				// Enable offboard mode
				pub.enableOffboard();
				t_section_start_ms = System.currentTimeMillis();
			}
			
			t_section_elapsed_s = (System.currentTimeMillis() - t_section_start_ms ) * 1e-3f;
			if(t_section_elapsed_s > xyzwExecutor.geTimeToFinish()) {
				// TODO: Switch to next section
				pub.switchToLoiter();
				this.stop();
				return;
			} 
	
			xyzwExecutor.getStateAt(t_section_elapsed_s, setpoints);
			pub.publishToVehicle(setpoints, mask);	
			pub.publishTrajectoryToGCL(xyzwExecutor, t_section_elapsed_s);
			
		}
		
	}

}
