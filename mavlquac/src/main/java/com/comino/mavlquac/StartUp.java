/****************************************************************************
 *
 *   Copyright (c) 2021 Eike Mansfeld ecm@gmx.de. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

package com.comino.mavlquac;

import java.net.InetSocketAddress;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

import org.mavlink.messages.ESTIMATOR_STATUS_FLAGS;
import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.lquac.msg_debug_vect;
import org.mavlink.messages.lquac.msg_msp_command;
import org.mavlink.messages.lquac.msg_msp_micro_grid;
import org.mavlink.messages.lquac.msg_msp_status;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.control.impl.MAVController;
import com.comino.mavcom.control.impl.MAVProxyController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.mavlink.IMAVLinkListener;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcontrol.commander.MSPCommander;
import com.comino.mavlquac.console.Console;
import com.comino.mavlquac.dispatcher.MAVLinkDispatcher;
import com.comino.mavlquac.inflight.MSPInflightCheck;
import com.comino.mavlquac.preflight.MSPPreflightCheck;
import com.comino.mavlquac.simulation.SensorSimulation;
import com.comino.mavodometry.estimators.MAVR200DepthEstimator;
import com.comino.mavodometry.estimators.MAVR200PositionEstimator;
import com.comino.mavodometry.estimators.MAVT265PositionEstimator;
import com.comino.mavodometry.video.impl.DefaultOverlayListener;
import com.comino.mavodometry.video.impl.HttpMJPEGHandler;
import com.comino.mavutils.hw.HardwareAbstraction;
import com.comino.mavutils.hw.upboard.UpLEDControl;
import com.comino.mavutils.legacy.ExecutorService;
import com.comino.mavutils.workqueue.WorkQueue;
import com.sun.net.httpserver.HttpServer;

import boofcv.concurrency.BoofConcurrency;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class StartUp  {
	
	private final WorkQueue wq = WorkQueue.getInstance();

	private static final int WIDTH  = 320;
	private static final int HEIGHT = 240;


	IMAVMSPController    control    = null;
	MSPConfig	          config    = null;
	MAVLinkDispatcher    dispatcher = null;

	private HttpMJPEGHandler<Planar<GrayU8>> streamer = null;

	private MSPCommander  commander = null;
	private DataModel     model     = null;

	private MAVR200PositionEstimator vision = null;
	private MAVT265PositionEstimator pose = null;
	private MAVR200DepthEstimator depth = null;
	
	private int mode;

	private MSPLogger logger;
	private PX4Parameters params;
	

	private final HardwareAbstraction hw;
	private final MSPInflightCheck inflightCheck;


	public StartUp(String[] args) {

		//		new JetsonNanoInferenceTest();
		
		addShutdownHook();
		
		this.hw = HardwareAbstraction.instance();

		Runtime.getRuntime().addShutdownHook(new Thread() 
		{ 
			public void run() 
			{ 
				System.out.println("MSPControlService shutdown...");

				control.close();

				if(pose!=null) pose.stop();
				if(depth!=null) depth.stop();
				
				wq.stop();

			} 
		}); 

		BoofConcurrency.setMaxThreads(4);

		ExecutorService.create();

		if(args.length != 0) {
			if(args[0].contains("SIM"))
				mode = MAVController.MODE_SITL;
			else
				if(args[0].contains("SERVER"))
					mode = MAVController.MODE_SERVER;
				else
					if(args[0].contains("USB"))
						mode = MAVController.MODE_USB;
					else  
						mode = MAVController.MODE_NORMAL;
		}

		switch(mode) {

		case  MAVController.MODE_NORMAL:

			config  = MSPConfig.getInstance("/home/lquac/","msp.properties");
			control = new MAVProxyController(MAVController.MODE_NORMAL);
			System.out.println("MSPControlService (LQUAC build) version "+config.getVersion());

			break;

		case MAVController.MODE_SERVER:

			config  = MSPConfig.getInstance("/home/ecm/lquac/","msp.properties");
			control = new MAVProxyController(MAVController.MODE_SERVER);
			System.out.println("MSPControlService (LQUAC JetsonNano) version "+config.getVersion()+" Mode = "+mode);
			control.getCurrentModel().clear();
			break;

		default:

			config  = MSPConfig.getInstance(System.getProperty("user.home")+"/","msp.properties");
			control = new MAVProxyController(mode);
			System.out.println("MSPControlService (LQUAC simulation) version "+config.getVersion()+" Mode = "+mode);
		}

		logger = MSPLogger.getInstance(control);
		logger.enableDebugMessages(true);
		
		inflightCheck = new MSPInflightCheck(control, hw);
		dispatcher = new MAVLinkDispatcher(control, config, hw);

		control.start();
		model = control.getCurrentModel();

		params = PX4Parameters.getInstance(control);

		commander = new MSPCommander(control,config);

		
		// Request parameter refresh when reconnected on ground
		control.getStatusManager().addListener(Status.MSP_CONNECTED, (n) -> {
			if(n.isStatus(Status.MSP_CONNECTED) && !model.sys.isStatus(Status.MSP_ARMED))
				params.requestRefresh();
		});

		// Set initial PX4 Parameters
		control.getStatusManager().addListener(Status.MSP_PARAMS_LOADED, (n) -> {
			if(n.isStatus(Status.MSP_PARAMS_LOADED)) {			
				params.sendParameter("RTL_DESCEND_ALT", 1.0f);
				params.sendParameter("RTL_RETURN_ALT", 1.0f);
				params.sendParameter("NAV_MC_ALT_RAD", 0.05f);

				if(control.isSimulation()) {
					params.sendParameter("COM_RC_OVERRIDE", 0);
				}
			}

		});


		// Preflight checks when arming
		control.getStatusManager().addListener(StatusManager.TYPE_PX4_STATUS,
				Status.MSP_ARMED, StatusManager.EDGE_RISING, (n) -> {
					if(MSPPreflightCheck.getInstance(control).performArmCheck(params)==MSPPreflightCheck.FAILED) {
						control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM,0 );
						logger.writeLocalMsg("[msp] Disarmed. PreFlight health check failed",
								MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
					} 
				});

		// ?????
		control.getStatusManager().addListener(StatusManager.TYPE_MSP_SERVICES,
				Status.MSP_SLAM_AVAILABILITY, StatusManager.EDGE_FALLING, (n) -> {
					logger.writeLocalMsg("[msp] SLAM disabled", MAV_SEVERITY.MAV_SEVERITY_INFO);
				});




		logger.writeLocalMsg("MAVProxy "+config.getVersion()+" loaded");
		
		// Start services if required

		try {	Thread.sleep(200); } catch(Exception e) { }


		if(config.getBoolProperty("vision_enabled", "true")) {

			//				if(config.getBoolProperty("vision_highres", "false"))
			//					info = new RealSenseInfo(640,480, RealSenseInfo.MODE_RGB);
			//				else
			//					info = new RealSenseInfo(320,240, RealSenseInfo.MODE_RGB);

			streamer = new HttpMJPEGHandler<Planar<GrayU8>>(WIDTH,HEIGHT, control.getCurrentModel());

			//*** R200 Odometry, Mapping, Groundtruth via T265


			//	vision = new MAVVisualPositionEstimatorVO(info, control, config, streamer);
			//				vision = new MAVVisualPositionEstimatorVIO(info, control, config, streamer);
			//				vision.registerDetector(new FwDirectDepthDetector(control,config,commander.getMap(),streamer));
			//				pose = new StreamRealSensePose(control, StreamRealSensePose.GROUNDTRUTH_MODE, streamer);

			//				if(vision!=null && !vision.isRunning()) {
			//					vision.start();
			//				    pose.start();
			//				}


			//*** T265 odometry

			try {

				pose = new MAVT265PositionEstimator(control, config, WIDTH,HEIGHT, MAVT265PositionEstimator.LPOS_ODO_MODE_NED, streamer);
				pose.start();

			} catch(UnsatisfiedLinkError | Exception e ) {
				System.out.println("! No pose estimation available");

				if(!control.isSimulation())
					e.printStackTrace();
			}


			//*** R200 Depth estimation

			try {

				depth = new MAVR200DepthEstimator(control, commander.getAutopilot(), commander.getAutopilot().getMap(),config, WIDTH,HEIGHT, streamer);
				depth.enableStream(true);
				depth.start();

			} catch(UnsatisfiedLinkError | Exception e ) {
				System.out.println("! No depth estimation available");

				if(!control.isSimulation())
					e.printStackTrace();
			}
			//***********

			try {


				final HttpServer server;

				server = HttpServer.create(new InetSocketAddress(8080),2);
				server.createContext("/mjpeg", streamer);
				server.setExecutor(ExecutorService.get()); // creates a default executor
				server.start();

				streamer.registerOverlayListener(new DefaultOverlayListener(WIDTH,HEIGHT,model));

			} catch(Exception e) { System.out.println("! No vision stream available"); }


		}

		control.connect();

		
		// Dispatch commands
		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {
				msg_msp_command cmd = (msg_msp_command)o;
				switch(cmd.command) {
				case MSP_CMD.MSP_TRANSFER_MICROSLAM:
					commander.getAutopilot().invalidate_map_transfer();
					break;
				case MSP_CMD.SELECT_VIDEO_STREAM:
					switch((int)cmd.param1) {
					case 1:
						if(pose!=null)  pose.enableStream(true);  
						if(depth!=null) depth.enableStream(false);
						break;
					case 0:
						if(depth!=null) depth.enableStream(true);
						if(pose!=null)  pose.enableStream(false);
						break;
					}
					break;
				}
			}
		});

//		// To ensure, that LIDAR is started when reboot via console
//		control.getStatusManager().addListener( Status.MSP_IMU_AVAILABILITY, (n) -> {
//			if(n.isStatus(Status.MSP_IMU_AVAILABILITY)) {
//				control.sendShellCommand("sf1xx start -X");
//				//	control.sendShellCommand("rm3100 start");
//			}
//		});

		// ?????
		control.getStatusManager().addListener(StatusManager.TYPE_ESTIMATOR, ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL, StatusManager.EDGE_FALLING, (n) -> {
			MSPLogger.getInstance().writeLocalMsg("[msp] Position estimation failure. Action required.", MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
			// TODO: Eventually Emergency Off if altitude < 1m
			// or other action to recover
		});


		System.out.println(control.getStatusManager().getSize()+" status events registered");
		
		// Setup WorkQueues and start them
		
		wq.addCyclicTask("LP", 200,  new Console(control));
		wq.addCyclicTask("LP", 200,  hw);
		wq.addCyclicTask("LP", 1000, inflightCheck);
		wq.addCyclicTask("NP", 10,   dispatcher);
		
		wq.addSingleTask("LP", 500, new initPX4());
		
		wq.start();
		
	}


	public static void main(String[] args)  {
        
		new StartUp(args);

	}
	
	private void addShutdownHook() {
		
		Runtime.getRuntime().addShutdownHook(new Thread() {
			public void run() {
				
				wq.stop();

				if(vision!=null)
					vision.stop();
				if(pose!=null)
					pose.stop();
				if(depth!=null)
					depth.stop();
			}
		});

	}
	
	private class initPX4 implements Runnable {

		@Override
		public void run() {
			if(mode==MAVController.MODE_NORMAL && control.isConnected()) {
				System.out.println("Execute init PX4");
				//control.sendShellCommand("dshot beep4");
				control.sendShellCommand("sf1xx start -X");
	//			control.sendShellCommand("rm3100 start");

				// enforce NUTTX RTC set to companion time
				SimpleDateFormat sdf = new SimpleDateFormat("MMM dd HH:mm:ss YYYY");   
				sdf.setTimeZone(TimeZone.getTimeZone("UTC"));
				String s = sdf.format(new Date());
				control.sendShellCommand("date -s \""+s+"\"");
			}	
		}	
	}

}

