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

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.net.InetSocketAddress;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

import org.mavlink.messages.ESTIMATOR_STATUS_FLAGS;
import org.mavlink.messages.IMAVLinkMessageID;
import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.lquac.msg_adsb_vehicle;
import org.mavlink.messages.lquac.msg_msp_command;

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
import com.comino.mavodometry.estimators.MAVD455DepthEstimator;
import com.comino.mavodometry.estimators.MAVR200DepthEstimator;
import com.comino.mavodometry.estimators.MAVR200PositionEstimator;
import com.comino.mavodometry.estimators.MAVT265PositionEstimator;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavodometry.video.impl.DefaultOverlayListener;
import com.comino.mavodometry.video.impl.h264.HttpH264Handler;
import com.comino.mavodometry.video.impl.mjpeg.HttpMJPEGHandler;
import com.comino.mavodometry.video.impl.mjpeg.RTSPMjpegHandler;
import com.comino.mavutils.hw.HardwareAbstraction;
import com.comino.mavutils.legacy.ExecutorService;
import com.comino.mavutils.workqueue.WorkQueue;
import com.sun.net.httpserver.HttpServer;

import boofcv.concurrency.BoofConcurrency;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class StartUp  {

	private final WorkQueue wq = WorkQueue.getInstance();

	private static final int WIDTH  = 640;
	private static final int HEIGHT = 480;

	//	private static final int WIDTH  = 320;
	//	private static final int HEIGHT = 240;


	IMAVMSPController    control    = null;
	MSPConfig	          config    = null;
	MAVLinkDispatcher    dispatcher = null;

	private IVisualStreamHandler<Planar<GrayU8>> streamer = null;

	private MSPCommander  commander = null;
	private DataModel     model     = null;

	private MAVR200PositionEstimator vision = null;
	private MAVT265PositionEstimator pose = null;

	private MAVD455DepthEstimator depth = null;
	//	private MAVR200DepthEstimator depth = null;

	private int mode;

	private MSPLogger logger;
	private PX4Parameters params;


	private final HardwareAbstraction hw;
	private final MSPInflightCheck inflightCheck;


	public StartUp(String[] args) {

		//		new JetsonNanoInferenceTest();

		addShutdownHook();

		this.hw = HardwareAbstraction.instance();

		BoofConcurrency.setMaxThreads(2);

		ExecutorService.create();

		if(args.length != 0) {
			if(args[0].contains("SIM"))
				mode = MAVController.MODE_SITL;
			else if(args[0].contains("SERVER"))
				mode = MAVController.MODE_SERVER;
			else if(args[0].contains("USB"))
				mode = MAVController.MODE_USB;
			else if(args[0].contains("log")) {
				redirectConsole();
				mode = MAVController.MODE_NORMAL;
			}
			else  
				mode = MAVController.MODE_NORMAL;
		}

		System.out.println("BoofConcurrency: "+BoofConcurrency.isUseConcurrent());

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


		control.getStatusManager().addListener(Status.MSP_CONNECTED, (n) -> {

			// Request parameter refresh when reconnected on ground
			if(n.isStatus(Status.MSP_CONNECTED) && !model.sys.isStatus(Status.MSP_ARMED)) {
				params.requestRefresh();

				// Disable UTM stream
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL,(cmd,result) -> {
					if(result == MAV_RESULT.MAV_RESULT_ACCEPTED) 
						logger.writeLocalMsg("[msp] UTM stream disabled.",
								MAV_SEVERITY.MAV_SEVERITY_DEBUG);
				},IMAVLinkMessageID.MAVLINK_MSG_ID_UTM_GLOBAL_POSITION,-1);	

			}		
		});

		// Set initial PX4 Parameters
		control.getStatusManager().addListener(Status.MSP_PARAMS_LOADED, (n) -> {
			if(n.isStatus(Status.MSP_PARAMS_LOADED) && !model.sys.isStatus(Status.MSP_ARMED)) {
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




		logger.writeLocalMsg("MAVProxy (Version: "+config.getVersion()+") loaded");

		// Start services if required

		control.connect();

		try {	Thread.sleep(300); } catch(Exception e) { }


		if(config.getBoolProperty("vision_enabled", "true")) {



			//			streamer = new HttpMJPEGHandler<Planar<GrayU8>>(WIDTH,HEIGHT, control.getCurrentModel());
			//			try {
			//
			//
			//				final HttpServer server;
			//
			//				server = HttpServer.create(new InetSocketAddress(8080),1);
			//				server.createContext("/mjpeg", (HttpMJPEGHandler<Planar<GrayU8>>)streamer);
			//				//		server.setExecutor(ExecutorService.get()); // creates a default executor
			//				server.start();
			//
			//				streamer.registerOverlayListener(new DefaultOverlayListener(WIDTH,HEIGHT,model));
			//
			//			} catch(Exception e) { System.out.println("! No vision stream available"); }

			if(!control.isSimulation()) {

				streamer = new RTSPMjpegHandler<Planar<GrayU8>>(WIDTH,HEIGHT,control.getCurrentModel());
				streamer.registerOverlayListener(new DefaultOverlayListener(WIDTH,HEIGHT,model));
				try {
					((RTSPMjpegHandler)streamer).start(1051);
				} catch (Exception e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
			}



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

				pose = new MAVT265PositionEstimator(control, config, WIDTH,HEIGHT, MAVT265PositionEstimator.LPOS_ODO_MODE_NED_GND, streamer);
				pose.start();

			} catch(UnsatisfiedLinkError | Exception e ) {
				System.out.println("! No pose estimation available");

				if(!control.isSimulation())
					e.printStackTrace();
			}

			//*** D455 depth

			try {

				depth = new MAVD455DepthEstimator(control, commander.getAutopilot(), commander.getAutopilot().getMap(),config, WIDTH,HEIGHT, streamer);
				depth.enableStream(true);
				depth.start();


			} catch(UnsatisfiedLinkError | Exception e ) {
				System.out.println("! No depth estimation available");

				if(!control.isSimulation())
					e.printStackTrace();
			}


			//			//*** R200 Depth estimation
			//
			//			try {
			//
			//				depth = new MAVR200DepthEstimator(control, commander.getAutopilot(), commander.getAutopilot().getMap(),config, WIDTH,HEIGHT, streamer);
			//				depth.enableStream(true);
			//				depth.start();
			//
			//			} catch(UnsatisfiedLinkError | Exception e ) {
			//				System.out.println("! No depth estimation available");
			//
			//				if(!control.isSimulation())
			//					e.printStackTrace();
			//			}
			//			//***********




		}

		if(depth!=null) {
			depth.enableStream(true);
			pose.enableStream(false);
		}


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


		System.out.println(control.getStatusManager().getSize()+" status events registered");

		// Setup WorkQueues and start them

		Console console = new Console(control);
		console.registerCmd("rate", () -> System.out.println(streamer.toString()));

		wq.addCyclicTask("LP", 200,  console);
		wq.addCyclicTask("LP", 200,  hw);
		wq.addCyclicTask("LP", 500,  inflightCheck);

		wq.addSingleTask("LP", 1000, new initPX4());

		wq.start();

	}


	public static void main(String[] args)  {

		new StartUp(args);

	}

	private void addShutdownHook() {

		Runtime.getRuntime().addShutdownHook(new Thread() {
			public void run() {

				control.shutdown();
				wq.stop();

				if(vision!=null)
					vision.stop();
				if(pose!=null)
					pose.stop();
				if(depth!=null)
					depth.stop();
				try {
					Thread.sleep(200);
				} catch (InterruptedException e) {}
			}

		});

	}

	private class initPX4 implements Runnable {


		@Override
		public void run() {
			if((mode==MAVController.MODE_NORMAL || mode==MAVController.MODE_USB)  && model.sys.isStatus(Status.MSP_CONNECTED)) {
				System.out.println("Execute init PX4");
				//control.sendShellCommand("dshot beep4");
				//			control.sendShellCommand("sf1xx start -X");
				control.sendShellCommand("lightware_laser_i2c start -X");	

				// enforce NUTTX RTC set to companion time
				SimpleDateFormat sdf = new SimpleDateFormat("MMM dd HH:mm:ss YYYY");   
				sdf.setTimeZone(TimeZone.getTimeZone("UTC"));
				String s = sdf.format(new Date());
				control.sendShellCommand("date -s \""+s+"\"");
			}	
			
		}	
	}

	private void redirectConsole()  {

		try {
			File file = new File("/home/lquac/msp.log");

			if(file.exists())
				file.delete();
			file.createNewFile();

			PrintStream fileOut = new PrintStream(file);
			System.setOut(fileOut);
			System.setErr(fileOut);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

	}

}

