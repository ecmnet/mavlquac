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
import java.net.URLDecoder;
import java.security.CodeSource;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

import org.mavlink.messages.IMAVLinkMessageID;
import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.lquac.msg_msp_command;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.control.impl.MAVController;
import com.comino.mavcom.control.impl.MAVProxyController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.mavlink.IMAVLinkListener;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcontrol.commander.MSPCommander;
import com.comino.mavlquac.console.Console;
import com.comino.mavlquac.dispatcher.MAVLinkDispatcher;
import com.comino.mavlquac.inflight.MSPInflightCheck;
import com.comino.mavlquac.preflight.MSPPreflightCheck;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavodometry.estimators.depth.MAVOAKDDepthEstimator;
import com.comino.mavodometry.estimators.depth.MAVOAKDDepthSegmentEstimator;
import com.comino.mavodometry.estimators.depth.MAVSimDepthSegmentEstimator;
import com.comino.mavodometry.estimators.position.MAVT265PositionEstimator;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavodometry.video.impl.DefaultOverlayListener;
import com.comino.mavodometry.video.impl.mjpeg.RTSPMjpegHandler;
import com.comino.mavutils.hw.HardwareAbstraction;
import com.comino.mavutils.legacy.ExecutorService;
import com.comino.mavutils.workqueue.WorkQueue;

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

	private MAVAbstractEstimator pose  = null;
	private MAVAbstractEstimator depth = null;
	private MAVAbstractEstimator flow  = null;


	private int mode;

	private MSPLogger logger;
	private PX4Parameters params;


	private final HardwareAbstraction hw;
	private final MSPInflightCheck inflightCheck;


	public StartUp(String[] args) {


		addShutdownHook();

		this.hw = HardwareAbstraction.instance();

		BoofConcurrency.setMaxThreads(4);
	//	BoofConcurrency.USE_CONCURRENT = false;

		ExecutorService.create();

		if(args.length != 0) {
			if(args[0].contains("SIM"))
				mode = MAVController.MODE_SITL;
			//			else if(args[0].contains("SERVER"))
			//				mode = MAVController.MODE_SERVER;
			else if(args[0].contains("USB"))
				mode = MAVController.MODE_USB;
			else if(args[0].contains("log")) {
				redirectConsole();
				mode = MAVController.MODE_NORMAL;
			}
			else  
				mode = MAVController.MODE_NORMAL;
		}


		switch(mode) {

		case  MAVController.MODE_NORMAL:

			config  = MSPConfig.getInstance(getJarContainingFolder(this.getClass()),"msp.properties");
			control = new MAVProxyController(MAVController.MODE_NORMAL);
			System.out.println("MSPControlService (LQUAC build) version "+config.getVersion());

			break;

			//		case MAVController.MODE_SERVER:
			//
			//			config  = MSPConfig.getInstance(getJarContainingFolder(this.getClass()),"msp.properties");
			//			control = new MAVProxyController(MAVController.MODE_SERVER);
			//			System.out.println("MSPControlService (LQUAC JetsonNano) version "+config.getVersion()+" Mode = "+mode);
			//			control.getCurrentModel().clear();
			//			break;

		default:

			config  = MSPConfig.getInstance(System.getProperty("user.home")+"/","msp.properties");
			control = new MAVProxyController(mode);
			System.out.println("MSPControlService (LQUAC simulation) version "+config.getVersion()+" Mode = "+mode);
		}

		params = PX4Parameters.getInstance(control);


		logger = MSPLogger.getInstance(control);
		logger.enableDebugMessages(true);

		try { Thread.sleep(300); } catch(Exception e) { }

		control.connect();

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {}


		inflightCheck = new MSPInflightCheck(control, hw);
		dispatcher = new MAVLinkDispatcher(control, config, hw);

		control.start();
		model = control.getCurrentModel();


		commander = new MSPCommander(control,config);

		control.getStatusManager().addListener(StatusManager.TYPE_PX4_STATUS, Status.MSP_CONNECTED, StatusManager.EDGE_RISING, (a) -> {
			if(!model.sys.isStatus(Status.MSP_ARMED)) {
				System.out.println("Setting up MAVLINK streams...");
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL,IMAVLinkMessageID.MAVLINK_MSG_ID_UTM_GLOBAL_POSITION,-1);			
				//	control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL,IMAVLinkMessageID.MAVLINK_MSG_ID_ESC_STATUS,100);
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL,IMAVLinkMessageID.MAVLINK_MSG_ID_ESTIMATOR_STATUS,0);
				params.requestRefresh(true);
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
					params.sendParameter("COM_RCL_EXCEPT", 7);
					params.sendParameter("MPC_XY_VEL_P_ACC", 4.5f);
					params.sendParameter("MIS_TAKEOFF_ALT", 1.5f);
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


		logger.writeLocalMsg("MSP (Version: "+config.getVersion()+") started");

		// Start services if required

		//		flow = new MAVFlowPositionEstimator(control);
		//		try {
		//			flow.start();
		//		} catch (Exception e) {
		//		}


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

		// Send system time

		//		Instant ins = Instant.now();
		//		long now_ns = ins.getEpochSecond() * 1000000000L + ins.getNano();
		//		msg_system_time time     = new msg_system_time(1, 1);
		//		time.time_unix_usec = now_ns /1000L;
		//		control.sendMAVLinkMessage(time);

		// Setup WorkQueues and start them

		Console console = new Console(control);
		console.registerCmd("rate", () -> System.out.println(streamer.toString()));

		wq.addCyclicTask("LP", 200,  console);
		wq.addCyclicTask("LP", 500,  hw);
		wq.addCyclicTask("LP", 500,  inflightCheck);
		wq.addSingleTask("LP", 100,  new initPX4());

		wq.start();
		
		if(config.getBoolProperty(MSPParams.VISION_ENABLED, "true")) {
			startOdometry();
		}

	}


	public static void main(String[] args)  {
		System.setProperty("sun.java2d.opengl", "false");
		System.setProperty("sun.java2d.xrender", "false");
//		System.setProperty("org.bytedeco.javacpp.logger.debug", "true");
//		System.setProperty("org.bytedeco.javacpp.nopointergc", "true");

		
		new StartUp(args);

	}

	private void addShutdownHook() {

		Runtime.getRuntime().addShutdownHook(new Thread() {
			public void run() {

				control.shutdown();
				wq.printStatus();
				wq.stop();

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

	private void startOdometry() {

		streamer = new RTSPMjpegHandler<Planar<GrayU8>>(WIDTH,HEIGHT,control.getCurrentModel());
		streamer.registerOverlayListener(new DefaultOverlayListener(WIDTH,HEIGHT,model));
		streamer.registerNoVideoListener(() -> {
			if(pose!=null)  
				pose.enableStream(true);  
			else if(depth!=null) 
				depth.enableStream(true);
		});
		try {
			((RTSPMjpegHandler<Planar<GrayU8>>)streamer).start(1051);
		} catch (Exception e1) {
			// TODO Auto-generated catch block
			if(!control.isSimulation())
				e1.printStackTrace();
		}

		model.vision.setStatus(Vision.VIDEO_ENABLED, false);

		try {

			pose = new MAVT265PositionEstimator(control, config, WIDTH,HEIGHT, MAVT265PositionEstimator.LPOS_ODO_MODE_POSITION, streamer);
			pose.start();
			model.vision.setStatus(Vision.VIDEO_ENABLED, true);

		} catch(UnsatisfiedLinkError | Exception e ) {
			System.out.println("No T265 device found");
		}


//		if(control.isSimulation() && pose == null) {
//			pose = new MAVSITLPositionEstimator(control);
//		}


		//*** OAK-D as depth
		if(depth==null) {
			try {
	//			depth = new MAVOAKDDepthSegmentEstimator(control,config, commander.getAutopilot().getMap(),WIDTH,HEIGHT, streamer);
				depth = new MAVOAKDDepthEstimator(control,config, commander.getAutopilot().getMap(),WIDTH,HEIGHT, streamer); 
				depth.start();
				model.vision.setStatus(Vision.VIDEO_ENABLED, true);

			} catch (Exception e) {
				if(!control.isSimulation())
				  e.printStackTrace();
				depth=null;
				System.out.println("No OAKD-Lite device found");
			}

		}
		
		if(depth==null && control.isSimulation()) {
			try {
				depth = new MAVSimDepthSegmentEstimator(control,config, commander.getAutopilot().getMap(),WIDTH,HEIGHT, streamer);
				depth.start();
				model.vision.setStatus(Vision.VIDEO_ENABLED, true);

			} catch (Exception e) {
				System.out.println("No depth simulation found");
			}

		}
		
		
		//*** OAK-D as simple Camera
//		if(depth==null) {
//			try {
//				depth = new MAVOAKDCamEstimator(control,config, WIDTH,HEIGHT, streamer);
//				depth.start();
//				model.vision.setStatus(Vision.VIDEO_ENABLED, true);
//
//			} catch (Exception e) {
//				System.out.println("No OAKD-Lite device found");
//			}
//
//		}

		//*** D4xx as depth

//		if(depth==null) {
//			try {
//				depth = new MAVD4xxDepthEstimator(control, commander.getAutopilot(), commander.getAutopilot().getMap(),config, WIDTH,HEIGHT, streamer);
//				depth.start();
//				model.vision.setStatus(Vision.VIDEO_ENABLED, true);
//
//			} catch(UnsatisfiedLinkError | Exception e ) {
//				System.out.println("No D455 device found");
//			}
//		}


		//*** WebCam as depth

		//		if(!control.isSimulation() && depth == null) {
		//			try {
		//				depth = new MAVFPVCameraNullEstimator(control, config, WIDTH,HEIGHT, MAVT265PositionEstimator.LPOS_ODO_MODE_POSITION, streamer);
		//				depth.start();
		//
		//			} catch(UnsatisfiedLinkError | Exception e ) {
		//				System.out.println("! No FPV camera available");
		//			}
		//		}

		//		if(control.isSimulation() && depth == null) {
		//			try {
		//			depth = new MAVGazeboFpvNullEstimator(control, config, WIDTH,HEIGHT, MAVT265PositionEstimator.LPOS_ODO_MODE_POSITION, streamer);
		//			depth.start();
		//			depth.enableStream(true);
		//			} catch(UnsatisfiedLinkError | Exception e ) {
		//				System.out.println("! No FPV available");
		//			}
		//		}


		if(pose!=null) {
			if(depth!=null) 
				depth.enableStream(false);
			pose.enableStream(true);
		} 

		if(depth!=null) {
			depth.enableStream(true);
			if(pose!=null) 
				pose.enableStream(false);
		}
	}

	private class initPX4 implements Runnable {


		@Override
		public void run() {
			if((mode==MAVController.MODE_NORMAL || mode==MAVController.MODE_USB)  && model.sys.isStatus(Status.MSP_CONNECTED)) {
				System.out.println("Execute init PX4");

				// enforce NUTTX RTC set to companion time
				SimpleDateFormat sdf = new SimpleDateFormat("MMM dd HH:mm:ss YYYY");   
				sdf.setTimeZone(TimeZone.getTimeZone("UTC"));
				String s = sdf.format(new Date());
				control.sendShellCommand("date -s \""+s+"\"");
				control.sendShellCommand("lightware_laser_i2c start -X");	
				params.requestRefresh(false);

			}	

		}	
	}

	private void redirectConsole()  {

		try {
			File file = new File(getJarContainingFolder(this.getClass())+"/msp.log");

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

	@SuppressWarnings("rawtypes")
	public static String getJarContainingFolder(Class aclass)  {
		CodeSource codeSource = aclass.getProtectionDomain().getCodeSource();

		File jarFile;

		try {

			if (codeSource.getLocation() != null) {
				jarFile = new File(codeSource.getLocation().toURI());
			}
			else {
				String path = aclass.getResource(aclass.getSimpleName() + ".class").getPath();
				String jarFilePath = path.substring(path.indexOf(":") + 1, path.indexOf("!"));
				jarFilePath = URLDecoder.decode(jarFilePath, "UTF-8");
				jarFile = new File(jarFilePath);
			}
			System.out.println("Properties searched in "+jarFile.getParentFile().getAbsolutePath());
			return jarFile.getParentFile().getAbsolutePath();

		} catch(Exception e) {
			System.out.println("Properties not found: "+e.getMessage());
			return null;
		}
	}

}

