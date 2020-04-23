/****************************************************************************
 *
 *   Copyright (c) 2020 Eike Mansfeld ecm@gmx.de. All rights reserved.
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

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_COMPONENT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.lquac.msg_heartbeat;
import org.mavlink.messages.lquac.msg_msp_command;
import org.mavlink.messages.lquac.msg_msp_micro_grid;
import org.mavlink.messages.lquac.msg_msp_status;
import org.mavlink.messages.lquac.msg_timesync;

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
import com.comino.mavlquac.preflight.MSPPreflightCheck;
import com.comino.mavodometry.estimators.MAVR200DepthEstimator;
import com.comino.mavodometry.estimators.MAVR200PositionEstimator;
import com.comino.mavodometry.estimators.MAVT265PositionEstimator;
import com.comino.mavodometry.video.impl.HttpMJPEGHandler;
import com.comino.mavutils.hw.HardwareAbstraction;
import com.comino.mavutils.hw.upboard.UpLEDControl;
import com.comino.mavutils.legacy.ExecutorService;
import com.sun.net.httpserver.HttpServer;

import boofcv.concurrency.BoofConcurrency;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class StartUp implements Runnable {

	private static final int WIDTH  = 320;
	private static final int HEIGHT = 240;


	IMAVMSPController    control = null;
	MSPConfig	          config  = null;

	private HttpMJPEGHandler<Planar<GrayU8>> streamer = null;

	private MSPCommander  commander = null;
	private DataModel     model     = null;

	private final long startTime_ms = System.currentTimeMillis();

	private MAVR200PositionEstimator vision = null;
	private MAVT265PositionEstimator pose = null;
	private MAVR200DepthEstimator depth = null;

	private boolean publish_microslam;
	private int mode;

	private MSPLogger logger;
	private PX4Parameters params;

	private final HardwareAbstraction hw = HardwareAbstraction.instance();

	public StartUp(String[] args) {

		//		new JetsonNanoInferenceTest();

		BoofConcurrency.setMaxThreads(2);

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

			try {
				Thread.sleep(1000);
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}

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


		control.start();
		model = control.getCurrentModel();

		params = PX4Parameters.getInstance(control);

		commander = new MSPCommander(control,config);
		//	commander.getAutopilot().resetMap();

		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {
				msg_msp_command cmd = (msg_msp_command)o;
				switch(cmd.command) {
				case MSP_CMD.MSP_TRANSFER_MICROSLAM:
					model.grid.invalidateTransfer();
					break;
				}
			}
		});

		control.getStatusManager().addListener(Status.MSP_CONNECTED, (n) -> {
			if(n.isStatus(Status.MSP_CONNECTED))
				params.requestRefresh();
		});

		// Set initial PX4 Parameters
		control.getStatusManager().addListener(Status.MSP_PARAMS_LOADED, (n) -> {
			if(n.isStatus(Status.MSP_PARAMS_LOADED))
				params.sendParameter("COM_OBS_AVOID", 0);
		});

		control.getStatusManager().addListener(StatusManager.TYPE_PX4_STATUS,
				Status.MSP_ARMED, StatusManager.EDGE_RISING, (n) -> {
					if(MSPPreflightCheck.getInstance(control).performCheck(model, params)==MSPPreflightCheck.FAILED) {
						control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM,0 );
						logger.writeLocalMsg("[msp] Disarmed. PreFlight health check failed",
								MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
					}
				});


		control.getStatusManager().addListener(StatusManager.TYPE_MSP_SERVICES,
				Status.MSP_SLAM_AVAILABILITY, StatusManager.EDGE_FALLING, (n) -> {
					logger.writeLocalMsg("[msp] SLAM disabled", MAV_SEVERITY.MAV_SEVERITY_INFO);
				});


		Runtime.getRuntime().addShutdownHook(new Thread() {
			public void run() {
				if(vision!=null)
					vision.stop();
				if(pose!=null)
					pose.stop();
				if(depth!=null)
					depth.stop();
			}
		});



		logger.writeLocalMsg("MAVProxy "+config.getVersion()+" loaded");
		//if(!is_simulation) {
		Thread worker = new Thread(this);
		worker.setPriority(Thread.MIN_PRIORITY);
		worker.setName("Main");
		worker.start();
		//	}


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

				pose = new MAVT265PositionEstimator(control, config, WIDTH,HEIGHT, MAVT265PositionEstimator.LPOS_MODE_NED);
				pose.start();

			} catch(UnsatisfiedLinkError | Exception e ) { System.out.println("! No pose estimation available"); }


			//*** R200 Depth estimation

			try {

				depth = new MAVR200DepthEstimator(control, commander.getAutopilot(), config, WIDTH,HEIGHT, commander.getMap(), streamer);
				depth.start();

			} catch(UnsatisfiedLinkError | Exception e) { 	System.out.println("! No depth estimation available"); 	}



			//***********

			try {


				HttpServer server;

				server = HttpServer.create(new InetSocketAddress(8080),2);
				server.createContext("/mjpeg", streamer);
				server.setExecutor(ExecutorService.get()); // creates a default executor
				server.start();

			} catch(Exception e) { System.out.println("! No vision stream available"); }


		}

		control.connect();


		this.publish_microslam = config.getBoolProperty("slam_publish_microslam", "true");
		System.out.println("[vis] Publishing microSlam enabled: "+publish_microslam);

	}

	public static void main(String[] args)  {

		new StartUp(args);

	}

	@Override
	public void run() {
		long tms = System.currentTimeMillis();
		long blink = tms;
		boolean shell_commands = false;
		int pack_count;

		final msg_heartbeat beat_gcs = new msg_heartbeat(2,MAV_COMPONENT.MAV_COMP_ID_AUTOPILOT1);

		final DataModel model = control.getCurrentModel();

		final msg_msp_micro_grid grid = new msg_msp_micro_grid(2,1);
		final msg_msp_status msg = new msg_msp_status(2,1);

		if(hw.getArchId() == HardwareAbstraction.UPBOARD)
			UpLEDControl.clear();


		while(true) {
			try {

				if(!control.isConnected()) {
					Thread.sleep(200);
					continue;
				}

				pack_count = 0; publish_microslam = true;

				if(model.grid.count > 3) {
					while(publish_microslam && model.grid.hasTransfers() && pack_count++ < 10) {
						if(model.grid.toArray(grid.data)) {
							grid.resolution = 0.05f;
							grid.extension  = model.grid.getExtension();
							grid.cx  = model.grid.getIndicatorX();
							grid.cy  = model.grid.getIndicatorY();
							grid.cz  = model.grid.getIndicatorZ();
							grid.tms = model.sys.getSynchronizedPX4Time_us();
							grid.count = model.grid.count;
							control.sendMAVLinkMessage(grid);
						}
					}
				}

				//     streamer.addToStream(Autopilot2D.getInstance().getMap2D().getMap().subimage(400-160, 400-120, 400+160, 400+120), model, System.currentTimeMillis()*1000);

				Thread.sleep(50);

				if((System.currentTimeMillis()-tms) < 333)
					continue;

				tms = System.currentTimeMillis();


				if(mode==MAVController.MODE_NORMAL) {

					if(!shell_commands ) {
						//control.sendShellCommand("rm3100 start");
						//control.sendShellCommand("sf1xx start -a");
						//control.sendShellCommand("dshot beep4");
						shell_commands = true;
					}

				}

				hw.update();

				msg.load = hw.getCPULoad();
				msg.memory = hw.getMemoryUsage();
				msg.wifi_quality = hw.getWifiQuality();
				msg.threads = Thread.activeCount();
				msg.cpu_temp = hw.getTemperature();
				msg.com_error = control.getErrorCount();
				msg.autopilot_mode =control.getCurrentModel().sys.autopilot;
				msg.uptime_ms = System.currentTimeMillis() - startTime_ms;
				msg.status = control.getCurrentModel().sys.getStatus();
				msg.setVersion(config.getVersion()+"/"+config.getVersionDate().replace(".", ""));
				msg.setArch(hw.getArchName());
				msg.unix_time_us = System.currentTimeMillis() * 1000;
				control.sendMAVLinkMessage(msg);

				if(msg.cpu_temp > 80) {
					MSPLogger.getInstance().writeLocalMsg("Companion Temperature critical. Shut down.", MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
				}

				if((System.currentTimeMillis()-blink) < 3000)
					continue;

				blink = System.currentTimeMillis();

				msg_timesync sync_s = new msg_timesync(255,1);
				sync_s.tc1 = 0;
				sync_s.ts1 = System.currentTimeMillis()*1000000L;
				control.sendMAVLinkMessage(sync_s);

				if(hw.getArchId() != HardwareAbstraction.UPBOARD)
					continue;


				if(model.sys.isStatus(Status.MSP_ACTIVE))
					UpLEDControl.flash("green", 10);
				else
					UpLEDControl.flash("red", 200);

			} catch (Exception e) {
				e.printStackTrace();
				control.close();
			}
		}
	}
}

