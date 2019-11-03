/****************************************************************************
 *
 *   Copyright (c) 2017 Eike Mansfeld ecm@gmx.de. All rights reserved.
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

import java.io.IOException;
import java.lang.management.MemoryMXBean;
import java.lang.management.OperatingSystemMXBean;
import java.net.InetSocketAddress;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_CMD;
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
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcontrol.commander.MSPCommander;
import com.comino.mavlquac.mjpeg.impl.HttpMJPEGHandler;
import com.comino.mavlquac.odometry.detectors.impl.FwDirectDepthDetector;
import com.comino.mavlquac.odometry.estimators.IPositionEstimator;
import com.comino.mavlquac.odometry.estimators.impl.MAVVisualPositionEstimatorVIO;
import com.comino.mavodometry.librealsense.r200.RealSenseInfo;
import com.comino.mavutils.legacy.ExecutorService;
import com.comino.mavutils.linux.LinuxUtils;
import com.comino.mavutils.upboard.CPUTemperature;
import com.comino.mavutils.upboard.UpLEDControl;
import com.comino.mavutils.upboard.WifiQuality;
import com.sun.net.httpserver.HttpServer;

import boofcv.concurrency.BoofConcurrency;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.point.Point3D_F64;

public class StartUp implements Runnable {

	IMAVMSPController    control = null;
	MSPConfig	          config  = null;

	private OperatingSystemMXBean osBean = null;
	private MemoryMXBean mxBean = null;

	private HttpMJPEGHandler<?> streamer = null;

	private MSPCommander  commander = null;
	private DataModel     model     = null;

	private final long startTime_ms = System.currentTimeMillis();

	IPositionEstimator vision = null;
	private boolean publish_microslam;
	private boolean is_simulation;

	private MSPLogger logger;

	public StartUp(String[] args) {


		BoofConcurrency.setMaxThreads(2);

		ExecutorService.create();

		if(args.length != 0) {
			is_simulation = true;
		}

		RealSenseInfo info = null;

		if(is_simulation) {
			config  = MSPConfig.getInstance(System.getProperty("user.home")+"/","msp.properties");
			control = new MAVProxyController(MAVController.MODE_SITL);
		}
		else {

			try {
				Thread.sleep(5000);
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}

			config  = MSPConfig.getInstance("/home/lquac/","msp.properties");
			control = new MAVProxyController(MAVController.MODE_NORMAL);
		}

		System.out.println("MSPControlService (LQUAC build) version "+config.getVersion());

		osBean =  java.lang.management.ManagementFactory.getOperatingSystemMXBean();
		mxBean = java.lang.management.ManagementFactory.getMemoryMXBean();

		logger = MSPLogger.getInstance(control);

		commander = new MSPCommander(control,config);
		commander.getAutopilot().resetMap();

		control.start();
		model = control.getCurrentModel();


		control.getStatusManager().addListener(StatusManager.TYPE_MSP_SERVICES,
				Status.MSP_SLAM_AVAILABILITY, StatusManager.EDGE_FALLING, (n) -> {
					logger.writeLocalMsg("[msp] SLAM disabled", MAV_SEVERITY.MAV_SEVERITY_INFO);
				});


		Runtime.getRuntime().addShutdownHook(new Thread() {
			public void run() {
				if(vision!=null)
					vision.stop();
			}
		});

		control.getStatusManager().addListener(StatusManager.TYPE_PX4_STATUS, Status.MSP_GCL_CONNECTED, StatusManager.EDGE_FALLING, (n)-> {
			System.out.println("Connection to GCL lost..");

		});



		logger.writeLocalMsg("MAVProxy "+config.getVersion()+" loaded");
		//if(!is_simulation) {
		Thread worker = new Thread(this);
		worker.setPriority(Thread.MIN_PRIORITY);
		worker.setName("Main");
		worker.start();
		//	}

		// Start services if required

		try {
			Thread.sleep(200);

			if(config.getBoolProperty("vision_enabled", "true")) {

				if(config.getBoolProperty("vision_highres", "false"))
					info = new RealSenseInfo(640,480, RealSenseInfo.MODE_RGB);
				else
					info = new RealSenseInfo(320,240, RealSenseInfo.MODE_RGB);


				streamer = new HttpMJPEGHandler<Planar<GrayU8>>(info, control.getCurrentModel());


				//		vision = new MAVVisualPositionEstimatorVO(info, control, config, streamer);
				vision = new MAVVisualPositionEstimatorVIO(info, control, config, streamer);

				vision.registerDetector(new FwDirectDepthDetector(control,config,streamer));



				HttpServer server;
				try {
					server = HttpServer.create(new InetSocketAddress(8080),2);
					server.createContext("/mjpeg", streamer);
					server.setExecutor(null); // creates a default executor
					server.start();
				} catch (IOException e) {
					System.err.println(e.getMessage());
				}

			}
		} catch(Exception e) { System.out.println("No vision available: "+e.getMessage()); }


		this.publish_microslam = config.getBoolProperty("slam_publish_microslam", "true");
		System.out.println("[vis] Publishing microSlam enabled: "+publish_microslam);

		if(vision!=null && !vision.isRunning()) {
			vision.start();
		}

		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {
				Point3D_F64 target = null;
				msg_msp_command cmd = (msg_msp_command)o;
				switch(cmd.command) {
				case MSP_CMD.SET_OPTICAL_TARGET:
					if(vision.getOdometry() == null || !vision.isRunning()) {
						return;
					}
					try {
						if(Float.isNaN(cmd.param1) || Float.isNaN(cmd.param2))
							target = vision.getOdometry().getPoint3DFromPixel(160, 120);
						else
							target = vision.getOdometry().getPoint3DFromPixel((int)cmd.param1, (int)cmd.param2);

                        if(target!=null && target.z < 15.0f)
						   logger.writeLocalMsg(String.format("OpticalTarget: [%.2f %.2f %.2f]", target.x,target.z,target.y));
                        else
                        	logger.writeLocalMsg("OpticalTarget could not be set)");

					} catch(Exception e) {
						e.printStackTrace();
					}
					// TODO: Rotate into body_ned and transform to world (add LPOS)
					//  commander.getAutopilot().moveto((float)target.x, (float)target.y, (float)target.z - 0.25f, Float.NaN);
					break;
				}
			}
		});

	}

	public static void main(String[] args)  {

		if(args.length==0)
			UpLEDControl.clear();

		new StartUp(args);

	}


	@Override
	public void run() {
		long tms = System.currentTimeMillis();
		long blink = tms;
		boolean shell_commands = false;
		int pack_count;

		DataModel model = control.getCurrentModel();

		WifiQuality wifi = new WifiQuality();
		CPUTemperature temp = new CPUTemperature();
		msg_msp_micro_grid grid = new msg_msp_micro_grid(2,1);
		msg_msp_status msg = new msg_msp_status(2,1);


		while(true) {
			try {

				if(!control.isConnected()) {
					Thread.sleep(200);
					control.connect();
					continue;
				}

				pack_count = 0; publish_microslam = true;

				while(publish_microslam && model.grid.hasTransfers() && pack_count++ < 5 && model.sys.isStatus(Status.MSP_GCL_CONNECTED)) {
					if(model.grid.toArray(grid.data)) {
						grid.resolution = 0.05f;
						grid.extension  = 0;
						grid.cx  = model.grid.getIndicatorX();
						grid.cy  = model.grid.getIndicatorY();
						grid.cz  = model.grid.getIndicatorZ();
						grid.tms = model.grid.tms;
						grid.count = model.grid.count;
						control.sendMAVLinkMessage(grid);
						Thread.sleep(20);
					}
				}

				//     streamer.addToStream(Autopilot2D.getInstance().getMap2D().getMap().subimage(400-160, 400-120, 400+160, 400+120), model, System.currentTimeMillis()*1000);

				Thread.sleep(50);

				if((System.currentTimeMillis()-tms) < 333)
					continue;

				tms = System.currentTimeMillis();


				if(!control.isSimulation()) {

					if(!shell_commands ) {
						control.sendShellCommand("pmw3901 start");
						//control.sendShellCommand("tune_control play -m MFT200e8a8aE");
						shell_commands = true;
					}

					msg_timesync sync_s = new msg_timesync(255,1);
					sync_s.tc1 = 0;
					sync_s.ts1 = System.currentTimeMillis()*1000000L;
					control.sendMAVLinkMessage(sync_s);

					wifi.getQuality();
					temp.getTemperature();
				}

				msg.load = LinuxUtils.getProcessCpuLoad();
				msg.memory = (int)(mxBean.getHeapMemoryUsage().getUsed() * 100 /mxBean.getHeapMemoryUsage().getMax());
				msg.wifi_quality = (byte)wifi.get();
				msg.threads = Thread.activeCount();
				msg.cpu_temp = (byte)temp.get();
				msg.com_error = control.getErrorCount();
				msg.autopilot_mode =control.getCurrentModel().sys.autopilot;
				msg.uptime_ms = System.currentTimeMillis() - startTime_ms;
				msg.status = control.getCurrentModel().sys.getStatus();
				msg.setVersion(config.getVersion()+"/"+config.getVersionDate().replace(".", ""));
				msg.setArch(osBean.getArch());
				msg.unix_time_us = System.currentTimeMillis() * 1000;
				control.sendMAVLinkMessage(msg);

				if((System.currentTimeMillis()-blink) < 3000 || is_simulation)
					continue;

				blink = System.currentTimeMillis();

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

