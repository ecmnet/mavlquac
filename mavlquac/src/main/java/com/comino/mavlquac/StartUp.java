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
import java.net.URISyntaxException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.TimeZone;

import org.apache.ftpserver.FtpServer;
import org.bytedeco.javacpp.Loader;
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
import com.comino.mavjros.MavJROSNode;
import com.comino.mavjros.subscribers.depth.MavJROSDepthMappingSubscriber;
import com.comino.mavjros.subscribers.rgb.MavJROSRGBSubscriber;
import com.comino.mavlquac.console.Console;
import com.comino.mavlquac.dispatcher.MAVLinkDispatcher;
import com.comino.mavlquac.ftpserver.MAVFtpServerFactory;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavodometry.estimators.depth.MAVOAKDDepthEstimator;
import com.comino.mavodometry.estimators.position.MAVT265PositionEstimator;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavodometry.video.impl.DefaultOverlayListener;
import com.comino.mavodometry.video.impl.mjpeg.RTSPMultiStreamMjpegHandler;
import com.comino.mavutils.MSPStringUtils;
import com.comino.mavutils.file.MSPFileUtils;
import com.comino.mavutils.hw.HardwareAbstraction;
import com.comino.mavutils.legacy.ExecutorService;
import com.comino.mavutils.workqueue.WorkQueue;

import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class StartUp  {

	private final WorkQueue wq = WorkQueue.getInstance();

	private static final int WIDTH  = 640;
	private static final int HEIGHT = 480;


	IMAVMSPController    control    = null;
	MSPConfig	          config    = null;
	MAVLinkDispatcher    dispatcher = null;

	private IVisualStreamHandler<Planar<GrayU8>> streamer = null;

	private MSPCommander  commander = null;
	private DataModel     model     = null;

	private MAVAbstractEstimator pose  = null;
	private MAVAbstractEstimator depth = null;
	


	private int mode;
	private boolean stream_auto_switched = false;

	private MSPLogger logger;
	private PX4Parameters params;
	
	private MavJROSNode node;


	private final HardwareAbstraction hw;

	private FtpServer ftpServer;


	public StartUp(String[] args) {


		try { Thread.sleep(1000); } catch(Exception e) { }


		addShutdownHook();

		this.hw = HardwareAbstraction.instance();

		try {
			System.out.println("Platform: "+Loader.getPlatform()+" JavaCPP version: "+Loader.getVersion());
		} catch (IOException e) {  }
		
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
			else if(args[0].contains("VM")) {
				mode = MAVController.MODE_SITL_PROXY;
			}
			else if(args[0].contains("ORIN")) {
				mode = MAVController.MODE_ORIN;
			}
			else  
				mode = MAVController.MODE_NORMAL;
		}


		switch(mode) {

		case  MAVController.MODE_NORMAL:

			config  = MSPConfig.getInstance(MSPFileUtils.getJarContainingFolder(this.getClass()),"msp.properties");
			control = new MAVProxyController(MAVController.MODE_NORMAL,config);
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

			config  = MSPConfig.getInstance(MSPFileUtils.getJarContainingFolder(this.getClass())+"/../properties/","msp.properties");
			control = new MAVProxyController(mode, config);
			System.out.println("MSPControlService (LQUAC simulation) version "+config.getVersion()+" Mode = "+mode);
		}
		

		MSPStringUtils.getInstance(control.isSimulation());

		logger = MSPLogger.getInstance(control);
		logger.enableDebugMessages(true);
		
		try {
			ftpServer = MAVFtpServerFactory.createAndStart();			
		} catch (Exception e) {
			System.out.println("FTP server not started: "+e.getMessage());
		}

		params = PX4Parameters.getInstance(control);

		model = control.getCurrentModel();

		commander  = new MSPCommander(control,config);
		dispatcher = new MAVLinkDispatcher(control, config, hw);

		registerActions();
		registerCommands();

		Console console = new Console(control);
		console.registerCmd("rate", () -> System.out.println(streamer.toString()));

		System.out.println(control.getStatusManager().getSize()+" status events registered");


		if(config.getBoolProperty(MSPParams.VISION_ENABLED, "true")) { 
			
			streamer = new RTSPMultiStreamMjpegHandler<Planar<GrayU8>>(control,WIDTH,HEIGHT,control.getCurrentModel());
			streamer.registerOverlayListener(new DefaultOverlayListener(WIDTH,HEIGHT,model));
	
			streamer.registerNoVideoListener(() -> {
				if(pose!=null)  
					pose.enableStream(true);  
				else if(depth!=null) 
					depth.enableStream(true);
			});
			
			try {
				((RTSPMultiStreamMjpegHandler<Planar<GrayU8>>)streamer).start(1051);
			} catch (Exception e1) { System.err.println( e1.getMessage());  }
			
			model.vision.setStatus(Vision.VIDEO_ENABLED, false);
			
			switch(mode) {
			
			case  MAVController.MODE_NORMAL:
		    // No ROS, everything runs on JAVA
				
				try {
					pose = new MAVT265PositionEstimator(control, config, WIDTH,HEIGHT, MAVT265PositionEstimator.LPOS_ODO_MODE_POSITION_BODY, streamer);
					pose.start();
					pose.enableStream(true);
					model.vision.setStatus(Vision.VIDEO_ENABLED, true);
				} catch(UnsatisfiedLinkError | Exception e ) {
					pose = null;
					System.out.println("No T265 device found");
				}
				
				if(depth==null) {
					try {
						depth = new MAVOAKDDepthEstimator(control,config, commander.getAutopilot().getMapper().getShorTermMap(),WIDTH,HEIGHT, streamer); 
						depth.start();
						depth.enableStream(true);
						model.vision.setStatus(Vision.VIDEO_ENABLED, true);

					} catch (Exception e) {
						depth=null;
						System.out.println("No OAKD-Lite device found");
					}
				}
				
				break;
				
			case  MAVController.MODE_ORIN:
		    // OAKD runs on ROS, T265 on Java
				
//				node = MavJROSNode.getInstance(control.getCurrentModel());
//			//	node.addSubscriber(new MavJROSLocalMap2OctomapSubscriber(model,commander.getAutopilot().getMapper().getShorTermMap(),"/local2global"));
//				node.addSubscriber(new MavJROSLocalMapTransferSubscriber(control,"/local2global"));
//				node.addSubscriber(new MavJROSRGBSubscriber(model,"/stereo_publisher/color/image", WIDTH,HEIGHT, streamer));
//				node.addSubscriber(new MavJROSDepthSubscriber(model,"/stereo_publisher/stereo/depth", WIDTH,HEIGHT, streamer));
//				try {
//					node.connect();
//					model.vision.setStatus(Vision.VIDEO_ENABLED, true);
//				} catch (Exception e) {
//					System.out.println("ROS Node not available");
//				}
				
				if(depth==null) {
					try {
						depth = new MAVOAKDDepthEstimator(control,config, commander.getAutopilot().getMapper().getShorTermMap(),WIDTH,HEIGHT, streamer); 
						depth.start();
						depth.enableStream(true);
						model.vision.setStatus(Vision.VIDEO_ENABLED, true);

					} catch (Exception e) {
						depth=null;
						System.out.println("No OAKD-Lite device found");
					}
				}
				
				try {
					pose = new MAVT265PositionEstimator(control, config, WIDTH,HEIGHT, MAVT265PositionEstimator.LPOS_ODO_MODE_POSITION_BODY, streamer);
					pose.start();
					pose.enableStream(true);
					model.vision.setStatus(Vision.VIDEO_ENABLED, true);
				} catch(UnsatisfiedLinkError | Exception e ) {
					pose = null;
					System.out.println("No T265 device found");
				}
				
				break;
				
		    default:
		    // ROS only
		    	
		    	node = MavJROSNode.getInstance(control.getCurrentModel());
		    //	node.addSubscriber(new MavJROSLocalMap2OctomapSubscriber(model,commander.getAutopilot().getMapper().getShorTermMap(),"/local2global"));
		    //	node.addSubscriber(new MavJROSOdometrySubscriber(control,"/gt_iris_base_link_imu"));
		    	node.addSubscriber(new MavJROSRGBSubscriber(model,"/camera/color/image_raw", WIDTH,HEIGHT, streamer));
		    //	node.addSubscriber(new MavJROSDepthSubscriber(model,"/camera/depth_aligned_to_color_and_infra1/image_raw", WIDTH,HEIGHT, streamer));
		    	       
	        	node.addSubscriber(new MavJROSDepthMappingSubscriber(model,commander.getAutopilot().getMapper().getShorTermMap(),
	        			"/camera/depth_aligned_to_color_and_infra1/image_raw", WIDTH,HEIGHT, streamer));
	        	try {
					node.connect();
					model.vision.setStatus(Vision.VIDEO_ENABLED, true);
				} catch (URISyntaxException e) { }
		    	
		    	break;
			}
			streamer.enableStream("RGB+DOWN");
		}

		wq.addCyclicTask("LP", 200,  console);
		wq.addCyclicTask("LP", 500,  hw);
		wq.addSingleTask("LP", 100,  new initPX4());
		wq.start();
		
		control.start();

		logger.writeLocalMsg("MSP (Version: "+config.getVersion()+") in mode "+mode+" started");
	}

	public static void main(String[] args)  {
		System.setProperty("sun.java2d.opengl", "false");
		System.setProperty("sun.java2d.xrender", "false");
		
		new StartUp(args);

	}

	private void addShutdownHook() {

		Runtime.getRuntime().addShutdownHook(new Thread() {
			public void run() {
				
				if(node!=null)
					node.shutdown();

				control.shutdown();
				//wq.printStatus();
				wq.stop();
				
				if(ftpServer!=null)
				   ftpServer.stop();

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

	private void registerCommands() {

		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {


				msg_msp_command cmd = (msg_msp_command)o;
				switch(cmd.command) {
				case MSP_CMD.MSP_TRANSFER_MICROSLAM:
					commander.getAutopilot().getMapper().invalidate_map_transfer();
					break;
				case MSP_CMD.SELECT_VIDEO_STREAM:

					if(streamer==null)
						return;

					switch((int)cmd.param1) {
					case 0:
						streamer.enableStream("RGB+DOWN");
						break;
					case 1:
						streamer.enableStream("DOWN+RGB");
						break;
					case 2:
						streamer.enableStream("DEPTH+RGB");
						break;
					case 3:
						streamer.enableStream("RGB"); 
						break;
					case 4:
						streamer.enableStream("DOWN");

					}
					break;
				}
			}
		});
	}

	private void registerActions() {

		// Switch to down view when precision landing
		control.getStatusManager().addListener(StatusManager.TYPE_PX4_NAVSTATE, Status.NAVIGATION_STATE_AUTO_PRECLAND, (n) -> {
			if(n.isNavState(Status.NAVIGATION_STATE_AUTO_PRECLAND) && pose !=null) 
				streamer.enableStream("DOWN+RGB");	
			stream_auto_switched = true;
		});

		// switch back to FPV is previsously switched to down view
		control.getStatusManager().addListener(StatusManager.TYPE_MSP_STATUS,Status.MSP_ARMED, StatusManager.EDGE_FALLING, (n) -> {
			if(stream_auto_switched) {
				stream_auto_switched = false;
				if(streamer!=null)
					streamer.enableStream("RGB+DOWN");	
			}
		});
	}
	

	private class initPX4 implements Runnable {


		@Override 
		public void run() {
			if((mode==MAVController.MODE_NORMAL || mode==MAVController.MODE_USB)  && model.sys.isStatus(Status.MSP_CONNECTED)) {
				System.out.println("Execute init PX4");

				// enforce NUTTX RTC set to companion time
				SimpleDateFormat sdf = new SimpleDateFormat("MMM dd HH:mm:ss YYYY", Locale.ENGLISH);   
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
			File file = new File(MSPFileUtils.getJarContainingFolder(this.getClass())+"/msp.log");

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

