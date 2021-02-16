package com.comino.mavlquac.dispatcher;

import org.mavlink.messages.lquac.msg_debug_vect;
import org.mavlink.messages.lquac.msg_msp_micro_grid;
import org.mavlink.messages.lquac.msg_msp_micro_slam;
import org.mavlink.messages.lquac.msg_msp_status;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavutils.hw.HardwareAbstraction;

public class MAVLinkDispatcher {

	private long last_call_10  = 0;
	private long last_call_50  = 0;
	private long last_call_200 = 0;
	private long last_call_500 = 0;

	private final DataModel        model;
	private final IMAVController control;
	private final MSPConfig       config;
	
	private final HardwareAbstraction hw;

	private final msg_msp_micro_grid  grid     = new msg_msp_micro_grid(2,1);
	private final msg_msp_status      msg      = new msg_msp_status(2,1);
	private final msg_debug_vect      debug    = new msg_debug_vect(2,1);
	private final msg_msp_micro_slam  slam     = new msg_msp_micro_slam(2,1);

	private boolean publish_microslam;
	private boolean publish_microgrid;
	private boolean publish_debug;


	public MAVLinkDispatcher(IMAVController control, MSPConfig config, HardwareAbstraction hw) {
		this.model   = control.getCurrentModel();
		this.control = control;
		this.config  = config;
		this.hw      = hw;

		this.publish_microgrid = config.getBoolProperty("publish_microsgrid", "true");
		System.out.println("[vis] Publishing microGrid enabled: "+publish_microgrid);

		this.publish_microslam = config.getBoolProperty("publish_microslam", "true");
		System.out.println("[vis] Publishing microSlam enabled: "+publish_microslam);

		this.publish_debug = config.getBoolProperty("publish_debug", "true");
		System.out.println("[vis] Publishing debug messages enabled: "+publish_debug);
	}


	public void dispatch(long tms) {

		if((System.currentTimeMillis()-last_call_10)>10) {
			dispatch_10ms();
			last_call_10 = System.currentTimeMillis();
		}

		if((System.currentTimeMillis()-last_call_50)>50) {
			dispatch_50ms();
			last_call_50 = System.currentTimeMillis();
		}

		if((System.currentTimeMillis()-last_call_200)>200) {
			dispatch_200ms();
			last_call_200 = System.currentTimeMillis();
		}

		if((System.currentTimeMillis()-last_call_500)>500) {
			dispatch_500ms();
			last_call_500 = System.currentTimeMillis();
		}

	}

	private void dispatch_10ms() {


		// Publish grid
		if(publish_microgrid && model.grid.hasTransfers()) {
			if(model.grid.toArray(grid.data)) {
				grid.cx  = model.grid.ix;
				grid.cy  = model.grid.iy;
				grid.cz  = model.grid.iz;
				grid.tms = DataModel.getSynchronizedPX4Time_us();
				grid.count = model.grid.count;
				control.sendMAVLinkMessage(grid);
			}
		}
	}

	private void dispatch_50ms() {

		// Debug vector
		if(publish_debug) {
			debug.x = model.debug.x;
			debug.y = model.debug.y;
			debug.z = model.debug.z;
			control.sendMAVLinkMessage(debug);
		}

		// Publish SLAM data
		if(publish_microslam) {
			slam.pd = model.slam.pd;
			slam.pp = model.slam.pp;
			slam.pv = model.slam.pv;
			slam.px = model.slam.px;
			slam.py = model.slam.py;
			slam.pz = model.slam.pz;
			slam.md = model.slam.di;
			slam.mw = model.slam.dw;
			slam.dm = model.slam.dm;
			slam.ox = model.slam.ox;
			slam.oy = model.slam.oy;
			slam.oz = model.slam.oz;
			slam.quality = model.slam.quality;
			slam.wpcount = model.slam.wpcount;
			slam.flags = model.slam.flags;
			slam.fps = model.slam.fps;
			slam.tms = model.slam.tms;
			control.sendMAVLinkMessage(slam);
		}

	}


	private void dispatch_200ms() {

		msg.load = hw.getCPULoad();
		msg.memory = hw.getMemoryUsage();
		msg.wifi_quality = hw.getWifiQuality();
		msg.threads = Thread.activeCount();
		msg.cpu_temp = (byte)hw.getCPUTemperature();
		msg.bat_temp = (byte)hw.getBatteryTemperature();
		msg.com_error = control.getErrorCount();
		msg.takeoff_ms = model.sys.t_takeoff_ms;
		msg.autopilot_mode =control.getCurrentModel().sys.autopilot;
		msg.uptime_ms = model.sys.t_boot_ms;
		msg.status = control.getCurrentModel().sys.getStatus();
		msg.setVersion(config.getVersion()+"/"+config.getVersionDate().replace(".", ""));
		msg.setArch(hw.getArchName());
		msg.unix_time_us = System.currentTimeMillis() * 1000;
		control.sendMAVLinkMessage(msg);

	}

	private void dispatch_500ms() {

	}

}
