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

package com.comino.mavlquac.dispatcher;

import org.mavlink.messages.lquac.msg_debug_vect;
import org.mavlink.messages.lquac.msg_msp_micro_grid;
import org.mavlink.messages.lquac.msg_msp_micro_slam;
import org.mavlink.messages.lquac.msg_msp_status;
import org.mavlink.messages.lquac.msg_msp_vision;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavutils.hw.HardwareAbstraction;

public class MAVLinkDispatcher implements Runnable {

	private long last_call_10  = 0;
	private long last_call_50  = 0;
	private long last_call_200 = 0;
	private long last_call_500 = 0;

	private final DataModel        model;
	private final IMAVController control;
	private final MSPConfig       config;
	
	private final HardwareAbstraction hw;

	private final msg_msp_micro_grid  grid     = new msg_msp_micro_grid(2,1);
	private final msg_msp_status      status   = new msg_msp_status(2,1);
	private final msg_debug_vect      debug    = new msg_debug_vect(2,1);
	private final msg_msp_micro_slam  slam     = new msg_msp_micro_slam(2,1);
	private final msg_msp_vision      vis      = new msg_msp_vision(2,1);

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


	public void run() {

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
		if(publish_microslam && model.slam.quality > 0) {
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

		status.load = hw.getCPULoad();
		status.memory = hw.getMemoryUsage();
		status.wifi_quality = hw.getWifiQuality();
		status.threads = Thread.activeCount();
		status.cpu_temp = (byte)hw.getCPUTemperature();
		status.bat_temp = (byte)hw.getBatteryTemperature();
		status.com_error = control.getErrorCount();
		status.takeoff_ms = model.sys.t_takeoff_ms;
		status.autopilot_mode =control.getCurrentModel().sys.autopilot;
		status.uptime_ms = model.sys.t_boot_ms;
		status.status = control.getCurrentModel().sys.getStatus();
		status.setVersion(config.getVersion()+"/"+config.getVersionDate().replace(".", ""));
		status.setArch(hw.getArchName());
		status.unix_time_us = System.currentTimeMillis() * 1000;
		control.sendMAVLinkMessage(status);

	}

	private void dispatch_500ms() {

	}

}
