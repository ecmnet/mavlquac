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

import java.time.Instant;

import org.mavlink.messages.lquac.msg_debug_vect;
import org.mavlink.messages.lquac.msg_msp_micro_grid;
import org.mavlink.messages.lquac.msg_msp_micro_slam;
import org.mavlink.messages.lquac.msg_msp_status;
import org.mavlink.messages.lquac.msg_msp_trajectory;
import org.mavlink.messages.lquac.msg_system_time;
import org.mavlink.messages.lquac.msg_timesync;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Slam;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavutils.hw.HardwareAbstraction;
import com.comino.mavutils.workqueue.WorkQueue;

public class MAVLinkDispatcher  {

	private final DataModel        model;
	private final IMAVController control;
	private final MSPConfig       config;

	private final HardwareAbstraction hw;

	private final msg_msp_status      status   = new msg_msp_status(2,1);
	private final msg_debug_vect      debug    = new msg_debug_vect(2,1);
	private final msg_msp_micro_slam  slam     = new msg_msp_micro_slam(2,1);
	private final msg_msp_trajectory  traj 	   = new msg_msp_trajectory(2,1);

	private boolean publish_microslam;
	private boolean publish_debug;

	private long    init_tms = System.currentTimeMillis();

	// Rework using WorkQueue!
	private final WorkQueue wq = WorkQueue.getInstance();


	public MAVLinkDispatcher(IMAVController control, MSPConfig config, HardwareAbstraction hw) {
		this.model   = control.getCurrentModel();
		this.control = control;
		this.config  = config;
		this.hw      = hw;

		this.publish_microslam = config.getBoolProperty(MSPParams.PUBLISH_MICROSLAM, "true");
		System.out.println("[vis] Publishing microSlam enabled: "+publish_microslam);

		this.publish_debug = config.getBoolProperty(MSPParams.PUBLISH_DEBUG, "true");
		System.out.println("[vis] Publishing debug messages enabled: "+publish_debug);

		wq.addCyclicTask("NP", 20,  new Dispatch_20ms());
		wq.addCyclicTask("NP", 50,  new Dispatch_50ms());
		wq.addCyclicTask("NP", 100, new Dispatch_100ms());
		wq.addCyclicTask("NP", 200, new Dispatch_200ms());
		wq.addCyclicTask("LP", 500, new Dispatch_500ms());
	}

	private class Dispatch_20ms implements Runnable {
		@Override
		public void run() {

			// Debug vector
			if(publish_debug) {
				debug.x = model.debug.x;
				debug.y = model.debug.y;
				debug.z = model.debug.z;
				debug.time_usec = DataModel.getSynchronizedPX4Time_us();
				control.sendMAVLinkMessage(debug);
			}

		}
	}

	private class Dispatch_50ms implements Runnable {
		@Override
		public void run() {



		}
	}


	private class Dispatch_100ms implements Runnable {
		@Override
		public void run() {

			// Publish SLAM data
			if(publish_microslam && ( model.slam.fps > 0 || control.isSimulation())) {
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

			// Trajectory publishing	

			if(model.slam.flags == Slam.OFFBOARD_FLAG_MOVE) {

				traj.ls = model.traj.ls;
				traj.fs = model.traj.fs;

				traj.ax = model.traj.ax;
				traj.ay = model.traj.ay;
				traj.az = model.traj.az;
				traj.bx = model.traj.bx;
				traj.by = model.traj.by;	
				traj.bz = model.traj.bz;	
				traj.gx = model.traj.gx;
				traj.gy = model.traj.gy;	
				traj.gz = model.traj.gz;	
				traj.sx = model.traj.sx;
				traj.sy = model.traj.sy;	
				traj.sz = model.traj.sz;	
				traj.svx = model.traj.svx;
				traj.svy = model.traj.svy;
				traj.svz = model.traj.svz;

				traj.tms = model.traj.tms;

				control.sendMAVLinkMessage(traj);

			}

		}
	}

	private class Dispatch_200ms implements Runnable {
		@Override
		public void run() {
			
			model.sys.setStatus(Status.MSP_ACTIVE,true);
			model.sys.wifi_quality = hw.getWifiQuality()/100f;

			status.load = hw.getCPULoad();
			status.memory = hw.getMemoryUsage();
			status.wifi_quality = hw.getWifiQuality();
			status.threads = Thread.activeCount();
			status.cpu_temp = (byte)hw.getCPUTemperature();
			status.bat_temp = (byte)hw.getBatteryTemperature();
			status.com_error = control.getErrorCount();
			status.takeoff_ms = model.sys.t_takeoff_ms;
			status.autopilot_mode =control.getCurrentModel().sys.autopilot;
			if(model.sys.t_boot_ms > 0)
				status.uptime_ms = model.sys.t_boot_ms;
			else
				status.uptime_ms = System.currentTimeMillis() - init_tms;
			status.status = control.getCurrentModel().sys.getStatus();
			status.setVersion(config.getVersion()+"/"+config.getVersionDate().replace(".", ""));
			status.setArch(hw.getArchName());
			status.unix_time_us = DataModel.getUnixTime_us();
			control.sendMAVLinkMessage(status);
			

		}
	}

	private class Dispatch_500ms implements Runnable {
		@Override
		public void run() {


		}

	}

}
