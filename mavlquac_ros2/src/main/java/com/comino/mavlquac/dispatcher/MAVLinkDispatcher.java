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

import org.mavlink.messages.LANDING_TARGET_TYPE;
import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.lquac.msg_debug_vect;
import org.mavlink.messages.lquac.msg_landing_target;
import org.mavlink.messages.lquac.msg_msp_local_position_corrected;
import org.mavlink.messages.lquac.msg_msp_micro_grid;
import org.mavlink.messages.lquac.msg_msp_micro_slam;
import org.mavlink.messages.lquac.msg_msp_obstacle;
import org.mavlink.messages.lquac.msg_msp_status;
import org.mavlink.messages.lquac.msg_msp_trajectory;
import org.mavlink.messages.lquac.msg_msp_vision;

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

	// Messages to GC
	private final msg_msp_status      				status   = new msg_msp_status(2,1);
	private final msg_msp_vision                    vis      = new msg_msp_vision(2,1);

	// Messages to PX4
	private final msg_landing_target                landing  = new msg_landing_target(1,1);

	// Rework using WorkQueue!
	private final WorkQueue wq = WorkQueue.getInstance();


	public MAVLinkDispatcher(IMAVController control, MSPConfig config, HardwareAbstraction hw) {
		this.model   = control.getCurrentModel();
		this.control = control;
		this.config  = config;
		this.hw      = hw;

	}
	
	public void start() {
		wq.addCyclicTask("NP", 20,  new Dispatch_20ms());
		wq.addCyclicTask("NP", 50,  new Dispatch_50ms());
		wq.addCyclicTask("NP", 100, new Dispatch_100ms());
		wq.addCyclicTask("NP", 200, new Dispatch_200ms());
		wq.addCyclicTask("LP", 500, new Dispatch_500ms());
	}

	private class Dispatch_20ms implements Runnable {
		@Override
		public void run() {

		}
	}

	private class Dispatch_50ms implements Runnable {
		@Override
		public void run() {

			vis.x       = model.vision.x;
			vis.y       = model.vision.y;
			vis.z       = model.vision.z;
			
			vis.vx      = model.vision.vx;
			vis.vy      = model.vision.vy;
			vis.vz      = model.vision.vz;
			
			vis.tms     = model.vision.tms;
			vis.flags   = model.vision.flags;
			vis.fps     = model.vision.fps;
			
			vis.errors  = model.vision.errors;
			vis.quality = (int)( 100 * model.vision.qual);

			control.sendMAVLinkMessage(vis);
		}
	}
	


	private class Dispatch_100ms implements Runnable {

		@Override
		public void run() {

		}
	}

	private class Dispatch_200ms implements Runnable {
		@Override
		public void run() {


			if(hw !=null) {
				model.sys.wifi_quality = hw.getWifiQuality()/100f;
				status.load = hw.getCPULoad();
				status.memory = hw.getMemoryUsage();
				status.wifi_quality = hw.getWifiQuality();
				status.cpu_temp = (byte)hw.getCPUTemperature();
				status.setArch(hw.getArchName());
			}

			status.threads = Thread.activeCount();
			status.bat_type = (byte)model.sys.bat_type;
			status.com_error = control.getErrorCount();
			status.takeoff_ms = model.sys.t_takeoff_ms;
			status.autopilot_mode =control.getCurrentModel().sys.autopilot;
			status.uptime_ms = DataModel.getBootTime();
			status.status = control.getCurrentModel().sys.getStatus();
			status.sensors = control.getCurrentModel().sys.getSensors();
			status.setVersion(config.getVersion()+"/"+config.getVersionDate().replace(".", ""));
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
