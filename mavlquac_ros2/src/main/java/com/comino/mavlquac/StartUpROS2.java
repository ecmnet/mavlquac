/****************************************************************************
 *
 *   Copyright (c) 2025 Eike Mansfeld ecm@gmx.de. All rights reserved.
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

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.control.impl.MAVController;
import com.comino.mavcom.control.impl.MAVProxyController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavlquac.dispatcher.MAVLinkDispatcher;
import com.comino.mavros2bridge.msp.MSPROS2Node;
import com.comino.mavutils.file.MSPFileUtils;
import com.comino.mavutils.legacy.ExecutorService;
import com.comino.mavutils.workqueue.WorkQueue;

public class StartUpROS2  {

	private final WorkQueue wq = WorkQueue.getInstance();

	IMAVMSPController     comm       = null;
	MSPConfig	          config     = null;
	MAVLinkDispatcher     dispatcher = null;
	MSPROS2Node           ros2       = null;
	

	private MSPLogger logger;


	public StartUpROS2(String[] args) {

		addShutdownHook();

		ExecutorService.create();
		
		config  = MSPConfig.getInstance(MSPFileUtils.getJarContainingFolder(this.getClass()),"msp.properties");
		System.out.println("MSPLquac-ROS2 (LQUAC build) version "+config.getVersion());

		comm = new MAVProxyController(MAVController.MODE_ORIN,config);
		logger = MSPLogger.getInstance(comm);
		logger.enableDebugMessages(true);

		PX4Parameters.getInstance(comm);
		System.out.println(comm.getStatusManager().getSize()+" status events registered");
	
		
		ros2 = new MSPROS2Node(comm);
		ros2.setup();

		wq.start();
		comm.start();

		logger.writeLocalMsg("MSP (Version: "+config.getVersion()+") started");
		
		dispatcher = new MAVLinkDispatcher(comm, config, null);
		dispatcher.start();
		
		
	}

	public static void main(String[] args)  {
		System.setProperty("sun.java2d.opengl", "false");
		System.setProperty("sun.java2d.xrender", "false");

		new StartUpROS2(args);

	}

	private void addShutdownHook() {

		Runtime.getRuntime().addShutdownHook(new Thread() {
			public void run() {

				comm.shutdown();
				wq.stop();
			}

		});

	}



}

