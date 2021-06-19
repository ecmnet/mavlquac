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

package com.comino.mavlquac.console;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;
import java.util.concurrent.Executors;
import java.util.function.Consumer;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavutils.workqueue.WorkQueue;

public class Console implements Runnable {



	private final IMAVController control;
	private final BufferedReader br;

	private Map<String,IConsolePrint> cmds = new HashMap<String,IConsolePrint>();

	public Console(IMAVController control) {
		this.control = control;	
		this.br = new BufferedReader(new InputStreamReader(System.in));
	}

	public void registerCmd(String cmd, IConsolePrint printer) {
		cmds.put(cmd, printer);
	}

	@Override
	public void run() {
		try {
			if(br.ready()) {
				parseConsole(br.readLine().trim().toLowerCase());
			}
		} catch (IOException e) { }
	}

	private void parseConsole(String s) {

		if(s.isEmpty()) {
			return;
		}

		if(s.contains("exit")) {
			System.exit(-1);
			return;
		}

		// Workqueue status
		if(s.contains("wq")) {
			WorkQueue.getInstance().printStatus();
			return;
		}

		// Status
		if(s.contains("st")) {
			System.out.println(control.getCurrentModel().sys.toString());
			return;
		}

		// Status
		if(s.contains("nav")) {
			System.out.println(control.getCurrentModel().sys.getModeString());
			return;
		}

		// Parameters
		if(s.contains("pa")) {
			System.out.println(PX4Parameters.getInstance().toString());
			return;
		}

		// Vision flags
		if(s.contains("vi")) {
			System.out.println(control.getCurrentModel().vision.toString());
			return;
		}

		IConsolePrint p = cmds.get(s);
		if(p!=null) {
			p.print();
			return;
		}

		//		// otherwise execute it as OS level command
		//		if(!control.getCurrentModel().sys.isStatus(Status.MSP_ARMED)) {
		//			executeOSCommand(s);
		//		}		
	}


	//	private void executeOSCommand(String command) {
	//		try {
	//			Process process;
	//			process = Runtime.getRuntime().exec(command);
	//			StreamGobbler streamGobbler = 
	//					new StreamGobbler(process.getInputStream(), System.out::println);
	//			Executors.newSingleThreadExecutor().submit(streamGobbler);
	//			int exitCode = process.waitFor();
	//			assert exitCode == 0;	
	//		} catch (Exception e) {
	//			MSPLogger.getInstance().writeLocalMsg("OS command '"+command+"' failed: "+e.getMessage(),
	//					MAV_SEVERITY.MAV_SEVERITY_WARNING);
	//		}
	//	}

	private static class StreamGobbler implements Runnable {
		private InputStream inputStream;
		private Consumer<String> consumer;

		public StreamGobbler(InputStream inputStream, Consumer<String> consumer) {
			this.inputStream = inputStream;
			this.consumer = consumer;
		}

		@Override
		public void run() {
			new BufferedReader(new InputStreamReader(inputStream)).lines()
			.forEach(consumer);
		}
	}

}
