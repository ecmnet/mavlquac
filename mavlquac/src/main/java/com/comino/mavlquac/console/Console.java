package com.comino.mavlquac.console;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

import com.comino.mavutils.workqueue.WorkQueue;

public class Console implements Runnable {

	@Override
	public void run() {
		BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
		try {
			if(br.ready()) {
			  parseConsole(br.readLine());
			}
		} catch (IOException e) { }
	}
	
	private void parseConsole(String s) {
		if(s.contains("status")) {
			WorkQueue.getInstance().printStatus();
		} 
	}

}
