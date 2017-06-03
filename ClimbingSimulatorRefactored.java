package com.craigcode.climbing_simulator_refactored;

import java.awt.*;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

public class ClimbingSimulatorRefactored extends JFrame {

	private static final long serialVersionUID = 1L;

	public static void main (String[] args) {	
		SwingUtilities.invokeLater(new Runnable () {
			public void run() {
				createAndShowWall();
			}
		});
	}
	
	private static void createAndShowWall () {
		ClimbingSimulatorRefactored cSR = new ClimbingSimulatorRefactored();
		cSR.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		
		ClimbingWall cw = new ClimbingWall();
		
		cSR.add(cw);
		cSR.pack();
		cSR.setVisible(true);
		cw.requestFocusInWindow();
	}
}
