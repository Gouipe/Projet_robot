package robot;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class TestPilot {

	public static void main(String[] args) {
		EV3LargeRegulatedMotor mLeftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor mRightMotor = new EV3LargeRegulatedMotor(MotorPort.D);
		MovePilot cedric = new MovePilot(29, 91, mLeftMotor, mRightMotor);
	
		cedric.forward();
		Delay.msDelay(2000);
	}

}
