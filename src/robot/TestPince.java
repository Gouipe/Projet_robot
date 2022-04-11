package robot;

import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;

public class TestPince {

	public static void main(String[] args) {
		EV3MediumRegulatedMotor pince = new EV3MediumRegulatedMotor(MotorPort.C);
		
		pince.forward();
		Delay.msDelay(1000);
		pince.stop(); 
		
		pince.backward();
		Delay.msDelay(3000); 
		
		pince.stop(); 
		pince.close();
	}

}
