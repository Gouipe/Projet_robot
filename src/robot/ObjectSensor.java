package robot;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class ObjectSensor {
	public static void main(String[] args) {
		/*
		TestMoteur roues = new TestMoteur(MotorPort.A,MotorPort.D);
		roues.forward();

		Delay.msDelay(10000);

		roues.stop(); 
		*/
		Port port = LocalEV3.get().getPort("S2");
		EV3UltrasonicSensor objectSensor = new EV3UltrasonicSensor(port);
		
		SampleProvider sampleProvider = objectSensor.getDistanceMode();
		for (int i = 0 ; i < 100 ; i++) {
			float[] sample = new float[sampleProvider.sampleSize()];
			sampleProvider.fetchSample(sample, 0);
			System.out.println(sample[0]);
			Delay.msDelay(500);
		}
		objectSensor.close();
		
		

	}

}
