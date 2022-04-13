package projet;

import java.util.ArrayList;
import java.util.ListIterator;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class Main {

	public static void main(String[] args) {
		/********* On instancie capteurs et moteurs ****************/

		// On récupère les moteurs
		EV3LargeRegulatedMotor mLeftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor mRightMotor = new EV3LargeRegulatedMotor(MotorPort.D);
		EV3MediumRegulatedMotor pinces = new EV3MediumRegulatedMotor(MotorPort.C);
		pinces.setSpeed(1080);
		// On instancie le pilot
		MonPilot cedric = new MonPilot(560, 1045, mLeftMotor, mRightMotor, pinces);
		// On règle ses vitesses
		cedric.setAngularSpeed(120); // on set la vitesse de rotation (degre/seconde)
		cedric.setLinearSpeed(800); // vitesse de déplacement (mm/seconde?)
		
		// On récupère un ultrasonic sensor
		Port port = LocalEV3.get().getPort("S2");
		EV3UltrasonicSensor soundSensor = new EV3UltrasonicSensor(port);
		// On récupère un touch sensor
		port = LocalEV3.get().getPort("S1");
		TouchSensor touchSensor = new TouchSensor(port);
		// On instancie capteurs
		Capteurs capteurs = new Capteurs(soundSensor, touchSensor);

		/************************************************************/

		Nettoyage nettoyage = new Nettoyage(cedric, capteurs);
		
		//nettoyage.run();
		nettoyage.test();
		//nettoyage.test2();
	}

}
