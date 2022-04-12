package cedric;

import java.util.ArrayList;
import java.util.ListIterator;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class Main {

	public static void main(String[] args) {
		/********* On instancie un pilot pour piloter cedric ****************/
		
		// On récupère les moteurs
		EV3LargeRegulatedMotor mLeftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor mRightMotor = new EV3LargeRegulatedMotor(MotorPort.D);
		EV3MediumRegulatedMotor pinces = new EV3MediumRegulatedMotor(MotorPort.C);

		// On récupère un ultrasonic sensor
		Port port = LocalEV3.get().getPort("S2");
		EV3UltrasonicSensor soundSensor = new EV3UltrasonicSensor(port);

		// On instancie le pilot
		MonPilot cedric = new MonPilot(56, 105, mLeftMotor, mRightMotor, pinces);
		// On règle ses vitesses
		cedric.setAngularSpeed(90); // on set la vitesse de rotation (degre/seconde)
		cedric.setLinearSpeed(90); // vitesse de déplacement (mm/seconde?)
		// On instancie capteurs
		Capteurs capteurs = new Capteurs(soundSensor);

		/************************************************************/

		Nettoyage nettoyage = new Nettoyage(cedric, capteurs);
		nettoyage.run();
	}

}
