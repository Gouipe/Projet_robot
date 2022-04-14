package projet;

import java.util.ArrayList;
import java.util.ListIterator;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class MonPilot extends MovePilot {

	// indique si les pinces sont ouvertes ou fermées. C'est une simple
	// sécurité supplémentaire, pour ne pas appeler la même méthode 2 fois de suite
	private boolean ouvertes; 
						
	EV3MediumRegulatedMotor pinces;
	private double angleAbsoluCourant = 0.0;


	/**
	 * @param wheelDiameter
	 * @param trackWidth
	 * @param leftMotor
	 * @param rightMotor
	 * @param pinces        Crée un pilote pour un robot avec pinces qui doivent
	 *                      être fermées à l'instanciation.
	 */
	public MonPilot(double wheelDiameter, double trackWidth, RegulatedMotor leftMotor, RegulatedMotor rightMotor,
			EV3MediumRegulatedMotor pinces) {
		super(wheelDiameter, trackWidth, leftMotor, rightMotor);
		this.pinces = pinces;
		ouvertes = false; // pinces doivent être fermées au début
	}

	/**
	 * Verifie si les pinces sont ouvertes, auquel cas ferme les pinces et change
	 * l'attribut ouvertes
	 */
	public void fermePinces() {
		//if (ouvertes) {
			pinces.backward();
			Delay.msDelay(2000); 
			pinces.stop();
		//}
		ouvertes = false;
	}

	/**
	 * Verifie si les pinces sont fermées, auquel cas ouvre les pinces et change
	 * etatPinces.
	 */
	public void ouvrePinces() {
		//if (!ouvertes) {
			pinces.forward();
			Delay.msDelay(2000);
			pinces.stop();
		//}
		ouvertes = true;
	}
	
	/*
	public void rotate(double angle, boolean immediateReturn) {
		super.rotate(angle, immediateReturn);
		angleAbsoluCourant = (angleAbsoluCourant + angle) % 360;
		System.out.println("rotate1:" + angleAbsoluCourant);
		//Button.ENTER.waitForPressAndRelease();
	}
	*/
	
	public void rotate(double angle) {
		super.rotate(angle);
		angleAbsoluCourant = (angleAbsoluCourant + angle) % 360;
		//Button.ENTER.waitForPressAndRelease();
	}
	
	
	public void rotateWhileHoldingPalet(double angle, boolean immediateReturn) {
		super.rotate(angle, immediateReturn);
		angleAbsoluCourant = (angle * 0.95 + angleAbsoluCourant) % 360; 
		//on enlève quelques % de l'angle théorique
		// car le robot tourne un peu moins que prévu quand il tient un palet
		//Button.ENTER.waitForPressAndRelease();
	}
	
	public void rotateToGoal() {
		rotateWhileHoldingPalet(-angleAbsoluCourant, true);
	}

}
