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

	EV3MediumRegulatedMotor pinces;
	double angleAbsoluCourant = 0.0;

	/**
	 * @param wheelDiameter
	 * @param trackWidth
	 * @param leftMotor
	 * @param rightMotor
	 * @param pinces        Crée un pilote pour un robot avec pinces
	 */
	public MonPilot(double wheelDiameter, double trackWidth, RegulatedMotor leftMotor, RegulatedMotor rightMotor,
			EV3MediumRegulatedMotor pinces) {
		super(wheelDiameter, trackWidth, leftMotor, rightMotor);
		this.pinces = pinces;
	}

	/**
	 * Ferme les pinces
	 */
	public void fermePinces() {
		pinces.backward();
		Delay.msDelay(1000); 
		pinces.stop();
	}

	/**
	 *Ouvre les pinces
	 */
	public void ouvrePinces() {
		pinces.forward();
		Delay.msDelay(1000);
		pinces.stop();
	}
	
	public void rotate(double angle, boolean immediateReturn) {
		super.rotate(angle, immediateReturn);
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
		System.out.println("rotate to goal");
		if (angleAbsoluCourant < 180)
			rotate(-angleAbsoluCourant, false);
		else
			rotate(- angleAbsoluCourant + 360, false);
	}

}
