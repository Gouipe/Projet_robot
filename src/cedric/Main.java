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
		//Button.ENTER.waitForPressAndRelease();
		final double DISTANCE_MIN_PALET = 31.0;

		
		// On instancie un pilot pour le robot
		EV3LargeRegulatedMotor mLeftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor mRightMotor = new EV3LargeRegulatedMotor(MotorPort.D);
		MovePilot cedric = new MovePilot(56, 105, mLeftMotor, mRightMotor);
		cedric.setAngularSpeed(90); // on set la  vitesse de rotation (degre/seconde)
		cedric.setLinearSpeed(90); // vitesse de d�placement (mm/seconde?)
		
		// on ferme les pinces (il faut v�rifier si les pinces sont d�j� ferm�es)
		/*
		 * EV3MediumRegulatedMotor pince = new EV3MediumRegulatedMotor(MotorPort.C);
		 * pince.forward(); Delay.msDelay(4000);
		 */
		

		//On r�cup�re un ultrasonic sensor avec un sample provider pour d�tecter les objets
		Port port = LocalEV3.get().getPort("S2");
		EV3UltrasonicSensor objectSensor = new EV3UltrasonicSensor(port);
		SampleProvider sampleProvider = objectSensor.getDistanceMode();
		
		
		// cedric tourne sur lui meme sur 360 degres et r�cup�re tout du long les distances
		int nbMesures=0;
		ArrayList<Float> distances = new ArrayList<Float>();
		cedric.rotate(360, true);
		while (cedric.isMoving()) {
			float[] sample = new float[sampleProvider.sampleSize()];
			sampleProvider.fetchSample(sample, 0);
			distances.add(sample[0]);
			nbMesures++;
		}	
		
		//TESTS///////
		double degreParMesure = 360.0 / nbMesures;
		System.out.println("nbMesures : "  + nbMesures);
		System.out.println("taille distance : " + distances.size());
		System.out.println("degreParMesure : " + degreParMesure);
		Button.ENTER.waitForPressAndRelease();

		System.out.println("0 : " + distances.get(0));
		Button.ENTER.waitForPressAndRelease();
		
		System.out.println("90 : " + distances.get(distances.size()/4));
		Button.ENTER.waitForPressAndRelease();
		
		System.out.println("180 : " + distances.get(distances.size()/2));
		Button.ENTER.waitForPressAndRelease();
		
		System.out.println("270 : " + distances.get(distances.size() * 3 / 4 ));
		Button.ENTER.waitForPressAndRelease();
		///////////
		
		//On r�cup�re dans min la plus petite distance qui peut correspondre � un palet
		float min = distances.get(0);// initialisation
		int quantiemeMesure = 0; // nombre de mesures faites avant la mesure min
		ListIterator<Float> it = distances.listIterator();
		float courant;
		while (it.hasNext()) {
			courant = it.next();
			if (courant < min && courant > DISTANCE_MIN_PALET) { //courant est une distance qui peut correspondre � un palet
				min = courant;
				quantiemeMesure = it.nextIndex(); //l'index corresond au nombre de mesures / � la combientieme mesure
			}
		}
		
		// cedric rotate vers la direction qui correpond � la distance min
		cedric.rotate(degreParMesure * quantiemeMesure); //(degre/nbmesure) * nbmesures => degre
		
		//tant que la distance mesur�e est > � 31cm (- quelques cm par s�curit�), cedric avance
		
		//Si la distance mesur�e passe de 31 cm � 0 : on sait qu'il y a un palet, on le r�cup�re 
		
		objectSensor.close();
	}

}
