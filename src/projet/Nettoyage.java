package projet;
//Button.ENTER.waitForPressAndRelease();


import java.util.ArrayList;
import java.util.HashMap;
import java.util.ListIterator;
import java.util.Map;
import java.util.TreeMap;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.MovePilot;
//Button.ENTER.waitForPressAndRelease();
import lejos.utility.Delay;

public class Nettoyage {

	final private double DISTANCE_MIN_PALET = 0.321; // distance pour laquelle le palet n'est plus d�tect�

	final private int SCAN_PALETS = 0;
	final private int DIRIGER_VERS_PALET = 1;
	final private int ATTRAPER_PALET = 2;
	final private int RANGER_PALET_ATTRAPE = 3;
	
	final private double ANG_SPEED_CHECK = 40.0;
	final private double ANG_SPEED_REGULAR = 80.0;

	MonPilot robot;
	Capteurs capteurs;
	private int etat; // Etat courant de l'automate

	public Nettoyage(MonPilot robot, Capteurs capteurs) {
		this.robot = robot;
		this.capteurs = capteurs;
		etat = SCAN_PALETS; // au d�but, on va chercher la position des palets
	}

	public void run() {
		etat = SCAN_PALETS ;
		attraperPremierPalet();
		//attraperDeuxiemePalet(); a voir
		do {
			switch (etat) {
			case SCAN_PALETS:
				// on ferme les pinces (il faut v�rifier si les pinces sont d�j� ferm�es)
				// robot.fermePinces();

				// on cherches les palets potentiels
				if (scanPalet() == 0)
					etat = DIRIGER_VERS_PALET;
				// Sinon on continue � scanner pour le moment
				break;

			case DIRIGER_VERS_PALET:
				if (dirigerVersPalet() == 0)
					etat = ATTRAPER_PALET;
				else
					etat = SCAN_PALETS;
				break;

			case ATTRAPER_PALET:
				if (attrapePalet() == 0) // si on attrape bien un palet, on va le ranger
					etat = RANGER_PALET_ATTRAPE;
				else
					// A VOIR : //TODO
					// sinon on verifie tr�s rapidement si jamais un palet a �t� attrap� par les
					// pinces
					// mais n'a pas touch� le balai du capteur de toucher
					etat = verifAttrapPalet();

					// version normale :
					//etat = SCAN_PALETS;
				break;

			case RANGER_PALET_ATTRAPE:
				rangePalet();
				// une fois que le palet est pos�, on cherche un nouveau palet
				etat = SCAN_PALETS;
				break;
			}
		} while (etat != RANGER_PALET_ATTRAPE);
	}

	/**
	 * Attend qu'on appuie sur un bouton pour attraper le premier palet.
	 * D�pose le premier le palet et s'oriente vers le deuxieme
	 */
	public void attraperPremierPalet() {
		robot.ouvrePinces();
		
		// Le robot attend qu'on touche un bouton pour s'activer
		System.out.println("Appuyez pour commencer");
		Button.ENTER.waitForPressAndRelease();
		// Petit delay pour pas que la pression quand on appuie sur le bouton fasse d�vier le robot
		Delay.msDelay(75);
		// On avance vers le palet le plus proche en face
		//robot.forward();
		// Quand le palet est touch� on ferme les pinces...
		//while (!capteurs.touchSensor.isPressed()) {}
		robot.travel(4500);
		/*
		// ... de mani�re asynchrone...
		FermePincesAsynch t = new FermePincesAsynch(robot);
		t.start();
		*/
		robot.stop();
		// ... ou synchrone.
		robot.fermePinces();

		//tourne vers sa gauche et avance un peu
		//robot.arc(2200, 70);
		robot.rotate(45);
		robot.travel(3100);
		//tourne vers la droite et avance un peu
		robot.rotate(-45);
		//avancer jusqu'� traverser ligne blanche
		robot.forward();
		while(!capteurs.ligneBlancheDetectee()) {}

		robot.stop();
		//d�poser palet
		robot.ouvrePinces();
		//recule de 15cm
		robot.travel(-1500);
		//ferme pinces ?
		robot.fermePinces();
	}
	
	public void attraperDeuxiemePalet() {
		//s'oriente vers le premier
		
		//s'oriente de ~150 degres vers la gauche
		
		//avance jusqu'� traverser ligne blanche (ou proche du mur)
		
		//d�pose le palet
		
	}

	/**
	 * Range le palet une fois q'il est attrap�
	 */
	public void rangePalet() {
		// TODO
		robot.rotateToGoal();
		robot.forward();
		while(!capteurs.ligneBlancheDetectee()) {}
		robot.stop();
		robot.ouvrePinces();
		//recule de 20cm
		robot.travel(-2000);
		// si distance mesur�e devient < 31 cm, on tourne de 70 degres ?

		// si on traverse ligne blanche, on pose le palet
		
	}

	/**
	 * Fait bouger le robot en direction du palet potentiel d�tect� auparavant
	 * 
	 * @return 0 si le robot vient de se positionner � 30 cm d'un palet (a priori),
	 *         1 si on �tait sur le point de toucher un mur(on a pas trouv� de
	 *         palet)
	 * 
	 */
	public int dirigerVersPalet() {
		robot.forward();
		// mesurer de mani�re asynchrone la distance courante
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();

		// variables pour sauvegarder les mesures
		float[] sample = new float[sampleProvider.sampleSize()];
		float previous = 10; // valeur absurde 10m
		float courant;
		// tant que la distance mesur�e ne r�augmente pas (ce serait le cas o� on a
		// pass� le palet et le mur
		// derri�re est alors mesur�), le robot avance
		do {
			// on mesure la distance
			sampleProvider.fetchSample(sample, 0);
			courant = sample[0];
			// on �tait tres proche de quelquechose (un palet peut �tre) et cette chose
			// n'est plus visible et courrant n'est pas infinity
			if (previous < 0.37 && courant / previous > 1.6 /* && !(courant > 2.3)*/) {
				robot.stop();
				// A priori, on est juste devant un palet
				System.out.println("devant un palet");//
				//System.out.println("courant : " + courant);
				//System.out.println("previous : " + previous);
				//System.out.println("pre/cour:" + courant / previous);
				return 0;
			}
			previous = courant;
			//System.out.println(courant);
			//Delay.msDelay(100);
			// on arr�te la boucle si on va toucher un mur
		} while (courant > 0.27);
		robot.stop();
		// on a loup� le palet

		System.out.println("c'etait un mur");
		return 1;

		// se recaliber si il faut (si distance mesur�e change brusquement hors cas <
		// 31cm)

	}

	/**
	 * Attrape un palet
	 * A utiliser quand le palet n'est plus d�tect� alors qu'on
	 * avance vers lui, le robot est alors � moins de 31 cm. Renvoie 0 si un palet
	 * est attrap�, renvoie 1 sinon
	 */
	public int attrapePalet() {
		System.out.println("attrapePalet()");
		// on ouvre les pinces pour attraper le palet
		robot.ouvrePinces();

		// on avance vers le palet
		robot.forward();

		// on mesure de mani�re asynchrone la distance de l'objet devant le robot
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();
		float[] sample = new float[sampleProvider.sampleSize()];
		float distanceMesuree = 99; // valeur absurde
		// on arr�te d'essayer d'attraper au bout de 8 secondes ou si on s'apprete �
		// toucher un mur
		long beginning = System.currentTimeMillis();
		while (System.currentTimeMillis() - beginning < 8000 && distanceMesuree > 0.23) {
			// le senseur a senti quelque chose (un palet normalement)
			if (capteurs.touchSensor.isPressed()) {
				robot.stop();
				robot.fermePinces();
				System.out.println("attraped"); // TEST
				robot.rotateToGoal();
				return 0;
			}
			sampleProvider.fetchSample(sample, 0);
			distanceMesuree = sample[0];
		}

		// On abandonne au bout de 8 secondes ou si le robot allait toucher un mur
		robot.stop();
		robot.fermePinces();
		System.out.println("louped"); // TEST
		return 1;
	}

	/**
	 * Verifie tr�s rapidement si jamais un palet a �t� attrap� par les pinces mais
	 * n'a pas touch� le balai du capteur de toucher. R�ouvre les pinces et avance
	 * d'un chouia. Si un palet �tait attrap�, le d�tecteur de toucher le d�tecte
	 * maintenant.
	 * 
	 * @return RANGER_PALET_ATTRAPE si il y avait un palet, SCAN_PALETS sinon
	 */
	public int verifAttrapPalet() {
		System.out.println("verifAttrPalet()"); // TEST
		robot.ouvrePinces();
		robot.travel(200, true); // on bouge de 2cm

		// on ouvre les pinces et on avance une seconde pour voir si un palet est
		// d�tect�
		long beginning = System.currentTimeMillis();
		while (System.currentTimeMillis() - beginning < 500) {

			// il y avait bien un palet attrap�
			if (capteurs.touchSensor.isPressed()) {
				robot.stop();
				robot.fermePinces();
				System.out.println("un palet finalement"); // TEST
				// on renvoie l'etat suivant
				return RANGER_PALET_ATTRAPE;
			}
		}
		// Pas de palet d�tect� :
		robot.stop();
		robot.fermePinces();
		System.out.println("pas de palet"); // TEST
		// Il faut en chercher un palet
		return SCAN_PALETS;
	}

	/**
	 * Scanne alentour pour chercher des palets. S'oriente alors vers le palet
	 * potentiel le plus proche. Renvoie 0 si un palet a �t� trouv�, renvoie 1 sinon
	 */
	public int scanPalet() {
		//TODO : Ne pas prendre en compte les murs
		
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();

		/********************** PRISE MESURES **************************/
		
		// On prend une mesure par degre en deleyant chaque mesure
		// en fonction de la vitesse de rotation
		int nbMesures = 0; // le nombre de mesures ne vaudra pas exactement 360
		ArrayList<Float> distances = new ArrayList<Float>(); // on r�cup�re les mesures dans une arraylist
		float[] sample = new float[sampleProvider.sampleSize()];
		robot.rotate(360, true);
		while (robot.isMoving()) {
			sampleProvider.fetchSample(sample, 0); // on fait une mesure
			//Si mesure vaut infinity, on la transforme en 2.5
			if (sample[0] > 2.5)
				distances.add((float) 2.5); // on l'ajoute � l'array list
			else
				distances.add(sample[0]);
			nbMesures++;
			// on d�laie la prochaine mesure de : temps total de rotation / nombre de
			// mesures voules
			// <=> (nombre de degres parcourus / vitesse de rotation) en ms / nombre de
			// mesures voules
			Delay.msDelay(1000 * 360 / ((int) robot.getAngularSpeed()) / 360);
		}
		/****************************************************************/
		
		/*********** RECUPERATION MINIMUM/VALEUR INTERESSANTE **********************/
		
		// On r�cup�re dans la variable min la plus petite distance qui peut
		// correspondre � un palet
		 
		float min = (float) 999.0; // Initialisation du min avec valeur absurde.
		int quantiemeMesure = -1; // Ordre de la mesure, valeur absurde au depart
		
		//Liste des min int�ressants pour ne garder que le plus petit par la suite
		TreeMap<Float, Integer> interessantsList = new TreeMap<Float, Integer>();
		float minInteressant = (float)999.0; // Pareil pour minInteressant...
		int quantiemeMesureInteressante = -1; // ... et son ordre de mesure.
		
		float courant; // Mesure courante dans le parcours de l'array list
		float previous = (float) 0.0; // Mesure pr�cedente
		for (int i = 0; i < distances.size(); i++) {
			courant = distances.get(i);
			// Si courant est une distance qui peut correspondre � un palet
			if (courant < min && courant > DISTANCE_MIN_PALET) { 												
				min = courant;
				quantiemeMesure = i; // l'index corresond au nombre de mesures / � la combientieme mesure
			}
			// Si la mesure a diminu� de 15cm ou plus 
			if (i > 0 && previous - courant > 0.30 && courant != 2.5 && previous != 2.5
					&& distances.get(i+1) / courant > 0.97 && distances.get(i+1) / courant < 1.03) {
				// on "initialise" minInteressant et son index/degre
				interessantsList.put(courant, i);
				System.out.println("courant :" + courant);
				System.out.println("previous :" + previous);
				Button.ENTER.waitForPressAndRelease();
			}
			previous = courant;
		}
		
		//RECUPERATION DU MIN INTERESSANT
		minInteressant = interessantsList.firstKey();
		quantiemeMesureInteressante = interessantsList.firstEntry().getValue();
		
		/***************************************************************************/
		
		/****************** ORIENTATION VERS PALET POTENTIEL ************************/
		
		// Degres parcourus entre deux mesures
		double degreParMesure = 360.0 / nbMesures;
		// Degre correspondant au palet
		double degrePalet;
		
		//Si on a d�tect� une distance int�ressante
		if (!interessantsList.isEmpty()) {
			System.out.println("Min interessant : " +minInteressant); //TEST
			degrePalet = degreParMesure * quantiemeMesureInteressante ; 
		}
		else {
			System.out.println("min normal"); //TEST
			degrePalet = degreParMesure * quantiemeMesure ; 
		}
		// On rajoute un buffer de quelques degres 
		//car le capteur le voit vant d'�tre centr� sur le palet
		degrePalet = degrePalet + 7;
		
		// Si aucune mesure min mesur�e (que des infinity ou < 31cm)
		if (quantiemeMesure == -1) {
			return 1;
		}
		// Sinon :
		/*plus on fait de mesures, plus il manque des degres j'ai l'impression. 
		On multiplie donc le nombre de degres par un buffer
		float buffDegrees = (float) 1; */
		
		// (degre entre deux mesures) * nbmesures => degre vers distance min
		if (degrePalet < 180)		
			robot.rotate(degrePalet); 											
		else
			robot.rotate(-360 + degrePalet);

		/*************************************************************************/
		
		/*********VERIFICATION ROBOT SE TROUVE DEVANT PALET*************/
		sampleProvider.fetchSample(sample, 0);
		if(sample[0] > min * 0.97 && sample[0] < min * 1.03) 
			return 0;
		
		//mesures en continu
		robot.setAngularSpeed(ANG_SPEED_CHECK);
		int balayage = 50;
		int increment = 5;
		int balayageCourant = 0;
		while(true) {
			if (balayageCourant == balayage) {
				if(balayageCourant < 0) {
					robot.setAngularSpeed(ANG_SPEED_REGULAR);
					return 1;
				}
				// si on a parcouru les 40 degres du balayage, on change de sens
				balayage = -balayage;
				balayageCourant = 0;
				increment = -increment;
			}
			//tourne un peu � droite puis un peu � gauche tant que distance mesur�e est diff�rente
			// de min
			robot.rotate(increment, true);
			balayageCourant += increment;
			while (robot.isMoving()) {			
				sampleProvider.fetchSample(sample, 0);
				
				// on a retrouve la distance minimum
				if(sample[0] > min * 0.9 && sample[0] < min * 1.1) {
					//on ajoute juste quelques degres, parce qu'en g�n�ral, le robot
					// vise le bord
					/*
					if(balayage>0)
						robot.rotate(5);
					else
						robot.rotate(-5);
					*/
					robot.setAngularSpeed(ANG_SPEED_REGULAR);
					return 0;
				}
			}
		}
	}
	
	public void test() {
		// Button.ENTER.waitForPressAndRelease();
		//robot.ouvrePinces();
		//robot.fermePinces();
		robot.rotate(360);

		//scanPalet();
		//dirigerVersPalet();
		//attrapePalet();
		//verifAttrapPalet();
		//Button.ENTER.waitForPressAndRelease();	
	}
	
	public void test2() {
		
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();
		int nbMesures = 0; 
		ArrayList<Float> distances = new ArrayList<Float>(); // on r�cup�re les mesures dans une arraylist
		float[] sample = new float[sampleProvider.sampleSize()];
		for (int i = 0 ; i < 10 ; i++) {
			sampleProvider.fetchSample(sample, 0); // on fait une mesure
			
			System.out.println(sample [0]);
			/*if (sample[0] > 2.5)
				distances.add((float) 2.5); // on l'ajoute � l'array list
			else*/
				distances.add(sample[0]);
			nbMesures++;
			Button.ENTER.waitForPressAndRelease();
		}
		
		System.out.println("Array size :" + distances.size());
		Button.ENTER.waitForPressAndRelease();
		
		
		for (int i = 1 ; i <= 10 ; i++) {
			System.out.println(distances.get(i-1));
			if (i >2 && i < 7)
				System.out.println(distances.get(i) - distances.get(i+1));
			Button.ENTER.waitForPressAndRelease();

		}
		
		
	}

	public void test3() {
		/*
		robot.ouvrePinces();
		robot.forward();
		while (!capteurs.touchSensor.isPressed()) {}
		FermePincesAsynch t = new FermePincesAsynch(robot);
		t.start();
		robot.arc(1500, 70);
		robot.arc(-1500, 70);
		robot.forward();
		*/
		//robot.ouvrePinces();
		robot.fermePinces();
		/*
		for (int i = 0 ; i < 10 ; i++) {
			System.out.println(capteurs.ligneBlancheDetectee());
			Button.ENTER.waitForPressAndRelease();
		}
		*/
		/*
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();
		float[] sample = new float[1];
		for (int i = 1 ; i <= 300 ; i ++) {
			sampleProvider.fetchSample(sample, 0); // on fait une mesure
			System.out.println(sample[0]);
			robot.rotate(1);
			if (i % 7 == 0)
				Button.ENTER.waitForPressAndRelease();
		}
		*/
	}
}
