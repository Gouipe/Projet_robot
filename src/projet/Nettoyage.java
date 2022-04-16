package projet;
//Button.ENTER.waitForPressAndRelease();

import java.util.ArrayList;
import java.util.HashMap;
import java.util.ListIterator;
import java.util.Map;
import java.util.TreeMap;

import lejos.hardware.Button;
import lejos.hardware.Sound;
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

	// Constantes
	final private double DISTANCE_MIN_PALET = 0.321; // distance pour laquelle le palet n'est plus détecté
	final private int SCAN_PALETS = 0;
	final private int DIRIGER_VERS_PALET = 1;
	final private int ATTRAPER_PALET = 2;
	final private int RANGER_PALET_ATTRAPE = 3;
	final private double ANG_SPEED_CHECK = 40.0;
	final private double ANG_SPEED_REGULAR = 60.0;
	
	//Attributs
	float minAttribut;
	int coneAngle;
	MonPilot robot;
	Capteurs capteurs;
	private int etat; // Etat courant de l'automate

	//Constructeur
	public Nettoyage(MonPilot robot, Capteurs capteurs) {
		this.robot = robot;
		this.capteurs = capteurs;
		etat = SCAN_PALETS; // au début, on va chercher la position des palets
		coneAngle = 120;
	}

	public void run() {	
		System.out.println("Appuyez pour commencer");
		Button.ENTER.waitForPressAndRelease();
		Delay.msDelay(75);
		
		//VERSION NORMALE
		attraperPremierPalet();
		attraperDeuxiemePalet();
		orienteTroisiemePalet();
		etat = DIRIGER_VERS_PALET;
		
		//VERSION TEMPS MORT
		//etat = SCAN_PALETS;
		
		
		// On réduit un peu la vitesse pour l'automate au sinon problème avec le touch sensor
		robot.setLinearSpeed(2200);
		while(true) {
			switch (etat) {
			case SCAN_PALETS:
				// on ferme les pinces (il faut vérifier si les pinces sont déjà fermées)
				// robot.fermePinces();

				// on cherches les palets potentiels
				if (scanPaletRecursif(coneAngle) == 0)
					etat = DIRIGER_VERS_PALET;
				// Sinon on continue à scanner pour le moment
				break;

			case DIRIGER_VERS_PALET:
				if (dirigerVersPalet()/* Bis(minAttribut) */ == 0)
					etat = ATTRAPER_PALET;
				else {
					coneAngle = seReplaceBis();
					etat = SCAN_PALETS;
				}
				break;

			case ATTRAPER_PALET:
				if (attrapePalet() == 0) // si on attrape bien un palet, on va le ranger
					etat = RANGER_PALET_ATTRAPE;
				else
					// sinon on verifie très rapidement si jamais un palet a été attrapé par les 
					// pinces mais n'a pas touché le balai du capteur de toucher
					etat = verifAttrapPalet();
				break;

			case RANGER_PALET_ATTRAPE:
				rangePalet();
				// une fois que le palet est posé, on cherche un nouveau palet
				etat = SCAN_PALETS;
				break;
			}
		}
	}

	public void attraperPremierPalet() {

		robot.ouvrePinces();
		robot.travel(4700);

		robot.stop();
		// ... ou synchrone.
		robot.fermePinces();

		robot.rotate(55);
		robot.travel(2500);
		// tourne vers la droite et avance un peu
		robot.rotate(-55.5);
		robot.forward();
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();
		float[] sample = new float[sampleProvider.sampleSize()];
		sampleProvider.fetchSample(sample, 0); // on fait une mesure

		while (!capteurs.ligneBlancheDetectee() /*|| sample[0] < 0.30 */) {
			sampleProvider.fetchSample(sample, 0); // on fait une mesure

		}

		robot.stop();
		robot.ouvrePinces();
		// recule de 15cm
		robot.travel(-800);
		// ferme pinces ?
		robot.fermePinces();
	}

	public void attraperDeuxiemePalet() {
		robot.rotate(214);
		robot.ouvrePinces();
		robot.travel(4000);
		robot.fermePinces();
		// attrapePalet();
		// rangePalet();
		// seReplace();

		robot.rotate(-217);
		robot.forward();
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();
		float[] sample = new float[sampleProvider.sampleSize()];
		sampleProvider.fetchSample(sample, 0); // on fait une mesure

		while (!capteurs.ligneBlancheDetectee() /*|| sample[0] < 0.30 */) {
			sampleProvider.fetchSample(sample, 0); // on fait une mesure

		}
		robot.stop();
		robot.ouvrePinces();
		// recule de 10cm
		robot.travel(-800);
		// ferme pinces ?
		robot.fermePinces();
	}

	/**
	 * @return 0 si le deuxieme palet est attrapé, 1 sinon
	 */
	public int attraperDeuxiemePaletSouple() {
		robot.rotate(214);
		if (attrapePalet() == 0) {
			rangePalet();
			return 0;
		} else {
			coneAngle = seReplaceBis();
			etat = SCAN_PALETS;
			return 1;
		}

	}

	public void attraperTroisiemePalet() {
		// s'oriente vers le troisieme
		robot.rotate(180);
		// se placer en face du 3eme
		robot.rotate(45);
		robot.ouvrePinces();
		robot.travel(2000);
		robot.rotate(-40);// pour compenser le desequilibre au moment du travel
		// avancer j'usqu'au palet

		robot.travel(12000);
		System.out.println("je suis au 3eme palet ?");
		// recuperer le palet
		robot.fermePinces();
		robot.rotate(180);
		robot.forward();
		while (!capteurs.ligneBlancheDetectee() || !versLeMur()) {
		}
		robot.stop();
		robot.ouvrePinces();
		// recule de 10cm
		robot.travel(-1000);
		// ferme pinces ?
		robot.fermePinces();
	}

	public void orienteTroisiemePalet() {
		// s'oriente vers le troisieme
		robot.rotate(180);
		// se placer en face du 3eme
		robot.rotate(50);
		robot.travel(2000);
		robot.rotate(-60);// pour compenser le desequilibre au moment du travel
		// avancer j'usqu'au palet
	}

	/**
	 * Replace le robot après qu'il ait rencontré un mur en essayant d'attraper un palet
	 */
	public void seReplace() {
		robot.travel(-1500);
		robot.rotate(180, false);
	}

	/**
	 * Essaie de replacer le robot après qu'il ait rencontré un mur en fonction du mur rencontré
	 * 
	 * @return l'angle du prochain scan/recherche de palet
	 */
	public int seReplaceBis() {
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();
		float[] sample = new float[sampleProvider.sampleSize()];
		//on recule après avoir rencontré un mur
		robot.travel(-1500);
		// cas mur gauche
		if (robot.angleAbsoluCourant > 45 && robot.angleAbsoluCourant < 135) {
			System.out.println("mur gauche");
			robot.rotate(195 - robot.angleAbsoluCourant);
			return 155;
		}
		robot.rotateToGoal();
		sampleProvider.fetchSample(sample, 0);
		// cas mur but
		if (sample[0] < 0.45) {
			robot.rotate(105);
			return 120;
		}
		robot.rotate(90);
		sampleProvider.fetchSample(sample, 0);
		// cas mur gauche
		if (sample[0] < 0.45) {
			robot.rotate(105);
			return 155;
		}
		// cas mur droit
		else {
			robot.rotate(90);
			sampleProvider.fetchSample(sample, 0);
			// cas mur bas
			if (sample[0] < 0.45) {
				robot.rotate(105);
				return 140;
			} // else
			robot.rotate(-170);
			return 140;
		}

	}

	/**
	 * Range le palet une fois q'il est attrapé
	 */
	public void rangePalet() {
		robot.forward();
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();
		float[] sample = new float[sampleProvider.sampleSize()];
		sampleProvider.fetchSample(sample, 0); // on fait une mesure
		while (!capteurs.ligneBlancheDetectee()) {
			sampleProvider.fetchSample(sample, 0); // on fait une mesure
		}
		robot.stop();
		robot.ouvrePinces();
		robot.travel(-1000);
		robot.fermePinces();
		robot.rotate(120, false);
	}

	/**
	 * Fait bouger le robot en direction du palet potentiel détecté auparavant
	 * 
	 * @return 0 si le robot vient de se positionner à 30 cm d'un palet (a priori),
	 *         1 si on était sur le point de toucher un mur(on a pas trouvé de
	 *         palet)
	 * 
	 */
	public int dirigerVersPalet() {
		robot.forward();
		// mesure de manière asynchrone la distance courante
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();

		// variables pour sauvegarder les mesures
		float[] sample = new float[sampleProvider.sampleSize()];
		float previous = 10; // valeur absurde 10m
		float courant;
		// tant que la distance mesurée ne réaugmente pas (ce serait le cas où on a
		// passé le palet et le mur derrière est alors mesuré), le robot avance
		do {
			// on mesure la distance
			sampleProvider.fetchSample(sample, 0);
			courant = sample[0];
			// on était tres proche de quelquechose (un palet peut être) et cette chose
			// n'est plus visible 
			if (previous < 0.37 && courant / previous > 1.6 ) {
				robot.stop();
				// A priori, on est juste devant un palet
				return 0;
			}
			previous = courant;
			// on arrête la boucle si on va toucher un mur
		} while (courant > 0.305);
		
		robot.stop();
		return 1;
	}

	/**
	 * Attrape un palet. A utiliser quand le palet n'est plus détecté alors qu'on
	 * avance vers lui, le robot est alors à moins de 31 cm. Renvoie 0 si un palet
	 * est attrapé, renvoie 1 sinon
	 */
	public int attrapePalet() {
		// on ouvre les pinces pour attraper le palet
		robot.ouvrePinces();

		// on avance vers le palet
		robot.forward();

		// on mesure de manière asynchrone la distance de l'objet devant le robot
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();
		float[] sample = new float[sampleProvider.sampleSize()];
		float distanceMesuree = 99; // valeur absurde
		// on arrête d'essayer d'attraper au bout de 8 secondes ou si on s'apprete à
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
	 * Verifie très rapidement si jamais un palet a été attrapé par les pinces mais
	 * n'a pas touché le balai du capteur de toucher. Réouvre les pinces et avance
	 * d'un chouia. Si un palet était attrapé, le détecteur de toucher le détecte
	 * maintenant.
	 * 
	 * @return RANGER_PALET_ATTRAPE si il y avait un palet, SCAN_PALETS sinon
	 */
	public int verifAttrapPalet() {
		robot.ouvrePinces();
		robot.travel(200, true); // on bouge de 2cm
		
		long beginning = System.currentTimeMillis();
		while (System.currentTimeMillis() - beginning < 500) {
			// il y avait bien un palet attrapé
			if (capteurs.touchSensor.isPressed()) {
				robot.stop();
				robot.fermePinces();
				robot.rotateToGoal();
				// on renvoie l'etat suivant
				return RANGER_PALET_ATTRAPE;
			}
		}
		// Pas de palet détecté :
		robot.stop();
		robot.fermePinces();
		// Il faut chercher un palet
		return SCAN_PALETS;
	}

	/**
	 * Scanne alentour pour chercher des palets. S'oriente alors vers le palet
	 * potentiel le plus proche. Renvoie 0 si un palet a été trouvé, renvoie 1 sinon
	 */
	public int scanPalet() {
		/********************** PRISE MESURES **************************/

		// On prend une mesure par degre en deleyant chaque mesure
		// en fonction de la vitesse de rotation
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();
		// le nombre de mesures ne vaudra pas exactement celui voulu, donc on les compte
		int nbMesures = 0;
		// on récupère les mesures dans une arraylist
		ArrayList<Float> distances = new ArrayList<Float>(); 
		float[] sample = new float[sampleProvider.sampleSize()];
		int angleAnalyse = 360;
		robot.rotate(angleAnalyse, true);
		while (robot.isMoving()) {
			// on fait une mesure
			sampleProvider.fetchSample(sample, 0); 
			// Si mesure vaut infinity, on la transforme en 2.5
			if (sample[0] > 2.5)
				distances.add((float) 2.5); // on l'ajoute à l'array list
			else
				distances.add(sample[0]);
			nbMesures++;
			// on délaie la prochaine mesure de : 
			//temps total de rotation / nombre de mesures voules
			// <=> (nombre de degres parcourus / vitesse de rotation) en ms / nombre de
			// mesures voules
			Delay.msDelay(1000 * angleAnalyse / ((int) robot.getAngularSpeed()) / angleAnalyse);
		}
		/****************************************************************/

		/*********** RECUPERATION MINIMUM/VALEUR INTERESSANTE **********************/

		// On récupère dans la variable min la plus petite distance qui peut
		// correspondre à un palet

		float min = (float) 999.0; // Initialisation du min avec valeur absurde.
		int quantiemeMesure = -1; // Ordre de la mesure, valeur absurde au depart

		// Map des min intéressants pour ne garder que le plus petit par la suite
		TreeMap<Float, Integer> interessantsList = new TreeMap<Float, Integer>();
		float minInteressant = (float) 999.0; 
		int quantiemeMesureInteressante = -1;

		float courant; // Mesure courante dans le parcours de l'array list
		float previous = (float) 0.0; // Mesure précedente
		for (int i = 0; i < distances.size(); i++) {
			courant = distances.get(i);
			// Si courant est une distance qui peut correspondre à un palet
			if (courant < min && courant > DISTANCE_MIN_PALET) {
				min = courant;
				quantiemeMesure = i; // l'index corresond au nombre de mesures / à la combientieme mesure
			}
			// Si la mesure a diminué de 20cm ou plus
			if (i > 0 && i < distances.size() && previous - courant > 0.20 && courant != 2.5 && previous != 2.5) {
				// on "initialise" minInteressant et son index/degre
				// Si une des valeurs entre les 15 dernieres et les 15 suivantes est inférieure
				// à 30 cm (c'est un mur)
				
				boolean mur = false; 
				for (int j = -15 ; j < 15 && !mur; j++) { 
					if (i + j >=0 && i + j < distances.size()) { 
						if (distances.get(i + j) < 0.30) { 
							mur = true;
						} 
					} 
				}
				
				interessantsList.put(courant, i);
			}
			previous = courant;
		}
		if (!interessantsList.isEmpty()) {
			// RECUPERATION DU MIN INTERESSANT
			minInteressant = interessantsList.firstKey();
			quantiemeMesureInteressante = interessantsList.firstEntry().getValue();
		}

		/***************************************************************************/

		/****************** ORIENTATION VERS PALET POTENTIEL ************************/

		// Degres parcourus entre deux mesures
		double degreParMesure = angleAnalyse / nbMesures;
		// Degre correspondant au palet
		double degrePalet;

		// Si on a détecté une distance intéressante
		if (!interessantsList.isEmpty()) {
			// (degre entre deux mesures) * nbmesures = degre vers distance min
			degrePalet = degreParMesure * quantiemeMesureInteressante;
		} else {
			degrePalet = degreParMesure * quantiemeMesure;
		}
		// On rajoute un buffer de quelques degres car ça a l'air de marcher
		// de manière empirique
		degrePalet = degrePalet + 10;

		// Si aucune mesure min mesurée (que des infinity ou < 31cm)
		if (quantiemeMesure == -1) {
			return 1;
		}
		// Sinon :
		/*
		 * plus on fait de mesures, plus il manque des degres j'ai l'impression. On
		 * multiplie donc le nombre de degres par un buffer float buffDegrees = (float)
		 * 1;
		 */

		
		if (degrePalet < 180)
			robot.rotate(degrePalet);
		else
			robot.rotate(-360 + degrePalet);

		/*************************************************************************/

		/********* VERIFICATION ROBOT SE TROUVE DEVANT PALET *************/

		sampleProvider.fetchSample(sample, 0);
		if (sample[0] > min * 0.97 && sample[0] < min * 1.03)
			return 0;

		// On balaie un aller et un retour sur 40 degres pour retrouver la valeur min
		robot.setAngularSpeed(ANG_SPEED_CHECK);
		int balayage = 40;
		int increment = 5;
		int balayageCourant = 0;
		while (true) {
			if (balayageCourant == balayage) {
				//on a fait un aller retour, on abandonne
				if (balayageCourant < 0) {
					robot.setAngularSpeed(ANG_SPEED_REGULAR);
					return 1;
				}
				// si on a parcouru les 40 degres du balayage, on change de sens
				balayage = -balayage;
				balayageCourant = 0;
				increment = -increment;
			}
			// tourne un peu à droite puis un peu à gauche tant que distance mesurée est
			// différente de min
			robot.rotate(increment, true);
			balayageCourant += increment;
			while (robot.isMoving()) {
				sampleProvider.fetchSample(sample, 0);
				if (!interessantsList.isEmpty()) {
					// on a retrouve la distance minimum
					if (sample[0] > minInteressant * 0.80 && sample[0] < minInteressant * 1.2) {
						robot.setAngularSpeed(ANG_SPEED_REGULAR);
						return 0;
					}
				} else {
					if (sample[0] > min * 0.8 && sample[0] < min * 1.2) {
						// on ajoute juste quelques degres
						robot.setAngularSpeed(ANG_SPEED_REGULAR);
						return 0;
					}
				}

			}
		}

	}

	/**
	 * Recherche un palet de manière répétée avec des angles de plus en plus petit
	 * pour avoir une meilleure précision. 
	 * @param angleAnalyse :l'angle de recherche
	 * @return 0 si palet trouvé, 1 sinon
	 */
	public int scanPaletRecursif(int angleAnalyse) {
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();

		/********************** PRISE MESURES **************************/
		int nbMesures = 0; // le nombre de mesures ne vaudra pas exactement 360
		ArrayList<Float> distances = new ArrayList<Float>(); // on récupère les mesures dans une arraylist
		float[] sample = new float[sampleProvider.sampleSize()];

		robot.rotate(angleAnalyse, true);
		while (robot.isMoving()) {
			sampleProvider.fetchSample(sample, 0); // on fait une mesure
			if (sample[0] > 2.5)
				distances.add((float) 2.5); // on l'ajoute à l'array list
			else
				distances.add(sample[0]);
			nbMesures++;

			Delay.msDelay(1000 * (angleAnalyse / ((int) robot.getAngularSpeed())) / angleAnalyse);
		}

		/*********** RECUPERATION MINIMUM **********************/
		float min = (float) 999.0; // Initialisation du min avec valeur absurde.
		int quantiemeMesure = -1; // Ordre de la mesure, valeur absurde au depart
		float courant;
		for (int i = 0; i < distances.size(); i++) {
			courant = distances.get(i);
			// Si courant est une distance qui peut correspondre à un palet
			if (courant < min && courant > DISTANCE_MIN_PALET) {
				min = courant;
				quantiemeMesure = i; // l'index corresond au nombre de mesures / à la combientieme mesure
			}
		}
		minAttribut = min;
		// Degres parcourus entre deux mesures
		double degreParMesure = (double) angleAnalyse / nbMesures;
		// Degre correspondant au palet
		double degrePalet;
		degrePalet = degreParMesure * quantiemeMesure;
		double degreFinal = degrePalet - angleAnalyse;
		if (angleAnalyse <= 40) {
			double buff = 0;
			degreFinal += buff;
		}

		// Si aucune mesure min mesurée (que des infinity ou < 31cm)
		if (quantiemeMesure == -1) {
			return 1;
		}
		robot.rotate(degreFinal, false);

		// Cas de base
		if (angleAnalyse <= 40)
			return 0;
		// Cas récursif
		//Si un palet est trouvé, on refait
		// une recherche centrée sur le min trouvé avec un angle deux fois plus petit
		robot.rotate(-angleAnalyse / 4 - 10, false);
		scanPaletRecursif(angleAnalyse / 2);
		return 0;
	}

	public boolean versLeMur() {
		// capteurs.soundSensor.fetchSample(null, 0);

		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();
		float[] sample = new float[sampleProvider.sampleSize()];
		// float [] sample = new float[sampleProvider.sampleSize()];
		// float [] sample = new float[sampleProvider.sampleSize()];

		// UltraSonic ultra = new UltraSonic(sampleProvider);
		sampleProvider.fetchSample(sample, 0);
		float distanceCourante1 = sample[0];
		Delay.msDelay(20);
		sampleProvider.fetchSample(sample, 0);
		float distanceCourante2 = sample[0];
		Delay.msDelay(20);
		sampleProvider.fetchSample(sample, 0);
		float distanceCourante3 = sample[0];
		if (distanceCourante1 < 0.25f && distanceCourante2 < 0.25f && distanceCourante3 < 0.25f) {
			System.out.println(
					"direction vers un mur : " + distanceCourante1 + " " + distanceCourante2 + " " + distanceCourante3);
			return true;
		} else {
			System.out.println(
					"bonne direction : " + distanceCourante1 + " " + distanceCourante2 + " " + distanceCourante3);
			return false;
		}

	}
}
