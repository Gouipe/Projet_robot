package projet;

public class FermePincesAsynch extends Thread{
	
	MonPilot monPilot;
	
	public FermePincesAsynch(MonPilot monPilot) {
		this.monPilot = monPilot;
	}
	
	public void run() {
		monPilot.fermePinces();;
	}

}
