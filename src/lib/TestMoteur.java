package lib;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class TestMoteur
{
    private EV3LargeRegulatedMotor mLeftMotor;
    private EV3LargeRegulatedMotor mRightMotor;

    private final static int SPEED = 20;


    public TestMoteur(Port left_port, Port right_port)
    {
        mLeftMotor = new EV3LargeRegulatedMotor(left_port);
        mRightMotor = new EV3LargeRegulatedMotor(right_port);
        
        
        //mLeftMotor.synchronizeWith(new RegulatedMotor[] {mRightMotor});
        mLeftMotor.setSpeed(SPEED);
        mRightMotor.setSpeed(SPEED);
    }


    public void forward()
    {
        mLeftMotor.forward();
        mRightMotor.forward();
    }


    public void stop()
    {
        mLeftMotor.stop();
        mRightMotor.stop();
    }


    public void rotateClockwise()
    {
        mLeftMotor.forward();
        mRightMotor.backward();
    }


    public void rotateCounterClockwise()
    {
        mLeftMotor.backward();
        mRightMotor.forward();
    }
    
    public static void main(String[] args) {
		TestMoteur roues = new TestMoteur(MotorPort.A,MotorPort.D);
		roues.forward();

		Delay.msDelay(1000);

		roues.stop(); 
	}
}
