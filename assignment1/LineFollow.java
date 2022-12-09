package EV3.test;

import java.awt.Button;

import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;

public class LineFollow
{

	public static void main(String[] args)
	{
		// TODO Auto-generated method stub
		EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.B);
	    ColorSensor LeftColorSensor = new ColorSensor(SensorPort.S1);
	    ColorSensor RightColorSensor = new ColorSensor(SensorPort.S2);
		
		
		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		
		Keys bottons = ev3brick.getKeys();
		
		bottons.waitForAnyPress();
		LEFT_MOTOR.setSpeed(150);
		RIGHT_MOTOR.setSpeed(150);
		while (true) {
		while(LeftColorSensor.getColorID() != 7 && RightColorSensor.getColorID()!= 7) {
		LEFT_MOTOR.forward();
		RIGHT_MOTOR.forward();
		LEFT_MOTOR.setSpeed(150);
		RIGHT_MOTOR.setSpeed(150);
		}
		
		
		while(LeftColorSensor.getColorID()==7) {
			Delay.msDelay(10);
			LEFT_MOTOR.stop();
			RIGHT_MOTOR.forward();
		}
		
		while(RightColorSensor.getColorID()==7) {
			Delay.msDelay(10);
			RIGHT_MOTOR.stop();
			LEFT_MOTOR.forward();
		}
		
		bottons.waitForAnyPress();
		LEFT_MOTOR.stop();
		RIGHT_MOTOR.stop();
		
	
		}
		
		
	}
	
}




	


