package com.robotic.linefollower;

import com.robotic.ColorSensor;
import com.robotic.Lcd;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.robotics.Color;
import lejos.utility.Delay;

public class TestLineFollow
{

	public static void main(String[] args)
	{
		EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.B);
	    ColorSensor LeftColorSensor = new ColorSensor(SensorPort.S2);
	    ColorSensor RightColorSensor = new ColorSensor(SensorPort.S1);
		
		
		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		
		Keys bottons = ev3brick.getKeys();
		
		System.out.println("Press the botton to start");
		bottons.waitForAnyPress();
		
		LeftColorSensor.setFloodLight(true);
		RightColorSensor.setFloodLight(true);
		LeftColorSensor.setColorIdMode();
		RightColorSensor.setColorIdMode();
		
		
		while(!Button.ESCAPE.isDown()) {		
            Lcd.clear(2);
            Lcd.print(2, "id=%s", ColorSensor.colorName(LeftColorSensor.getColorID()));
            Lcd.print(3, "id=%s", ColorSensor.colorName(RightColorSensor.getColorID()));
            
            Delay.msDelay(200);
            
    		LEFT_MOTOR.forward();
    		RIGHT_MOTOR.forward();
    		LEFT_MOTOR.setSpeed(100);
    		RIGHT_MOTOR.setSpeed(100);
    		
    		
    		
           while(LeftColorSensor.getColorID() == Color.BLACK) {
           	Delay.msDelay(200);
            	LEFT_MOTOR.stop();
           	RIGHT_MOTOR.setSpeed(100);
           }
           while(RightColorSensor.getColorID() == Color.BLACK) {
           	Delay.msDelay(200);
           	LEFT_MOTOR.setSpeed(100);
           	RIGHT_MOTOR.stop();
           	}
            	
            }
		}
	}
