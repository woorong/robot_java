package com.robotic;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

/**
 * Class that will move a robot in a straight line and when it bumps into something it will rotate and go the other way
 * @author Administrator
 *
 */

public class BumperCar {
	



	public static void main(String[] args) {
		
		
		EV3 ev3brick = (EV3) BrickFinder.getLocal();

		Keys buttons = ev3brick.getKeys();
	
    	ColorSensor SensorLeft =  new ColorSensor(SensorPort.S1);
    	ColorSensor SensorRight = new ColorSensor(SensorPort.S2);
    	EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(
    			MotorPort.A);
    	EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(
    			MotorPort.B);

        System.out.println("Press any key to start");
		buttons.waitForAnyPress();

		Wheel wheel1 = WheeledChassis.modelWheel(LEFT_MOTOR, 56).offset(65);
		Wheel wheel2 = WheeledChassis.modelWheel(RIGHT_MOTOR, 56).offset(-65);
		
		Chassis chassis = new WheeledChassis(new Wheel [] {wheel1,  wheel2},
				WheeledChassis.TYPE_DIFFERENTIAL);

		MovePilot pilot = new MovePilot(chassis);
		
		while(true) {
			Delay.msDelay(2);
			if(SensorLeft.getColorID() == 7 || SensorRight.getColorID() == 7) {
			pilot.stop();
			pilot.travel(10);
			pilot.rotate(10);
			}
		
		}
		
		
			
	}
}
