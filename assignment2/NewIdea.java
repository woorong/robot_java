package com.robotic.assignment2.main;

import java.util.Stack;

import com.robotic.assignment2.basic.*;

import lejos.ev3.tools.LCDDisplay;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Keys;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

/**
 * The main function of the robot
 *
 * @author suzhen
 *
 */
public class NewIdea {

	/**
	 * create robotic main equipment
	 */

	//LeftMotor
	static EV3LargeRegulatedMotor LeftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
	//RightMotor
	static EV3LargeRegulatedMotor RightMotor = new EV3LargeRegulatedMotor(MotorPort.D);

	//irSenosr
	static EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S2);
	//ColorSensor
	static ColorSensor ColorSensor = new ColorSensor(SensorPort.S1);
	//MediumRegulateMotor
	static EV3MediumRegulatedMotor MediumMotor = new EV3MediumRegulatedMotor(MotorPort.B);

	//pilot
	static Wheel wheel1 = WheeledChassis.modelWheel(LeftMotor, 56).offset(65);
	static Wheel wheel2 = WheeledChassis.modelWheel(RightMotor, 56).offset(-65);

	static Chassis chassis = new WheeledChassis(new Wheel [] {wheel1,  wheel2},
			WheeledChassis.TYPE_DIFFERENTIAL);
	static MovePilot pilot = new MovePilot(chassis);


	public static void main(String[] args) {
		EV3 ev3Brick = (EV3) BrickFinder.getLocal();
		Keys buttons = ev3Brick.getKeys();

		TextLCD lcddisplay = ev3Brick.getTextLCD(Font.getFont(0, 0,Font.SIZE_SMALL));
		//Press Botton To Start
		Lcd.print(1, "System Ready! PRESS!");
        Button.LEDPattern(4);    // flash green led and
        Sound.beepSequenceUp();    // make sound when ready
        Button.waitForAnyPress();
        Button.LEDPattern(0);

        ColorSensor.setFloodLight(true);
		ColorSensor.setColorIdMode();

		//Maze
		int[][] Maze =
			{
				{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
				{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
				{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
				{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
				{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
				{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
				{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
				{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
				{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
				{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
				{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
				{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
				{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
			};

		//temp Point Value
		OccupyMaze Point = new OccupyMaze(1,11,1);
		Stack<OccupyMaze> s = new Stack<OccupyMaze>();

        while(Button.ESCAPE.isUp()) {
        	int stop = 0;
        	int front = 0;
        	int right = 0;
        	int left = 0;
//            lcddisplay.clear();
//            for(int i = 0; i < 13; i++) {
//            	for(int j = 0; j < 19; j++) {
//            		lcddisplay.drawInt(Maze[i][j], j, i);
//            	}
//            }
//

        	Lcd.clear();
        	Lcd.print(1,"foward");

//        	Lcd.print(1, "X = %d", Point.getX());
//        	Lcd.print(2, "Y = %d", Point.getY());
//        	Lcd.print(3, "D = %d", Point.getHeading());
//
        	Delay.msDelay(200);
        	// wall situation: int left/right



//        	Lcd.clear();
//        	Lcd.print(1, "forward");
//        	Lcd.print(2, "Heading = %d", Point.getHeading());
        	pilot.setLinearSpeed(80);
        	pilot.setAngularSpeed(40);
        	pilot.travel(400);

        	pilot.stop();
        	stop = 1;
        	//when ColorSensor find green
        	if(ColorSensor.getColorID() == Color.GREEN) {
//            	Lcd.clear();
//        		Lcd.print(1, "Green");
            	Maze[Point.getX()][Point.getY()] = 4;
        		pilot.rotate(180);

        		//ultrasonic find left and right whether is wall
        	}

        	//when ColorSensor find red
        	if(ColorSensor.getColorID() == Color.RED) {
//            	Lcd.clear();
//            	Lcd.print(1, "Red");
            	pilot.stop();
            	Maze[Point.getX()][Point.getY()] = 5;
        		Delay.msDelay(300);
        	}

        	if(stop == 1 && ColorSensor.getColorID() != Color.RED && ColorSensor.getColorID() != Color.GREEN) {
            	//caculate x,y
            	if(Point.getHeading() == 1) {
            		Point.setY(Point.getY() + 1);
            	}else
            	if(Point.getHeading() == 2) {
            		Point.setX(Point.getX() + 1);
            	}else
            	if(Point.getHeading() == 3) {
            		Point.setY(Point.getY() - 1);
            	}else
            	if(Point.getHeading() == 4) {
            		Point.setX(Point.getX() - 1);
            	}



            	Maze[Point.getX()][Point.getY()] = 3;



           		// heading whether is truely value
        		if(Point.getHeading() <= 0) {
        			Point.setHeading(4);
//                	Lcd.clear();
//                	Lcd.print(2, "Heading = %d", Point.getHeading());
        		}
        		if(Point.getHeading() > 4) {
//                	Lcd.clear();
        			Point.setHeading(1);
//                	Lcd.print(2, "Heading = %d", Point.getHeading());
        		}


        		if(isWall() == true) {
        			front = 1;
        		}
        		MediumMotor.rotate(-90);
        		if(isWall() == true) {
        			right = 1;
        		}
        		MediumMotor.rotateTo(90);
        		if(isWall() == true) {
        			left = 1;
        		}
        		//input value into array: wall
            	if(left == 1) {
            		Maze[Point.getX()-1][Point.getY()] = 1;
            		Lcd.print(2, "left");
            	}else
            	if(right == 1) {
            		Maze[Point.getX()+1][Point.getY()] = 1;
            		Lcd.print(2, "right");

            	}else
            	if(front == 1) {
            		Maze[Point.getX()][Point.getY()+1] = 1;
            		Lcd.print(2, "front");
            	}


        		if(right == 1 && left == 0) {
//                	Lcd.clear();
//                	Lcd.print(1, "Turn right");
                	Point.setHeading(Point.getHeading() + 1);
//                	Lcd.print(2, "Heading = %d", Point.getHeading());
                	pilot.rotate(-90);
        		}
        		if(left == 1 && right ==  0) {
//                	Lcd.clear();
        			Lcd.print(1, "Turn left");
        			Point.setHeading(Point.getHeading() - 1);
//                	Lcd.print(2, "Heading = %d", Point.getHeading());
        			pilot.rotate(90);
        		}
        		if(left == 1 && right == 1 && front == 1) {
//                	Lcd.clear();
//                	Lcd.print(1, "Turn around");
                	if(Point.getHeading() == 1) {
                		Point.setHeading(3);
//               			Lcd.clear(2);
//                    	Lcd.print(2, "Heading = %d", Point.getHeading());
                	}else
                	if(Point.getHeading() == 2) {
                		Point.setHeading(4);
//                    	Lcd.clear();
//                    	Lcd.print(2, "Heading = %d", Point.getHeading());
                	}else
                	if(Point.getHeading() == 3) {
                		Point.setHeading(1);
//                    	Lcd.clear();
//                    	Lcd.print(2, "Heading = %d", Point.getHeading());
                	}else
                	if(Point.getHeading() == 4) {
                		Point.setHeading(2);
//                    	Lcd.clear();
//                    	Lcd.print(2, "Heading = %d", Point.getHeading());
                	}
        			pilot.rotate(180);

        			// When dead ends are reached, s pop this Point
        			if(!s.isEmpty()) {
        			Point = (OccupyMaze) s.pop();
        			}

        		}

        		if(front ==1) {
        		if(right == 0 && left == 0) {
//                	Lcd.clear();
//                	Lcd.print(1, "Turn right");
                	Point.setHeading(Point.getHeading() + 1);
//                	Lcd.print(2, "Heading = %d", Point.getHeading());
                	pilot.rotate(-90);

        			}
        		}
        	}
        		MediumMotor.rotateTo(0);


            	//Each Point's wall situation
            	if(Point.getHeading() == 1 && right == 1) {
            		Point.setEastWall(true);
            	}else
            		if(Point.getHeading() == 1 && left == 1) {
            			Point.setWestWall(true);
            	}else
            		if(Point.getHeading() == 1 && front == 1) {
            			Point.setNorthWall(true);
            	}else
            		if(Point.getHeading() == 2 && right == 1) {
            			Point.setNorthWall(true);
            	}else
            		if(Point.getHeading() == 2 && left == 1) {
            			Point.setNorthWall(true);
            	}else
            		if(Point.getHeading() == 2 && front == 1) {
            			Point.setEastWall(true);
            	}else
            		if(Point.getHeading() == 3 && right == 1) {
            			Point.setWestWall(true);
            	}else
            		if(Point.getHeading() == 3 && left == 1) {
            			Point.setEastWall(true);
            	}else
            		if(Point.getHeading() == 3 && front == 1) {
            			Point.setNorthWall(true);
            	}else
            		if(Point.getHeading() == 4 && right == 1) {
            			Point.setNorthWall(true);
            	}else
            		if(Point.getHeading() == 4 && left == 1) {
            			Point.setSouthWall(true);
            	}else
            		if(Point.getHeading() == 4 && front == 1) {
            			Point.setWestWall(true);
            	}

            	//Push Point into Stack
            	s.push(Point);
        	}
        }

	/**
	 * find whether is wall. return boolean(true/fasle)
	 *
	 * @return false or true(wall)
	 */
	public static boolean isWall()
	{
		SampleProvider Sonic = irSensor.getDistanceMode();
		SampleProvider average = new MeanFilter(Sonic, 5);
		float[] distance = new float[average.sampleSize()];
		Sonic.fetchSample(distance, 0);
		float x = distance[0];
		boolean iswall = false;
		//maximum wall distance
		float MaxDistance = 24;
		if(x  <= MaxDistance) {
			iswall = true;
		}else
			{
			iswall = false;
			}
		return iswall;
	}

}
