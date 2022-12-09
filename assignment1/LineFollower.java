package com.robotic.linefollower;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.*;
import lejos.hardware.port.*;
import lejos.robotics.Color;

import com.robotic.ColorSensor;
import com.robotic.Lcd;

public class LineFollower 
{ 
    static UnregulatedMotor motorA = new UnregulatedMotor(MotorPort.A);
    static UnregulatedMotor motorB = new UnregulatedMotor(MotorPort.B);
    static ColorSensor        color = new ColorSensor(SensorPort.S3);
    
    public static void main(String[] args)
    {
        float    colorValue;
        
        System.out.println("Line Follower\n");
        
        color.setRedMode();
        color.setFloodLight(Color.RED);
        color.setFloodLight(true);

        Button.LEDPattern(4);    // flash green led and 
        Sound.beepSequenceUp();  // make sound when ready.

        System.out.println("Press any key to start");
        
        Button.waitForAnyPress();
        
        motorA.setPower(40);
        motorB.setPower(40);
       
        // drive waiting for touch sensor or escape key to stop driving.

        while (Button.ESCAPE.isUp()) 
        {
            colorValue = color.getRed();
            
            Lcd.clear(7);
            Lcd.print(7,  "value=%.3f", colorValue);

            if (colorValue > .100)
            {
                motorA.setPower(40);
                motorB.setPower(20);
            }
            else
            {
                motorA.setPower(20);    
                motorB.setPower(40);
            }
        }
       
        // stop motors with brakes on.
        motorA.stop();
        motorB.stop();

        // free up resources.
        motorA.close();
        motorB.close();
        color.close();
       
        Sound.beepSequence(); // we are done.
    }
}

