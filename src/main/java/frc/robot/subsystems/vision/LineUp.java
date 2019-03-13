/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap.RoboDir;

/**
 * Add your docs here.
 */
public class LineUp {
    
    public static void run() {
        
     double tapeAngle = SmartDashboard.getNumber("Tape Angle", 700); 
        //TODO: Fix these angles to be Correct
        //TODO: Adjust the angle of Pixycam based on button press
        //if (OI.lineUpEnable.get()) {
             Robot.pixyTilt.angleServo(180); 
            if(tapeAngle < -5 ) {
              SmartDashboard.putString("PIXY DIR", "LEFT");
              Robot.driveSystem.driveDirection(.7, RoboDir.RIGHT); 
            } else if (tapeAngle > 5) {
              SmartDashboard.putString("PIXY DIR", "RIGHT");
              
              Robot.driveSystem.driveDirection(.7, RoboDir.LEFT); 
            } else if (tapeAngle >= -5 && tapeAngle <= 5) {
              Robot.driveSystem.driveDirection(.5, RoboDir.STRAIGHT); 
              SmartDashboard.putString("PIXY DIR", "STRAIGHT");
              System.out.println(tapeAngle);
            } else {
              SmartDashboard.putString("PIXY DIR", "FAIL");
            }
            
         // }
    }
}
