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
public class BallFollow {
    
    public static void run() {
        
    double ballAngle = SmartDashboard.getNumber("Ball Angle", 700); 

        if (OI.visionEnable.get()) {
            if(ballAngle < -5 ) {
              System.out.println("LEFT");
             // Robot.driveSystem.driveDirection(.3, RoboDir.RIGHT); 
            } else if (ballAngle > 5) {
              System.out.println("RIGHT");
             // Robot.driveSystem.driveDirection(.3, RoboDir.LEFT); 
            } else if (ballAngle >= -5 && ballAngle <= 5) {
             // Robot.driveSystem.driveDirection(.2, RoboDir.STRAIGHT); 
              System.out.println("STRAIGHT");
              System.out.println(ballAngle);
            } else {
              System.out.println("FAIL");
            }
            
          }
    }
}
