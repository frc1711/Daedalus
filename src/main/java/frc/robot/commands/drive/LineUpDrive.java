/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap.RoboDir;

public class LineUpDrive extends Command {
  public LineUpDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double ballAngle = SmartDashboard.getNumber("Ball Angle", 700); 
        SmartDashboard.putBoolean("IS IT HOLDING", OI.ballVisionEnable.get()); 
        if (OI.ballVisionEnable.get()) {
          Robot.pixyTilt.angleServo(180);
         // System.gc(); 
            if(ballAngle < -5 ) {
              SmartDashboard.putString("DIRECTIONPIXY", "LEFT");
              Robot.driveSystem.driveDirection(.3, RoboDir.LEFT); 
            } else if (ballAngle > 5) {
              SmartDashboard.putString("DIRECTIONPIXY", "RIGHT");
              Robot.driveSystem.driveDirection(.3, RoboDir.RIGHT); 
            } else if (ballAngle >= -5 && ballAngle <= 5) {
              Robot.driveSystem.driveDirection(-.3, RoboDir.STRAIGHT); 
              SmartDashboard.putString("DIRECTIONPIXY", "STRAIGHT");
              System.out.println(ballAngle);
            } else {
              System.out.println("FAIL");
            }
            
          }
  }
  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
