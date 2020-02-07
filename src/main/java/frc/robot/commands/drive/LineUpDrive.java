/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.RoboDir;
import frc.robot.subsystems.DriveSystem;

public class LineUpDrive extends Command {
  private DriveSystem driveSystem; 
  public LineUpDrive(DriveSystem driveSystem) {
    requires(driveSystem); 
    this.driveSystem = driveSystem; 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double ballAngle = SmartDashboard.getNumber("Ball Angle", 700); 
    if(ballAngle < -5 ) {
      SmartDashboard.putString("DIRECTIONPIXY", "LEFT");
      driveSystem.driveDirection(.3, RoboDir.LEFT); 
    } else if (ballAngle > 5) {
      SmartDashboard.putString("DIRECTIONPIXY", "RIGHT");
      driveSystem.driveDirection(.3, RoboDir.RIGHT); 
    } else if (ballAngle >= -5 && ballAngle <= 5) {
      driveSystem.driveDirection(-.3, RoboDir.STRAIGHT); 
      SmartDashboard.putString("DIRECTIONPIXY", "STRAIGHT");
      System.out.println(ballAngle);
    } else {
      System.out.println("FAIL");
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
