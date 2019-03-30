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

public class PixyDrive extends Command {
  public PixyDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.i2CInterface); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (OI.lineUpEnable.get()) {
      if(Robot.i2CInterface.getPixyAngle() < -5 && Robot.i2CInterface.getPixyAngle() != 700) {
        SmartDashboard.putString("DIRECTIONPIXY", "LEFT"); 
        Robot.driveSystem.driveDirection(.5, RoboDir.LEFT); 
      } else if (Robot.i2CInterface.getPixyAngle() > 5 && Robot.i2CInterface.getPixyAngle() != 700) {
        SmartDashboard.putString("DIRECTIONPIXY", "RIGHT"); 
        Robot.driveSystem.driveDirection(.5, RoboDir.RIGHT); 
      } else if (Robot.i2CInterface.getPixyAngle() >= -5 && Robot.i2CInterface.getPixyAngle() <= 5 && Robot.i2CInterface.getPixyAngle() != 700) {
        SmartDashboard.putString("DIRECTIONPIXY", "STRAIGHT"); 
        Robot.driveSystem.driveDirection(0.7, RoboDir.STRAIGHT); 
      } else {
        SmartDashboard.putString("DIRECTIONPIXY", "FAIL"); 
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
//TODO: Add PixyDrive to command and wire arduino. Remove other methods. Add SmartDashboard logs to help with debugging.