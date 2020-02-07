/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSystem;

public class RawArcadeDrive extends Command {
  private boolean speed; 
  DriveSystem driveSystem; 
  Joystick stick; 

  public RawArcadeDrive(Joystick stick) {
    requires(Robot.driveSystem); 
    this.driveSystem = Robot.driveSystem; 
    this.stick = stick; 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    speed = false; 
    driveSystem.stopRobot(); 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (stick.getRawButtonReleased(1)) {
      speed = !speed; 
      SmartDashboard.putBoolean("Speed On", speed); 
    }

    if (!speed) {
      driveSystem.arcadeDrive(0.8*stick.getRawAxis(1), 0.8* stick.getRawAxis(4)); 
     } else if (speed) {
      driveSystem.arcadeDrive(0.4*stick.getRawAxis(1), 0.5*stick.getRawAxis(4)); 
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
