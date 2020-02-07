/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AutoDrive extends Command {

  double encValue; 
  double desiredLocation; 
  double gyroAngle; 
  double startTime; 
  double speed; 
  double currentTime; 
  double timeout; 

  public AutoDrive(double distance, double speed, double timeout) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    desiredLocation = Math.round(distance * 11.5); // 1 in per 11.5 revolutions (based on a total count of 69 count per rot and 6 in wheels)
    // 6 in per 69 count per rot = 1 in per 11.5 count per rot 
    this.speed = speed; 
    
    this.timeout = timeout * 1000; //timeout in seconds, system in millis
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSystem.stopRobot();     
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
