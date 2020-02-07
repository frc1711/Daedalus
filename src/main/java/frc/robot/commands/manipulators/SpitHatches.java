/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manipulators;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.ManipulatorHatch;

public class SpitHatches extends Command {
  double state; 
  double positionCounter; 
  boolean pressed; 
  ManipulatorHatch manipulator; 
  Joystick stick; 

  public SpitHatches(ManipulatorHatch manipulator, Joystick stick) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(manipulator); 
    this.manipulator = manipulator; 
    this.stick = stick; 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    pressed = false; 
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (stick.getRawAxis(2) > .1 || stick.getRawAxis(3) > .1) {
      manipulator.setHatchRelay(Relay.Value.kForward);
    } else {
      manipulator.setHatchRelay(Relay.Value.kOff); 
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
