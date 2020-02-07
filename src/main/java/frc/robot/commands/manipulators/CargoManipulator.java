/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manipulators;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Manipulator;

/***
 *@author: Lou DeZeeuw
 */

public class CargoManipulator extends Command {
  Manipulator manipulator; 
  Joystick stick; 
  public CargoManipulator(Joystick stick) {
    requires(Robot.manipulator);
    this.manipulator = Robot.manipulator; 
    this.stick = stick; 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    manipulator.runManipulator(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 
    //Intake and outtake method
    if (stick.getRawButton(5)) {
      manipulator.runManipulator(-.75);
    } else if (stick.getRawButton(6)){
      manipulator.runManipulator(.75); 
    } else {
      manipulator.runManipulator(0);
    } 

    if (stick.getRawButton(5) && !manipulator.getManipulatorSwitch()) {
      stick.setRumble(RumbleType.kLeftRumble, 1); 
    } else {
      stick.setRumble(RumbleType.kLeftRumble, 0);
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
    manipulator.runManipulator(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    manipulator.runManipulator(0);
  }
}
