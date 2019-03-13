/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manipulators;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

/***
 *@author: Lou DeZeeuw
 */

public class CargoManipulator extends Command {
  public CargoManipulator() {

    requires(Robot.manipulator);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.manipulator.runManipulator(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 
    //Intake and outtake method
    System.out.println(Robot.manipulator.getManipulatorSwitch()); 
    if (OI.manipButtonZero.get()) {
      
      Robot.manipulator.runManipulator(.75);

    } else if (OI.manipButtonOne.get()){

      Robot.manipulator.runManipulator(-.75); 

    } else {

      Robot.manipulator.runManipulator(0);

    } 

    if (OI.manipButtonZero.get() && !Robot.manipulator.getManipulatorSwitch()) {
      OI.controllerZero.setRumble(RumbleType.kLeftRumble, 1); 
    } else {
      OI.controllerZero.setRumble(RumbleType.kLeftRumble, 0);
    }
/*
    if (OI.manipButtonZero.get()) {
      Robot.manipulator.runManipulator(.75); 
    } else if (OI.manipButtonOne.get()) {
      Robot.manipulator.runManipulator(-.75); 
    } else {
      Robot.manipulator.runManipulator(0); 
    }
  */  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.manipulator.runManipulator(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.manipulator.runManipulator(0);
  }
}
