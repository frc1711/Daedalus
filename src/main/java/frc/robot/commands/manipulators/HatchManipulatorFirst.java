/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manipulators;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class HatchManipulatorFirst extends Command {

  public HatchManipulatorFirst() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.manipulator); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
     //Robot.manipulator.hatchRelay.set(Relay.Value.kOff); 
  }
  //OI.controllerZero.getRawAxis(2) > .1 || OI.controllerZero.getRawAxis(3) > .1
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(OI.controllerZero.getRawAxis(2) > .1 || OI.controllerZero.getRawAxis(3) > .1) {
     Robot.manipulator.hatchRelay.set(Relay.Value.kForward); 
     System.out.println("yes");
     
    } else if (OI.controllerZero.getRawAxis(2) < .1 && OI.controllerZero.getRawAxis(3) == 0) {
      Robot.manipulator.hatchRelay.set(Relay.Value.kOff); 
      System.out.println("Also yes"); 
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
