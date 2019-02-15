/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manipulators;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class HatchManipulator extends Command {
  int state; 
  public HatchManipulator() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.manipulator); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.manipulator.setHatchRelay(Value.kOff); 
    state = 0; 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(OI.hatchButton.get() && state == 0) {
      Robot.manipulator.setHatchRelay(Value.kForward); 
      state = 1; 
    } else if (OI.hatchButton.get() && state == 1) {
      Robot.manipulator.setHatchRelay(Value.kOff);
      state = 0; 
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
