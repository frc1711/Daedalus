/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.lift;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class AuxWheel extends Command {
  public int state = 0; 
  public AuxWheel() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.lift); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.lift.stopWheel(); 
    Robot.lift.setAuxWheel(Value.kReverse);
    state = 2; 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (OI.auxWheelButton.get() && state == 2 || state == 0) {
      Robot.lift.setAuxWheel(Value.kForward); 
      state = 1;
    } else if (OI.auxWheelButton.get() && state == 1) {
      Robot.lift.setAuxWheel(Value.kReverse); 
      state = 2; 
    } 
    
    if (Math.abs(OI.controllerOne.getRawAxis(0)) > .1) {
      Robot.lift.runAuxWheel(OI.controllerOne.getRawAxis(0));
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
