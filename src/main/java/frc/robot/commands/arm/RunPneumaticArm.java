/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class RunPneumaticArm extends Command {
  public int state; 
  public RunPneumaticArm() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.arm); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.arm.setArmSolenoid(Value.kReverse); 
    state = 2; 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //this has to work so that it can turn the solenoid on and off

    if (OI.armButton.get() && state == 1 || state == 0) {
      Robot.arm.setArmSolenoid(Value.kReverse); 
      state = 2; 
    } else if (OI.armButton.get() && state == 2) {
      Robot.arm.setArmSolenoid(Value.kForward); 
      state = 1; 
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
