/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class PneumaticOff extends Command {
  public PneumaticOff() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.manipulator); 
    requires(Robot.arm); 
    requires(Robot.lift); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (OI.pnuematicOff.get() && OI.pnuematicOffTwo.get()) {
      Robot.manipulator.hatchRelay.set(Relay.Value.kOff); 
      Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kOff); 
      Robot.lift.unlockBot.set(DoubleSolenoid.Value.kOff); 
      Robot.lift.auxWheelSolenoid.set(DoubleSolenoid.Value.kOff);
      Robot.lift.botLift.set(DoubleSolenoid.Value.kOff); 
      System.out.println("All pneumatics are off.");
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
