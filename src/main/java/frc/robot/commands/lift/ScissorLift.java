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

public class ScissorLift extends Command {
  int state = 0; 
  int unlockState = 0; 
  public ScissorLift() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.lift.setBotLift(Value.kReverse);
    state = 2;  
    Robot.lift.unlockBotLift(Value.kForward); 
    unlockState = 1; 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //TODO: put in two other safety protocols 
    if (Math.abs(OI.controllerZero.getRawAxis(3)) > .1 && Math.abs(OI.controllerZero.getRawAxis(2)) > .1 && state == 2) {
      Robot.lift.setBotLift(Value.kForward); 
      state = 1; 
    } else if (Math.abs(OI.controllerZero.getRawAxis(3)) > .1 && Math.abs(OI.controllerZero.getRawAxis(2)) > .1 && state == 1) {
      Robot.lift.setBotLift(Value.kReverse); 
      state = 2; 
    }
    //TODO: replace this with the actual buton later
    if (OI.controllerOne.getPOV() == 180 && unlockState == 1 || unlockState == 0) {
      Robot.lift.unlockBotLift(Value.kReverse); 
      unlockState = 2; 
    } else if (OI.controllerOne.getPOV() == 180 && Math.abs(OI.controllerZero.getRawAxis(3)) > .1 && Math.abs(OI.controllerZero.getRawAxis(2)) > .1 && unlockState == 2) {
      Robot.lift.unlockBotLift(Value.kForward); 
      unlockState = 1; 
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
