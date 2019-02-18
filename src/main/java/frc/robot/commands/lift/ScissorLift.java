/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.lift;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    Robot.lift.botLift.set(Value.kReverse);
    state = 2;  
    Robot.lift.unlockBot.set(Value.kReverse); 
    unlockState = 2; 
    System.out.println("Initialize");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (state  == 1) {
      SmartDashboard.putString("Scissor Lift State:", "Forward"); 
    } else if (state == 2) {
      SmartDashboard.putString("Scissor Lift State:", "Reverse"); 
    }
    if (unlockState  == 1) {
      SmartDashboard.putString("Pins State:", "Forward"); 
    } else if (unlockState == 2) {
      SmartDashboard.putString("Pins State:", "Reverse"); 
    }
    //TODO: put in two other safety protocols 
    boolean onOff = OI.climbSafety.get() && OI.liftDeployZero.get() && OI.liftDeployOne.get();
    if ( onOff && state == 2) {
      Robot.lift.botLift.set(DoubleSolenoid.Value.kForward); 
      if (OI.controllerOne.getRawButtonReleased(5) && OI.controllerOne.getRawButtonReleased(6) && OI.controllerZero.getRawButtonReleased(7))
        state = 1; 
      System.out.println("Forward" + state); 
    } else if (onOff && state == 1) {
      Robot.lift.botLift.set(DoubleSolenoid.Value.kReverse); 
      if (OI.controllerOne.getRawButtonReleased(5) && OI.controllerOne.getRawButtonReleased(6) && OI.controllerZero.getRawButtonReleased(7))
        state = 2; 
      System.out.println("Reverse" + state); 
    }
    //TODO: replace this with the actual buton later
    boolean onOffUnlock = OI.climbSafety.get() && OI.liftDeployZero.get() && OI.armPosZero.get();
    
    if (onOffUnlock && unlockState == 2) {
     
      Robot.lift.unlockBot.set(DoubleSolenoid.Value.kReverse); 
      
      if (OI.controllerZero.getRawButtonReleased(7) && OI.controllerOne.getRawButtonReleased(5) && OI.controllerOne.getRawButtonReleased(2)) 
        unlockState = 1; 
      System.out.println("Back");
    
    } else if ( onOffUnlock && unlockState == 1) {
      Robot.lift.unlockBot.set(DoubleSolenoid.Value.kForward); 
      if (OI.controllerZero.getRawButtonReleased(7) && OI.controllerOne.getRawButtonReleased(5) && OI.controllerOne.getRawButtonReleased(2)) 
         unlockState = 2; 
      System.out.println("Forward");
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
