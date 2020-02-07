/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.lift;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Lift;

public class ScissorLift extends Command {
  private int state = 0; 
  private int unlockState = 0; 
  private Lift lift; 
  Joystick stick; 

  public ScissorLift(Lift lift, Joystick stick) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(lift); 
    this.lift = lift; 
    this.stick = stick; 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    lift.setBotLift(Value.kReverse);
    state = 2;  
    lift.unlockBotLift(Value.kReverse); 
    unlockState = 2; 
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

    //triggers
    boolean onOff =  stick.getRawButton(5) && stick.getRawButton(6);
    if ( onOff && state == 2) {
      lift.setBotLift(DoubleSolenoid.Value.kForward); 
      if (stick.getRawButtonReleased(5) && stick.getRawButtonReleased(6))
        state = 1; 
    } else if (onOff && state == 1) {
      lift.setBotLift(DoubleSolenoid.Value.kReverse); 
      if (stick.getRawButtonReleased(5) && stick.getRawButtonReleased(6))
        state = 2; 
    }

    boolean onOffUnlock = stick.getRawButton(5) && stick.getRawButton(2);
    if (onOffUnlock && unlockState == 2) {
     
      lift.unlockBotLift(DoubleSolenoid.Value.kForward); 
      
      if (stick.getRawButtonReleased(5) && stick.getRawButtonReleased(2)) 
        unlockState = 1; 
    
    } else if ( onOffUnlock && unlockState == 1) {
      lift.unlockBotLift(DoubleSolenoid.Value.kReverse);

      if (stick.getRawButtonReleased(5) && stick.getRawButtonReleased(2)) 
        unlockState = 2; 
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
