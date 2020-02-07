/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.lift;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.AuxWheel;

public class RunAuxWheel extends Command {
  private int state = 0; 
  private AuxWheel auxWheel; 
  Joystick stick; 

  public RunAuxWheel(Joystick stick) {
    requires(Robot.auxWheel);     
    this.stick = stick; 
    this.auxWheel = Robot.auxWheel; 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    auxWheel.stop(); 
    auxWheel.set(Value.kReverse);
    state = 2; 

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 

    if (state  == 1) {
      SmartDashboard.putString("Aux Wheel State:", "Forward"); 
    } else if (state == 2) {
      SmartDashboard.putString("Aux Wheel State:", "Reverse"); 
    }

    if (stick.getRawButton(10) && state == 2 || state ==  0) {
      auxWheel.set(Value.kForward); 

      if (stick.getRawButtonReleased(10)) 
        state = 1;
        
    } else if (stick.getRawButtonReleased(10) && state == 1) {
      auxWheel.set(Value.kReverse); 

      if (stick.getRawButtonReleased(10)) 
        state = 2; 

    } 
    
    //oi.controllerOne.
    if (Math.abs(stick.getRawAxis(5)) > .1) {
      auxWheel.runAuxWheel(stick.getRawAxis(5));
    } else if (stick.getRawAxis(5) == 0) {
      auxWheel.runAuxWheel(0);
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
