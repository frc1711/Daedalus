/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.PneumaticArm;

public class RunPneumaticArm extends Command {
private int state = 0;  
private PneumaticArm pneumaticArm; 
private Joystick stick; 

  public RunPneumaticArm(PneumaticArm pneumaticArm, Joystick stick) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(pneumaticArm); 
    this.stick = stick; 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() { 
    pneumaticArm.set(DoubleSolenoid.Value.kReverse); 
    state = 2; 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //this has to work so that it can turn the solenoid on and off
    if (state  == 1) {
      SmartDashboard.putString("Pneumatic Arm State:", "Forward"); 
    } else if (state == 2) {
      SmartDashboard.putString("Pneumatic Arm State:", "Reverse"); 
    }
    
    if (stick.getPOV() == 180 && state == 1 ) {
      
      pneumaticArm.set(DoubleSolenoid.Value.kReverse); 
      state = 2; 

    } else if (stick.getPOV() == 0 && state == 2) {
      
      pneumaticArm.set(DoubleSolenoid.Value.kForward);
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
