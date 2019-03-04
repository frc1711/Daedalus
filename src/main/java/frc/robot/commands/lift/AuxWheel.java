/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.lift;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    Robot.lift.auxWheelSolenoid.set(Value.kReverse);
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

    if (OI.auxWheelButton.get() && state == 2 || state ==  0) {
      Robot.lift.auxWheelSolenoid.set(Value.kForward); 
      System.out.println("ForwardAux");
      if (OI.controllerOne.getRawButtonReleased(10)) 
        state = 1;
    } else if (OI.auxWheelButton.get() && state == 1) {
      Robot.lift.auxWheelSolenoid.set(Value.kReverse); 
      System.out.println("BackwardAux");
      if (OI.controllerOne.getRawButtonReleased(10)) 
        state = 2; 
    } 
    
    if (Math.abs(OI.controllerOne.getRawAxis(5)) > .1) {
      Robot.lift.runAuxWheel(OI.controllerOne.getRawAxis(5));
    } else if (OI.controllerOne.getRawAxis(5) == 0) {
      Robot.lift.runAuxWheel(0);
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
