/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;

public class RawArcadeDrive extends Command {
  public double tipAngle; 
  public double initAngle; 
  public boolean speed; 

  public RawArcadeDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSystem); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    speed = false; 
    Robot.driveSystem.stopRobot(); 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (OI.controllerZero.getRawButtonReleased(1)) {
      speed = !speed; 
      SmartDashboard.putBoolean("Speed On", speed); 
    }

    if (!speed) {
      Robot.driveSystem.arcadeDrive(0.8*OI.controllerZero.getRawAxis(1), 0.8* OI.controllerZero.getRawAxis(4)); 
     } else if (speed) {
      Robot.driveSystem.arcadeDrive(0.4*OI.controllerZero.getRawAxis(1), 0.5*OI.controllerZero.getRawAxis(4)); 
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
