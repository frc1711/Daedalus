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
import frc.robot.subsystems.DriveSystem;

public class RawArcadeDrive extends Command {
  public double tipAngle; 
  public double initAngle; 

  public RawArcadeDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSystem); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSystem.stopRobot(); 
    if (Robot.driveSystem.isGyroConnected()) 
      initAngle = Robot.driveSystem.getGyroRoll(); 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (Robot.driveSystem.isGyroConnected()) {
      tipAngle = Robot.driveSystem.getGyroRoll() - initAngle; 
      SmartDashboard.putNumber("TIP ANGLE", tipAngle); 
      if (tipAngle < 3 || tipAngle > 60) {
        SmartDashboard.putBoolean("TIP WARNING", false); 
        if (Math.abs(OI.controllerZero.getRawAxis(1)) > .1 || Math.abs(OI.controllerZero.getRawAxis(4)) > .1) {
          Robot.driveSystem.arcadeDrive(OI.controllerZero.getRawAxis(1), OI.controllerZero.getRawAxis(4)); 
        } else {
          Robot.driveSystem.arcadeDrive(0, 0);
        }
      } else if (tipAngle >= 3 && tipAngle < 7) {
        SmartDashboard.putBoolean("TIP WARNING", true); 
        Robot.driveSystem.arcadeDrive(0.5 * OI.controllerZero.getRawAxis(1), 0.5 * OI.controllerZero.getRawAxis(4));
      } else if (tipAngle >= 7 && tipAngle < 8.5) {
        SmartDashboard.putBoolean("TIP WARNING", true); 
        Robot.driveSystem.arcadeDrive(-1, 0); 
      } else if (tipAngle >= 8.5) {
        SmartDashboard.putBoolean("TIP WARNING", true); 
        Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kReverse); 
        Robot.driveSystem.arcadeDrive(-1, 0); 
      }
    } else {
      Robot.driveSystem.arcadeDrive(OI.controllerZero.getRawAxis(1), OI.controllerZero.getRawAxis(4)); 
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