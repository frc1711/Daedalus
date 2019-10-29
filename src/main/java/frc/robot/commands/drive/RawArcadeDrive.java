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

  /* if (Robot.driveSystem.isGyroConnected()) {

      tipAngle = Robot.driveSystem.getGyroRoll() - initAngle; 
      SmartDashboard.putNumber("TIP ANGLE", tipAngle); 

      if (tipAngle < 7 || tipAngle > 60) {

        SmartDashboard.putBoolean("TIP WARNING", false);

        if (Math.abs(OI.controllerZero.getRawAxis(1)) > .1 || Math.abs(OI.controllerZero.getRawAxis(4)) > .1) {
         
          Robot.driveSystem.arcadeDrive(OI.controllerZero.getRawAxis(1), OI.controllerZero.getRawAxis(4)); 
        
        } else {
        
          Robot.driveSystem.arcadeDrive(0, 0);
        
        }
      } else if (tipAngle >= 7 && tipAngle < 14) {
        
        SmartDashboard.putBoolean("TIP WARNING", true); 
        Robot.driveSystem.arcadeDrive(0.5 * OI.controllerZero.getRawAxis(1), 0.5 * OI.controllerZero.getRawAxis(4));
      
      } else if (tipAngle >= 7 && tipAngle < 8.5) {
      
        SmartDashboard.putBoolean("TIP WARNING", true); 
        Robot.driveSystem.arcadeDrive(-.5, 0); 
      
      } else if (tipAngle >= 14) {
      
        SmartDashboard.putBoolean("TIP WARNING", true); 
        Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kReverse); 
        Robot.driveSystem.arcadeDrive(-1, 0); 
      
      }
    } else {
      
      Robot.driveSystem.arcadeDrive(0.8*OI.controllerZero.getRawAxis(1), 0.8*OI.controllerZero.getRawAxis(4)); 
    
    } */
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
