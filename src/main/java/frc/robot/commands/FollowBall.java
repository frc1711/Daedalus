/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap.RoboDir;
import frc.robot.subsystems.vision.BallTrack;

public class FollowBall extends Command {

  double ballX = SmartDashboard.getNumber("Ball X", 700);
  double ballY = SmartDashboard.getNumber("Ball Y", 700); 
  double ballAngle = SmartDashboard.getNumber("Ball Angle", 700); 

  public FollowBall() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSystem.stopRobot();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (OI.visionEnable.get()) {
      if(ballX < 150) {
        Robot.driveSystem.driveDirection(.3, RoboDir.LEFT); 
      } else if (ballX > 160) {
        Robot.driveSystem.driveDirection(.3, RoboDir.RIGHT); 
      } else if (ballX >= 150 && ballX <= 160) {
        Robot.driveSystem.driveDirection(.3, RoboDir.STRAIGHT); 
      }
      
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
