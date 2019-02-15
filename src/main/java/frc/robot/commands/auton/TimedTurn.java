/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;
import frc.robot.RobotMap.RoboDir;

/**
 * Add your docs here.
 */
public class TimedTurn extends TimedCommand {
  /**
   * Add your docs here.
   */
  public RoboDir turnDirection;

  public TimedTurn(double timeout, RoboDir direction) {
    super(timeout);
    requires(Robot.driveSystem);
    turnDirection = direction; 
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSystem.stopRobot(); 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (turnDirection == RoboDir.LEFT) 
      Robot.driveSystem.driveDirection(0.25, RoboDir.LEFT); 
    else if (turnDirection == RoboDir.RIGHT) 
      Robot.driveSystem.driveDirection(0.25, RoboDir.LEFT); 
  }

  // Called once after timeout
  @Override
  protected void end() {
    Robot.driveSystem.stopRobot(); 
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveSystem.stopRobot(); 
  }
}
