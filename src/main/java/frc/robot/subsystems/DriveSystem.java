/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

import frc.robot.RobotMap.RoboDir;
import frc.robot.commands.drive.RawArcadeDrive;

/**
 * Add your docs here.
 */
public class DriveSystem extends Subsystem {

  private CANSparkMax frontLeftDrive; 
  private CANSparkMax frontRightDrive; 
  private CANSparkMax rearLeftDrive; 
  private CANSparkMax rearRightDrive; 

  private DifferentialDrive robotDrive;

  SpeedControllerGroup leftSideDrive; 
  SpeedControllerGroup rightSideDrive; 

  Joystick stick; 
  
  public DriveSystem(Joystick stick) {
    frontLeftDrive = new CANSparkMax(RobotMap.FLD, MotorType.kBrushless); 
    frontRightDrive = new CANSparkMax(RobotMap.FRD, MotorType.kBrushless); 
    rearLeftDrive = new CANSparkMax(RobotMap.RLD, MotorType.kBrushless); 
    rearRightDrive = new CANSparkMax(RobotMap.RRD, MotorType.kBrushless); 

    leftSideDrive = new SpeedControllerGroup(frontLeftDrive, rearLeftDrive);
    rightSideDrive = new SpeedControllerGroup(frontRightDrive, rearRightDrive); 

    robotDrive = new DifferentialDrive (leftSideDrive, rightSideDrive);

    this.stick = stick; 
  }
  
  public void stopRobot() {
    frontLeftDrive.set(0);
    frontRightDrive.set(0);
    rearLeftDrive.set(0);
    rearRightDrive.set(0); 
  }
  

  public void driveDirection (double speed, RoboDir direction) {
    
    if(direction == RoboDir.LEFT || direction == RoboDir.RIGHT) {
      frontLeftDrive.set(direction.getNum()*speed); 
      rearLeftDrive.set(direction.getNum()*speed); 
      frontRightDrive.set(direction.getNum()*speed); 
      rearRightDrive.set(direction.getNum()*speed); 
      SmartDashboard.putNumber("DIRECTION", direction.getNum()); 
    } else {

      frontLeftDrive.set(-speed); 
      frontRightDrive.set(speed);
      rearLeftDrive.set(-speed); 
      rearRightDrive.set(speed); 
      
    }

  }

  public void arcadeDrive (double speed, double rot) {
    robotDrive.arcadeDrive(-speed, rot); 
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new RawArcadeDrive(stick)); 
  }
}

