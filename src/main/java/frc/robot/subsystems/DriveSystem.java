/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveSystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX frontLeftDrive;
	public WPI_TalonSRX frontRightDrive;
	public WPI_TalonSRX rearLeftDrive;
  public WPI_TalonSRX rearRightDrive;

  public AHRS gyro;

  public DifferentialDrive robotDrive;

  SpeedControllerGroup leftSideDrive; 
  SpeedControllerGroup rightSideDrive; 

  public DriveSystem() {
    frontLeftDrive = new WPI_TalonSRX(RobotMap.FLD); 
    frontRightDrive = new WPI_TalonSRX(RobotMap.FRD);
    rearRightDrive = new WPI_TalonSRX(RobotMap.RRD);
    rearLeftDrive = new WPI_TalonSRX(RobotMap.RLD);

    leftSideDrive = new SpeedControllerGroup(frontLeftDrive, rearLeftDrive);
    rightSideDrive = new SpeedControllerGroup(frontRightDrive, rearRightDrive); 

    gyro = new AHRS(SerialPort.Port.kUSB);

    robotDrive = new DifferentialDrive (leftSideDrive, rightSideDrive);
    
  }
  public void stopRobot() {
    frontLeftDrive.set(0);
    frontRightDrive.set(0);
    rearLeftDrive.set(0);
    rearRightDrive.set(0); 
  }

  public double getGyroAngle()
    {
    	return gyro.getAngle();
    }
    
    public void zeroGyro()
    {
    	gyro.zeroYaw();
    }
    
    public boolean isGyroConnected()
    {
    	return gyro.isConnected();
    }
    
    public boolean isGyroCalibrating()
    {
    	return gyro.isCalibrating();
    }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
