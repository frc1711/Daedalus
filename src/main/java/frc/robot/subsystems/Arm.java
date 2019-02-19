/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX armTalon;
  
  public double armMin; 
  public double posZero; 
  public double posOne; 
  public double posTwo; 
  public double posThree; 
  public double rightAngle; 
  public int unitsPerRotation; 
  public double baseSpeed; 

  public Arm() {
    armTalon = new WPI_TalonSRX(RobotMap.armTalon); 
    
    armMin = -1; 
    posZero = 1000; //TODO: fix pos one and pos two to be in order bc we're not lazy programmers (jk but we do need to fix it)
    posOne = 500; 
    posTwo = 400; 
    posThree = 1100; 
    rightAngle = 2000;

    unitsPerRotation = 4096; //This was 1205 
    
    armTalon.setSelectedSensorPosition(0);
    armTalon.selectProfileSlot(0, 0);
    //corectionary values
    armTalon.config_kF(0, 0.0); //feed forward gain
    armTalon.config_kP(0, 0.3); //proportional
    armTalon.config_kI(0, 0.0005); 
    armTalon.config_kD(0, 0.2);
    //max speed
    armTalon.configClosedLoopPeakOutput(0, 0.7);
    //allowable error
    armTalon.configAllowableClosedloopError(0, 185);
    //(how fast you get there) how many counts per 100 milliseconds you can go  (rate of change of duty cycle)
    armTalon.configMotionAcceleration(250);
    //how fast you go once you're there
    armTalon.configMotionCruiseVelocity(200);
    
    

  }

  public void stopArm() {
    armTalon.set(0); 
  }

  public void runArm(double speed) {
    armTalon.set(speed); 
  }

  public void runPIDArm (double pos) {
    double tester = armTalon.getActiveTrajectoryVelocity(); 
    SmartDashboard.putNumber("Velocity", tester); 
    SmartDashboard.putNumber("Position", pos); 
    armTalon.set(ControlMode.MotionMagic, pos); 
  }

  public int getSensorValue() {
    return armTalon.getSelectedSensorPosition(0);
  }

  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
