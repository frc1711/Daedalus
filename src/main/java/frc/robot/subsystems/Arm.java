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
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX armTalon;
  
  public double armMin; 
  public double posOne; 
  public int unitsPerRotation; 
  public double baseSpeed; 

  public Arm() {
    armTalon = new WPI_TalonSRX(RobotMap.armTalon); 
    
    armMin = -1; 
    posOne = 3737;
    unitsPerRotation = 1205;
    baseSpeed = 0.5;

    armTalon.setSelectedSensorPosition(0);
    armTalon.selectProfileSlot(0, 0);
    //corectionary values
    armTalon.config_kF(0, 0.0); 
    armTalon.config_kP(0, 0.125);
    armTalon.config_kI(0, 0.0);
    armTalon.config_kD(0, 0.0);
    //max speed
    armTalon.configClosedLoopPeakOutput(0, 0.5);
    //allowable error
    armTalon.configAllowableClosedloopError(0, 70);
    //(how fast you get there) how many counts per 100 milliseconds you can go  (rate of change of duty cycle)
    armTalon.configMotionAcceleration(500);
    //how fast you go once you're there
    armTalon.configMotionCruiseVelocity(500);
    
    
    

  }

  public void stopArm() {
    armTalon.set(0); 
  }

  public void runArm(double speed) {
    armTalon.set(speed); 
  }

  public void runPIDArm (double speed) {
    System.out.println("Go " + speed); 
    armTalon.set(ControlMode.MotionMagic, speed); 
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
