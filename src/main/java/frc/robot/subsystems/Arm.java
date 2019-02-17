/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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
  public double armMax; 
  public int unitsPerRotation; 
  public double baseSpeed; 

  public Arm() {
    armTalon = new WPI_TalonSRX(RobotMap.armTalon); 
    
    /*armMin = -12288; 
    armMax = 12288;
    unitsPerRotation = 4096;
    baseSpeed = 0.1;

    armTalon.setSelectedSensorPosition(0);
    armTalon.selectProfileSlot(0, 0);
    armTalon.config_kF(0, 0.0);
    armTalon.config_kP(0, 0.25);
    armTalon.config_kI(0, 0.0);
    armTalon.config_kD(0, 0.0);
    armTalon.configClosedLoopPeakOutput(0, 0.2);
    armTalon.configAllowableClosedloopError(0, 2048);
    armTalon.configMotionAcceleration(13585);
    armTalon.configMotionCruiseVelocity(13585);
    */
    
    

  }

  public void stopArm() {
    armTalon.set(0); 
  }

  public void runArm(double speed) {
    armTalon.config_kP(0, 0.25);
    armTalon.configClosedLoopPeakOutput(0, 0.2);  
    armTalon.set(speed); 
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
