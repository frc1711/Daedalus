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

//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
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
  public double posAbsZero; 
  public double posOne; 
  public double posTwo; 
  public double posThree; 
  public double rightAngle; 
  public int unitsPerRotation; 
  public double baseSpeed; 

  public Arm() {
    armTalon = new WPI_TalonSRX(RobotMap.armTalon); 
    
    armMin = -1; 
    posZero = 1912; //-799
    posOne = 1355; //-1356
    posTwo = -2220; //-2823
    posThree = -1520; //-2195
    posAbsZero = 0; 
    rightAngle = 0; //1422

    unitsPerRotation = 4096; //This was 1205 
    
    armTalon.setSelectedSensorPosition(0);
    //armTalon.selectProfileSlot(0, 0);
    //corectionary values
    armTalon.config_kF(0, 0.0); //feed forward gain
    armTalon.config_kP(0, 0.8); //proportional
    armTalon.config_kI(0, 0.0001); 
    armTalon.config_kD(0, 5.5);
    //max speed
    armTalon.configClosedLoopPeakOutput(0, 0.7);
    //allowable error
    armTalon.configAllowableClosedloopError(0, 100);
    //(how fast you get there) how many counts per 100 milliseconds you can go  (rate of change of duty cycle)
    armTalon.configMotionAcceleration(250); //250 ON ROBOT
    //how fast you go once you're there
    armTalon.configMotionCruiseVelocity(100); //100 on robot
    
    armTalon.config_kF(1, 0.0); 
    armTalon.config_kP(1, 0.8); 
    armTalon.config_kI(1, 0.0001);
    armTalon.config_kD(1, 5.5);
    
    armTalon.configClosedLoopPeakOutput(1, 0.7); 

    armTalon.configAllowableClosedloopError(1, 50); 

    armTalon.configMotionAcceleration(50); //250 ON ROBOT

    armTalon.configMotionCruiseVelocity(50); //380 ON ROBOT

  }

  public void stopArm() {
    armTalon.set(0); 
  }

  public void runArm(double speed) {
    armTalon.set(speed); 
  }
  public double returnArmPos() {
    if(OI.controllerOne.getRawButtonReleased(2)) 
      return armTalon.getMotorOutputPercent(); 
    else {
      return 0; 
    }
  }
  public void runPIDArm (double pos) {
    double tester = armTalon.getActiveTrajectoryVelocity(); 
    double theSpeed = armTalon.getMotorOutputPercent(); 
    SmartDashboard.putNumber("Velocity", tester); 
    SmartDashboard.putNumber("Position", pos); 
    SmartDashboard.putNumber("armTalon Motor Output Percent", theSpeed);
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
