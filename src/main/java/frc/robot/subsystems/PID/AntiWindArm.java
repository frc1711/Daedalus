/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.PID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class AntiWindArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final float POSITION_KP = 0.00000150f;      // Proportional gain for the motor PID
  private final float POSITION_KI = 0.000075f;        // Integral gain for the motor PID
  private final float POSITION_KD = 0.0f;             // Derivative gain for the motor PID
  private final float POSITION_DFILTER = 1.0f;        // One second derivative filter time for the PID
  private final float POSITION_SAMPLE_TIME = 20E-3f;  // How often the current position is sampled
  
  private AntiWindPID pid;

  private float measuredPosition; 
  public float motorDC; 

 // private Thread measureThread; 
  
  public WPI_TalonSRX armTalon;

  public AntiWindArm() {

    pid = new AntiWindPID(); 

    pid.setKp(POSITION_KP); 
    pid.setKi(POSITION_KI); 
    pid.setKd(POSITION_KD); 
    pid.setDFilterTime(POSITION_DFILTER); 
    pid.setSampleTime(POSITION_SAMPLE_TIME); 

    pid.setMeasured(0.0f); 
    pid.setTarget(0.0f); 

    armTalon = new WPI_TalonSRX(RobotMap.armTalon); 

    motorDC = 0.0f; 

  }

  public void setTargetPosition (float pos) {
    pid.setTarget(pos); 
  }

  public float getTargetPosition() {
    return pid.getTarget(); 
  }

  public double getDC() {
    return armTalon.getMotorOutputPercent(); 
  }

  public float getCurrentPosition() {
    return measuredPosition; 
  }

  public void setCurrentPosition (float p) {
    measuredPosition = p; 
  }

  public void runArmTalon (double speed) { 
    armTalon.set(ControlMode.PercentOutput, speed); 
  }

  public void positionControl() {
    float adjustment; 

    measuredPosition = armTalon.getSelectedSensorPosition(); 
 
    pid.setMeasured(measuredPosition); 

    adjustment = pid.calcPID(); 

    motorDC += adjustment; 
    
    if (motorDC > 1.0f) {
      motorDC = 1.0f; 
    } else if (motorDC < -1.0f) {
      motorDC = -1.0f; 
    }


  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
