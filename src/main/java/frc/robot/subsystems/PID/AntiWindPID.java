/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.PID;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class AntiWindPID extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private float sampleTime; 
  private float kP; 
  private float kI; 
  private float kD; 
  private float dFilterTime; 

  private float measured; 
  private float target; 

  private float ePn_1; 
  private float eDfn_1; 
  private float eDfn_2; 

  public AntiWindPID() {
    ePn_1 = 0.0f; 
    eDfn_1 = 0.0f; 
    eDfn_2 = 0.0f; 
  }

  public void setMeasured (float mes) {
    measured = mes; 
  }

  public float getMeasured() {
    return measured; 
  }

  public void setTarget (float point) {
    target = point; 
  }

  public float getTarget() { 
    return target; 
  }

  public void setKp (float prop) {
    kP = prop;
  }
  
  public void setKi (float intgrl) {
    kI = intgrl; 
  } 

  public void setKd (float deriv) {
    kD = deriv; 
  }

  public void setSampleTime (float time) {
    sampleTime = time; 
  }

  public void setDFilterTime (float time) {
    dFilterTime = time; 
  }

  public void clear() {
    measured = 0.0f; 
    target = 0.0f; 
    ePn_1 = 0.0f; 
    eDfn_1 = 0.0f; 
    eDfn_2 = 0.0f; 
  }

  public float calcPID() {
    float eDfn; 
    float error; 
    float dun; 
    float filterRatio; 
    float PGain; 
    float IGain; 
    float DGain; 

    filterRatio = sampleTime / dFilterTime;
    PGain = kP; 
    IGain = kI * sampleTime; 
    DGain = kD / sampleTime; 
    
    error = target - measured; 

    eDfn = ( eDfn_1 - (measured * filterRatio) ) / (filterRatio + 1); 

    dun = ( (PGain * (error - ePn_1)                       ) + 
           (IGain * error                                      ) +
            (DGain * (eDfn - (2 * eDfn_1) + eDfn_2))    );

    eDfn_2 = eDfn_1; 
    eDfn_1 = eDfn; 
    ePn_1 = error; 

    return dun; 
  }

  public void PID_ResetDerivatives() {
    eDfn_2 = - measured; 
    eDfn_1 = -measured; 

    ePn_1 = target - measured; 
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
