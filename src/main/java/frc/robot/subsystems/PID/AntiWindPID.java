/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.PID;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 This file contains the source for a generic PID algorithm.  See
 the comments below for the actual formula.
 */

/**
 *
 * @author mwcardwell
 */



public class AntiWindPID extends Subsystem {
  // PID configurations
  private float sampleTime;
  private float Kp;
  private float Ki;
  private float Kd;
  private float DfilterTime;
  private float recieveCount; 
  private float measured;     // Measured value
  private float setpoint;     // Target value
  private double netKP = 0; 
  private double netKI = 0; 
  private double netKD = 0; 
  // Internal variables for storing PID history
  private float ePn_1;
  private float eDfn_1;
  private float eDfn_2;
  
  NetworkTableEntry kPNT; 
  NetworkTableEntry kDNT; 
  NetworkTableEntry kINT; 

  NetworkTable table; 
  NetworkTableInstance inst; 

/**************************************************************************** 
Constructor.  Initialize the PID structure so that it may be used by the
algorithm.  Only the "historic" variables are modified.  It does not
do any argument checking to verify the results are reasonable.
****************************************************************************/
public AntiWindPID() {
  inst = NetworkTableInstance.getDefault(); 
  table = inst.getTable("SmartDashboard"); 

  SmartDashboard.putNumber("Ki", 0); 
  SmartDashboard.putNumber("Kp", 0); 
  SmartDashboard.putNumber("Kd", 0); 

  ePn_1 = 0.0f;
  eDfn_1 = 0.0f;
  eDfn_2 = 0.0f;
}

public void setMeasured (float m) {
  measured = m;
}

public float getMeasured()
{
  return measured;
}

public void setSetpoint (float s) {
  setpoint = s;
}

public float getSetpoint() {
  return setpoint;
}

public void setKp (float k)
{
  Kp = k;
}

public void setKi (float k)
{
  Ki = k;
}

public void setKd (float k)
{
  Kd = k;
}

public void setSampleTime(float s)
{
  sampleTime = s;
}

public void setDfilterTime(float d)
{
  DfilterTime = d;   
}


/*
  Sets all non config related items within the pid structure to zero.
*/
public void clear() {
  measured = 0.0f;
  setpoint = 0.0f;
  ePn_1 = 0.0f;
  eDfn_1 = 0.0f;
  eDfn_2 = 0.0f;

}

/*
Calculate the required adjustment for the passed in PID structure.
*/
public float calculate()
{
  float eDfn;         // Filtered derivate input
  float error;        // error between setpoint and measured values
  float dun;          // Change in output determined by this function, delta un
  float filterRatio;  // The sample time dependent value within the derivative filter
  float PGain;        // The sample time dependent proportional gain
  float IGain;        // The sample time dependent integral gain
  float DGain;        // The sample time dependent derivative gain

  recieveCount++; 
  SmartDashboard.putNumber("COUTNER", recieveCount); 
  if (recieveCount > 10) {
    kPNT = table.getEntry("Kp"); 
    netKP = kPNT.getDouble(0.0);
    kDNT = table.getEntry("Kd"); 
    netKD = kDNT.getDouble(0.0); 
    kINT = table.getEntry("Ki"); 
    netKI = kINT.getDouble(0.0); 
    recieveCount = 0; 
  }
  System.out.println("NETWORK TABLE KI VALUE RAW:" + netKI); 
  Kp = (float)netKP; 
  Ki =  (float)netKI; 
  Kd = (float)netKD; 
      /* Calculate parameters which are dependent upon sample time so that 
       *  the user can use parameters that are sample time independent
       *  where sample time is the time between successive calls to this function
       *  with the same PID structure passed in */
  filterRatio = sampleTime / DfilterTime;
  PGain = Kp;
  IGain = Ki * sampleTime;
  DGain = Kd / sampleTime;

      /* Determine the error between the setpoint and the measured value */
  error = setpoint - measured;

      /* Negate the unfiltered measured feedback value for use in the derivative path.
       *  This is essentially an error calculation without the reference part of the 
       *  equation.  Omitting the reference prevents the derivative from responding to
       *  changes in the reference. 
       * Then, calculate the filtered "error" (which is really the negated feedback) */
  eDfn = ( eDfn_1 - (measured * filterRatio) ) / (filterRatio + 1);

      /* Run through the change in output calculation
       *  Y = P*(dE/dsample) + I*(E) + D*(d2E/dsample) */
  dun = ( (PGain * (error - ePn_1)                       ) + 
          (IGain * error                                      ) +
          (DGain * (eDfn - (2 * eDfn_1) + eDfn_2))    );

  /* Update all the PID history elements */
  eDfn_2 = eDfn_1;
  eDfn_1 = eDfn;
  ePn_1  = error;
  //SmartDashboard.putNumber("IGAIN", Ki*sampleTime); 
  SmartDashboard.putNumber("Error", error);   
  return dun;
}


/*
Reset the "historic" variables within the PID structure so that the derivative
 path is prepared to take action.  
*/

public void PID_ResetDerivatives() {
  /* Update all of the PID history elements to initialize the derivative filter,
   *  initialize the derivative history to zero, and initialize the error 
   *  history to the present error value */

      /* The filtered terms for the derivative get set to the negated measured
       *  value as this is what is normally passed to the filter in PID_Calculate(). */
  eDfn_2 = - measured;
  eDfn_1 = - measured;
      /* The error gets set to the difference between the setpoint and the measured
       *  values as this is what is normally passed to the Proportional gain in
       *  PID_Calculate(). */
  ePn_1  = setpoint - measured;
  
}

  @Override
  protected void initDefaultCommand() {

  }


}
