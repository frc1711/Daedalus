/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX armTalon;
  public DigitalInput modeToggle; 
  public double velCounter; 
  public double armMin; 
  public double posZero; 
<<<<<<< HEAD
  public double home; 
  public double hatchPosOne; 
  public double cargoPosOne; 
  public double hatchPosTwo; 
  public double cargoPosTwo; 
  public double hatchPosThree; 
  public double cargoPosThree; 
  public double hatchLift; 

=======
  public double posAbsZero; 
  public double hatchPosOne; 
  public double posOne; 
  public double hatchPosTwo; 
  public double posTwo; 
  public double hatchPosThree; 
  public double posThree; 
  public double rightAngle; 
>>>>>>> 545f5dca98d9702b3fc825682399047d239f3bc9
  public double posDepo; 
  public double hatchPosDepo; 
  public int unitsPerRotation; 
  public double baseSpeed; 
<<<<<<< HEAD
=======
  public double hatchLift; 
>>>>>>> 545f5dca98d9702b3fc825682399047d239f3bc9

  public Arm() {
    armTalon = new WPI_TalonSRX(RobotMap.armTalon); 
    modeToggle = new DigitalInput(RobotMap.modeToggle); 
    
<<<<<<< HEAD
    posZero = 1612; //-799
    home = 0; 

    cargoPosOne = 1055; //-1356
    cargoPosTwo = 2400; //-2823
    cargoPosThree = 1580; //-2195
=======
    armMin = -1; 
    posZero = 1612; //-799
    posOne = 1055; //-1356
    posTwo = 2400; //-2823
    posThree = 1580; //-2195
>>>>>>> 545f5dca98d9702b3fc825682399047d239f3bc9
    hatchLift = 600; 

    hatchPosOne = 1500; 
    hatchPosTwo = 2600; 
    hatchPosThree = 2100;

<<<<<<< HEAD
    armMin = -1; 

    unitsPerRotation = 4096; //This was 1205 
    velCounter = 0; 

    armTalon.setSelectedSensorPosition(0); //Sets the selected sensor (armTalon encoder) to zero upon enabling. 

    armTalon.config_kF(1, 0.0); //Configuring the 'kf' of the talon. This is a special value in relation to the talons. 
    armTalon.config_kP(1, 1.25); //Configure the 'kP' of the PID loop-- how much proportional gain they should have. How fast it will increase. 
    armTalon.config_kI(1, 0.000);//Configure the 'kI' of the PID loop-- how much adding of the area under the curve should happen. The kI is zero here because the application required that the loop not error-correct itself. It was good enough when it wasn't exact. 
    armTalon.config_kD(1, 3); //Configure the 'kD' of the PID loop-- how much the slope of it should be 'smoothed out'. The kD is high here because the loop is being applied to a non-linear application. 
=======
    posAbsZero = 0; 
    rightAngle = 1350; //1422sn-

    unitsPerRotation = 4096; //This was 1205 
    
    velCounter = 0; 

    //armTalon.setIntegralAccumulator(-40000);
    armTalon.setSelectedSensorPosition(0);
    //armTalon.selectProfileSlot(0, 0);
    //corectionary values
    armTalon.config_kF(0, 0.0); //feed forward gain
    armTalon.config_kP(0, .8); //proportional
    armTalon.config_kI(0, 0.000); 
    armTalon.config_kD(0, 5.5);
    //max speed
    armTalon.configClosedLoopPeakOutput(0, 0.7);
    //allowable error
    armTalon.config_kF(1, 0.0); 
    armTalon.config_kP(1, 1.25); 
    armTalon.config_kI(1, 0.000);
    armTalon.config_kD(1, 3);
>>>>>>> 545f5dca98d9702b3fc825682399047d239f3bc9
    
    armTalon.configClosedLoopPeakOutput(1, 0.7); 

    armTalon.configAllowableClosedloopError(1, 50); 

<<<<<<< HEAD
    armTalon.configMotionAcceleration(50);

    armTalon.configMotionCruiseVelocity(50);
=======
    armTalon.config_kF(2, 0.0); 
    armTalon.config_kP(2, 0.8); 
    armTalon.config_kI(2, 0.0001); 
    armTalon.config_kD(2, 5.5); 

    armTalon.configClosedLoopPeakOutput(2, .7); 
    armTalon.configAllowableClosedloopError(2, 50); 

    armTalon.config_kF(3, 0.0); 
    armTalon.config_kP(3, 0.8); 
    armTalon.config_kI(3, 0.00025); 
    armTalon.config_kD(3, 5.5); 

    armTalon.configClosedLoopPeakOutput(3, .7); 
    armTalon.configAllowableClosedloopError(3, 50); 

    armTalon.configMotionAcceleration(50); //250 ON ROBOT

    armTalon.configMotionCruiseVelocity(50); //380 ON ROBOT

>>>>>>> 545f5dca98d9702b3fc825682399047d239f3bc9
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
    //SmartDashboard.putNumber("Position", pos); 
    armTalon.set(ControlMode.MotionMagic, pos); 
  }

  public void stopPIDPos (double vel, double encPos, double targetPos, double MOP) {
    if (vel == 0) {
      velCounter++;
    } else {
      velCounter = 0; 
    }

    if (encPos >= targetPos-300 && encPos <= targetPos+300 && vel == 0 && velCounter == 15) {
<<<<<<< HEAD
      
      Robot.arm.runPIDArm(encPos); 
       
      velCounter = 0;  
=======
//SmartDashboard.putBoolean("HOLDING", true); 
  //    SmartDashboard.putNumber("VEL", vel); 
    //  SmartDashboard.putNumber("ENCPOS", encPos); 
      //SmartDashboard.putNumber("TARGETPOS", targetPos); 
      //SmartDashboard.putNumber("MOP", MOP); 
      //armTalon.set(MOP);
      Robot.arm.runPIDArm(encPos); 
       
     // Robot.arm.runPIDArm(encPos); 
      velCounter = 0;  
    } else {
      //SmartDashboard.putBoolean("HOLDING", false); 
>>>>>>> 545f5dca98d9702b3fc825682399047d239f3bc9
    } 
  }

  public int getSensorValue() {
    return armTalon.getSelectedSensorPosition(0);
  }

  public boolean getControllerMode() {
    return modeToggle.get(); 
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}