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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.arm.RunMotorArm;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX armTalon;

  private DigitalInput modeToggle; 

  private double velCounter; 

  private Joystick stick; 

  public Arm(Joystick stick) {
    armTalon = new WPI_TalonSRX(RobotMap.armTalon);

    modeToggle = new DigitalInput(RobotMap.modeToggle); 

    velCounter = 0; 

    armTalon.setSelectedSensorPosition(0); //Sets the selected sensor (armTalon encoder) to zero upon enabling. 

    armTalon.config_kF(0, 0.0); //Configuring the 'kf' of the talon. This is a special value in relation to the talons. 
    armTalon.config_kP(0, 1.25); //Configure the 'kP' of the PID loop-- how much proportional gain they should have. How fast it will increase. 
    armTalon.config_kI(0, 0.000); //Configure the 'kI' of the PID loop-- how much adding of the area under the curve should happen. The kI is zero here because the application required that the loop not error-correct itself. It was good enough when it wasn't exact. 
    armTalon.config_kD(0, 3); //Configure the 'kD' of the PID loop-- how much the slope of it should be 'smoothed out'. The kD is high here because the loop is being applied to a non-linear application. 
    
    armTalon.configClosedLoopPeakOutput(0, 0.7); 

    armTalon.configAllowableClosedloopError(0, 50); 

    armTalon.configMotionAcceleration(50);

    armTalon.configMotionCruiseVelocity(50);

    this.stick = stick; 
  }

  public void stopArm() {
    armTalon.set(0); 
  }

  public void runArm(double speed) {
    armTalon.set(speed); 
  }
  
  public void runPIDArm (double pos) {
    armTalon.set(ControlMode.MotionMagic, pos); 
  }

  public void stopPIDPos (double vel, double encPos, double targetPos, double MOP) {
    if (vel == 0) {
      velCounter++;
    } else {
      velCounter = 0; 
    }

    //To deal with the non-linearity of the arm PID, check if we're within 
    //300 counts either way, and if our velocity has been 0 for more than 30ms.
    if (encPos >= targetPos-300 && encPos <= targetPos+300 && vel == 0 && velCounter == 15) {
      
      runPIDArm(encPos); 
       
      velCounter = 0;  
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
    setDefaultCommand(new RunMotorArm(stick));
  }
}