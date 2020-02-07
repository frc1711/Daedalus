/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.drive.LineUpDrive;
import frc.robot.subsystems.DriveSystem;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  
  public Joystick controllerZero = new Joystick(RobotMap.controllerZero);
  public Joystick controllerOne = new Joystick(RobotMap.controllerOne); 
  public Joystick controllerTwo = new Joystick(RobotMap.controllerTwo); 

  //BUTTONS
  //controller zero, drive 
  public JoystickButton ballVisionEnable = new JoystickButton(controllerZero, 2);
   
  //controller two, toggle
  public JoystickButton modeToggleButton = new JoystickButton(controllerTwo, 2); 

  public OI(DriveSystem driveSystem) {
    ballVisionEnable.whenPressed(new LineUpDrive(driveSystem)); 
  }
}
