/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.AuxWheel;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.ManipulatorHatch;
import frc.robot.subsystems.PneumaticArm;
import frc.robot.subsystems.vision.CameraConfig;

/**
 * The VM is configured to austomatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  public static RobotMap robotMap; 
  public static DriveSystem driveSystem;
  public static UsbCamera camera; 
  public static Manipulator manipulator; 
  public static ManipulatorHatch manipulatorHatch;
  public static PneumaticArm pneumaticArm; 
  public static Arm arm;
  public static Lift lift;  
  public static AuxWheel auxWheel; 
  public static OI oi;

  public boolean endGame = false; 
  public double cameraCount = 0; 
  public double endGameCounter; 

  SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
        
    //BASIC FILES AND SUBSYTEMS
    robotMap = new RobotMap();
    driveSystem = new DriveSystem(oi.controllerZero);
    manipulator = new Manipulator(oi.controllerZero); 
    arm = new Arm(oi.controllerOne); 
    auxWheel = new AuxWheel(oi.controllerOne);
    pneumaticArm = new PneumaticArm(oi.controllerOne);
    manipulatorHatch = new ManipulatorHatch(oi.controllerZero);  
    lift = new Lift(oi.controllerOne); 
    oi = new OI(driveSystem);

    //CAMERAS AND PIXYCAM
    CameraConfig.setup(); 

    UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture();
    cam0.setFPS(30);
    cam0.setResolution(480, 320);   
  }


  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run(); 
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }
}
