// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/***************************************************
 * WPILIB IMPORTS
 **************************************************/
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/***************************************************
 * KRAKEN MOTOR / TALONFX CONTROLLER IMPORTS
 **************************************************/
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.can.*;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {}

  /****************************************************
	 * Kraken/TalonFX - CANbus IDs
	 ***************************************************/
	private static final int leftFrontDeviceID = 4;
	private static final int leftRearDeviceID = 2;
	private static final int rightFrontDeviceID = 1;
	private static final int rightRearDeviceID = 3;

	/****************************************************
	 * Kraken/TalonFX - drive motor variables
	 ***************************************************/
	private TalonFX m_leftLeader = new TalonFX(leftFrontDeviceID, "rio");
	private TalonFX m_leftFollower = new TalonFX(leftRearDeviceID, "rio");
	private TalonFX m_rightLeader = new TalonFX(rightFrontDeviceID, "rio");
	private TalonFX m_rightFollower = new TalonFX(rightRearDeviceID, "rio");


  /****************************************************
	 * Kraken/TalonFX - "sensor" aka encoder
	 ***************************************************/
  //empty for now

	/****************************************************
	 * Kraken - Setup
   ***************************************************/
  /** Creates a new configuration object for a TalonFX motor controller and stores it in
    * the leftConfiguration variable. This configuration object can later be used to set
    * specific parameters or settings for controlling the motor. Each side of the
    * drivetrain will have its own setup.*/

  leftDriveMotorGroup = new TalonFXConfiguration();
  rightDriveMotorGroup = new TalonFXConfiguration();  

  /** Talon FX has a set of inverts that are specific to it, TalonFXInvertType.Clockwise
    * and TalonFXInvertType.CounterClockwise. These new inverts allow the user to know
    * exactly what direction the Krakens will spin. These inverts are from the
    * perspective of looking at the mounting face of the motor; note that the standard
    * AndyMark gearbox reverses that rotation.
    *
    * rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; sets
    * the Inverted property of the MotorOutput object in the rightConfiguration to
    * Clockwise_Positive. This means that the motor's rotation will be considered
    * positive when it turns clockwise, essentially configuring the motor controller
    * to interpret clockwise rotation as positive and possibly affecting the direction
    * in which the motor operates.*/

  leftDriveMotorGroup.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
  rightDriveMotorGroup.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  
    /** Configure the m_leftFront motor by applying a configuration represented by
    * the leftConfiguration object through a configurator retrieved using the
    * getConfigurator() method. */

  m_leftLeader.getConfigurator().apply(leftDriveMotorGroup);
  m_leftFollower.getConfigurator().apply(leftDriveMotorGroup);
  m_rightLeader.getConfigurator().apply(rightDriveMotorGroup);
  m_rightFollower.getConfigurator().apply(rightDriveMotorGroup);

  /** The Follower() feature of the Talon FX/SRX and Victor SPX is a convenient method
    * to keep two or more motor controller outputs consistent. Follower() is intended
    * to match the direction and rotation of the master (or drive in the opposite 
    * direction depending on mechanical orientation)). NOTE: MotorControllerGroup() is
    * deprecated as of 2024 */

  m_leftFollower.setControl(new Follower(m_leftLeader.getDeviceID(), false));
  m_rightFollower.setControl(new Follower(m_rightLeader.getDeviceID(), false));

  /****************************************************
	 * WPILIB MOTOR SAFETY
	 ***************************************************/
  // From WPILib - Motor Safety {needed; long explanation in link below}
  // setSafetyEnabled() 

  m_leftLeader.setSafetyEnabled(true);
  m_rightLeader.setSafetyEnabled(true);
  
  /****************************************************
	 * Kraken/TalonFX - CURRENT LIMITS
	 ***************************************************/
  // NEEDED - empty for now

  /****************************************************
	 * DIFFERENTIAL DRIVE
	 ***************************************************/
  // Problems here - DD doesn't work with TalonFX
  
  DifferentialDrive differentialDrive = new DifferentialDrive(leftDriveMotorGroup::set, rightDriveMotorGroup::set);
  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

/***************************************************
 * DRIVE DOCUMENTATION & LINKS
 **************************************************/
// https://v5.docs.ctr-electronics.com/en/stable/ch13_MC.html
// https://api.ctr-electronics.com/phoenix6/release/cpp/classctre_1_1phoenix6_1_1hardware_1_1_talon_f_x.html
// https://docs.wcproducts.com/kraken-x60/kraken-x60-+-talonfx/overview-and-features
// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#motor-safety
// https://v6.docs.ctr-electronics.com/en/2024/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
