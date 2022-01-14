/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5427.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.usfirst.frc.team5427.robot.commands.auto.MoveClimberLegAuto;

import org.usfirst.frc.team5427.robot.commands.auto.presets.*;

import org.usfirst.frc.team5427.robot.subsystems.Arm;
import org.usfirst.frc.team5427.robot.subsystems.ClimberArm;
import org.usfirst.frc.team5427.robot.subsystems.ClimberLeg;
import org.usfirst.frc.team5427.robot.subsystems.ClimberWheel;
import org.usfirst.frc.team5427.robot.subsystems.DriveTrain;
import org.usfirst.frc.team5427.robot.subsystems.Intake;
import org.usfirst.frc.team5427.robot.subsystems.Wrist;
import org.usfirst.frc.team5427.util.Config;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot
{

    /**
     * The Operator Interface.
     */
    public static OI oi;

    /**
     * The motor controller for the top left motor of the drive train. 
     */
    public static SpeedController driveLeftTop;

    /**
     * The motor controller for the middle left motor of the drive train. 
     */
    public static SpeedController driveLeftMiddle;

    /**
     * The motor controller for the bottom left motor of the drive train. 
     */
    public static SpeedController driveLeftBottom;

    /**
     * The motor controller for the top right motor of the drive train. 
     */
    public static SpeedController driveRightTop;

    /**
     * The motor controller for the middle right motor of the drive train. 
     */
    public static SpeedController driveRightMiddle;

    /**
     * The motor controller for the bottom right motor of the drive train. 
     */
    public static SpeedController driveRightBottom;

    /**
     * The group of motor controllers for the left side of the drive train. 
     */
    public static SpeedControllerGroup driveLeft;

    /**
     * The group of motor controllers for the right side of the drive train.
     */
    public static SpeedControllerGroup driveRight;

    /**
     * The drive train subsystem. 
     */
    public static DriveTrain driveTrain;

    /**
     * The drive base. 
     */
    public static DifferentialDrive drive;

    /**
     * The speed controller for the arm motor. 
     */
    public static SpeedController armMotor;

    /**
     * The speed controller for the climber arm motor. 
     */
    public static SpeedController climberArmMotor;

    /**
     * The speed controller for the climber arm motor. 
     */
    public static SpeedController climberArmMotor1;

    /**
     * The speed controller for the climber wheel motor. 
     */
    public static SpeedController climberWheelMotor;

    /**
     * The speed controller for the climber leg motor (not in use). 
     */
    public static SpeedController climberLegMotor;

    /**
     * The speed controller for the wrist motor. 
     */
    public static SpeedController wristMotor;

    /**
     * The speed controller for the top intake motor. 
     */
    public static SpeedController intakeTopMotor;

    /**
     * The speed controller for the bottom intake motor. 
     */
    public static SpeedController intakeBottomMotor;

    /**
     * The arm subsystem. 
     */
    public static Arm arm;

    /**
     * The climber arm subsystem. 
     */
    public static ClimberArm climberArm;

    /**
     * The climber leg subsystem (not in use). 
     */
    public static ClimberLeg climberLeg;

    /**
     * The climber wheel subsystem. 
     */
    public static ClimberWheel climberWheel;

    /**
     * The wrist subsystem. 
     */
    public static Wrist wrist;

    /**
     * The intake subsystem. 
     */
    public static Intake intake;

    /**
     * The solenoid for the piston to change drive train gear ratio. 
     */
    public static Solenoid solenoidGearShifter;

    /**
     * The solenoid for the piston for the hatch collector. 
     */
    public static Solenoid solenoidHatchShifter;

    /**
     * For the safety light (connected to PCM). 
     */
    public static Solenoid solenoidLight;

    /**
     * The solenoid for the pistons for the climber. 
     */
    public static Solenoid solenoidClimb;

    //the dreaded potentiometers

    /**
     * The potentiometer for the wrist. 
     */
    public static AnalogPotentiometer wristPot;

    /**
     * The potentiometer for the arm. 
     */
    public static AnalogPotentiometer armPot;

    /**
     * The ultrasonic sensor for delivering hatches/cargo. 
     */
    public static Ultrasonic ultra;
    
    /**
     * The encoder for the climber. 
     */
    public static Encoder climb_enc;

    /**
     * The CameraServer instance. 
     */
    public static CameraServer camServer;

    /**
     * The USB Camera for the driver. 
     */
    public static UsbCamera cam1;

    /**
     * The USB Camera for the driver. 
     */
    public static UsbCamera cam2;

    private static Encoder encoder;

    /**
     * Method run at the beginning of the robot program. All subsystem, camera, and sensor initialization should go here. 
     * 
     * @see edu.wpi.first.wpilibj.TimedRobot.robotInit()
     */
    @Override
    public void robotInit()
    {
        //initialization of all subsystems
        driveLeftTop = new WPI_VictorSPX(Config.LEFT_TOP_MOTOR);
        driveLeftBottom = new WPI_VictorSPX(Config.LEFT_BOTTOM_MOTOR);
        driveRightMiddle = new WPI_VictorSPX(Config.RIGHT_MIDDLE_MOTOR);
        driveRightBottom = new WPI_VictorSPX(Config.RIGHT_BOTTOM_MOTOR);
        
        driveLeft = new SpeedControllerGroup(driveLeftTop,driveLeftBottom);
        driveRight = new SpeedControllerGroup(driveRightMiddle,driveRightBottom);

        drive = new DifferentialDrive(driveLeft, driveRight);
        driveTrain = new DriveTrain(driveLeft, driveRight, drive);

        climberArmMotor = new WPI_VictorSPX(Config.CLIMBER_ARM_MOTOR_LEFT);
        climberArmMotor1 = new WPI_VictorSPX(Config.CLIMBER_ARM_MOTOR_RIGHT);
        climberArm = new ClimberArm();

        climberWheelMotor = new WPI_VictorSPX(Config.CLIMBER_WHEEL_MOTOR);
        climberWheel = new ClimberWheel();

        climberLegMotor = new WPI_VictorSPX(Config.CLIMBER_LEG_MOTOR);
        climberLeg = new ClimberLeg();

        armMotor = new WPI_VictorSPX(Config.ARM_MOTOR);
        arm = new Arm(armMotor);

        wristMotor = new WPI_VictorSPX(Config.WRIST_MOTOR);
        wrist = new Wrist(wristMotor);

        intakeTopMotor = new WPI_VictorSPX(Config.INTAKE_TOP_MOTOR);
        intakeBottomMotor = new WPI_VictorSPX(Config.INTAKE_BOTTOM_MOTOR);
        intake = new Intake(intakeTopMotor, intakeBottomMotor);

        //ultrasonic initialization
        ultra = new Ultrasonic(Config.ULTRA_PORT2, Config.ULTRA_PORT1);
        ultra.setAutomaticMode(true);
      
        //solenoid initialization
        // solenoidGearShifter = new Solenoid(Config.PCM_ID, Config.SOLENOID_ONE_CHANNEL);
        // solenoidLight = new Solenoid(Config.PCM_ID, Config.SOLENOID_LIGHT_CHANNEL);
        // solenoidHatchShifter = new Solenoid(Config.PCM_ID, Config.SOLENOID_HATCH_CHANNEL);
        // solenoidClimb = new Solenoid(Config.PCM_ID, Config.SOLENOID_CLIMB_CHANNEL);
        // solenoidLight.set(true);

        //sensor initialization
        climb_enc = new Encoder(Config.ENCODER_CLIMB_PORT_1, Config.ENCODER_CLIMB_PORT_2, false, EncodingType.k4X);
        wristPot = new AnalogPotentiometer(Config.ROTATION_POTENTIOMETER_PORT_WRIST, 121);
        armPot = new AnalogPotentiometer(Config.ROTATION_POTENTIOMETER_PORT_ARM, 118);

        //camera setup
        camServer = CameraServer.getInstance();
        cam1 = camServer.startAutomaticCapture(0);
        cam1.setBrightness (35);
        cam1.setFPS(30);
        cam1.setResolution(100, 100);

        cam2 = camServer.startAutomaticCapture(1);
        cam2.setBrightness(40);
        cam2.setFPS(30);
        cam2.setResolution(100, 100);

        //dreaded shuffleboard preset buttons
        Shuffleboard.getTab("SmartDashboard").add("Hatch LV1", new HatchLevel1()).withWidget(BuiltInWidgets.kCommand);
        Shuffleboard.getTab("SmartDashboard").add("Ship Hatch LV1", new HatchLevel1()).withWidget(BuiltInWidgets.kCommand);
        
        Shuffleboard.getTab("SmartDashboard").add("Hatch LV2", new HatchLevel2()).withWidget(BuiltInWidgets.kCommand);
        Shuffleboard.getTab("SmartDashboard").add("Hatch LV3", new HatchLevel3()).withWidget(BuiltInWidgets.kCommand);

        Shuffleboard.getTab("SmartDashboard").add("Cargo LV1", new CargoLevel1()).withWidget(BuiltInWidgets.kCommand);
        Shuffleboard.getTab("SmartDashboard").add("Cargo LV2", new CargoLevel2()).withWidget(BuiltInWidgets.kCommand);
        Shuffleboard.getTab("SmartDashboard").add("Cargo LV3", new CargoLevel3()).withWidget(BuiltInWidgets.kCommand);

        Shuffleboard.getTab("SmartDashboard").add("Cargo", new IntakeCargoLevel1()).withWidget(BuiltInWidgets.kCommand);
        Shuffleboard.getTab("SmartDashboard").add("Cargo Keep", new IntakeCargoLevel1Keep()).withWidget(BuiltInWidgets.kCommand);
        Shuffleboard.getTab("SmartDashboard").add("Intake Cargo", new IntakeCargoLoadingStation()).withWidget(BuiltInWidgets.kCommand);
        
        Shuffleboard.getTab("SmartDashboard").add("Intake Hatch", new IntakeHatchLoadingStation()).withWidget(BuiltInWidgets.kCommand);
        Shuffleboard.getTab("SmartDashboard").add("Hatch", new IntakeHatchFloor()).withWidget(BuiltInWidgets.kCommand);

        Shuffleboard.getTab("SmartDashboard").add("Travel", new Travel()).withWidget(BuiltInWidgets.kCommand);
        Shuffleboard.getTab("SmartDashboard").add("Cargo Ship Cargo", new CargoShipCargo()).withWidget(BuiltInWidgets.kCommand);
        
        Shuffleboard.getTab("SmartDashboard").add("Climber Leg Level 2", new MoveClimberLegAuto(100)).withWidget(BuiltInWidgets.kCommand);
        Shuffleboard.getTab("SmartDashboard").add("Climber Leg Level 3", new MoveClimberLegAuto(200)).withWidget(BuiltInWidgets.kCommand);

        Shuffleboard.getTab("SmartDashboard").add("Load Hatch", new IntakeHatchLoadingStation()).withWidget(BuiltInWidgets.kCommand);
        Shuffleboard.getTab("SmartDashboard").add("Intake Cargo LS", new IntakeCargoLoadingStation()).withWidget(BuiltInWidgets.kCommand);
        Shuffleboard.getTab("SmartDashboard").add("Cargo Cargo", new CargoShipCargo()).withWidget(BuiltInWidgets.kCommand);
        Shuffleboard.getTab("SmartDashboard").add("Cargo Floor", new CargoFloor()).withWidget(BuiltInWidgets.kCommand);

        encoder = new Encoder(0, 1);

        //this should be initialized last in robotInit()
        oi = new OI();
    }

    /**
     * This is run periodically during the robot program. Periodic code and sensor value displays should go here.
     * 
     * @see edu.wpi.first.wpilibj.TimedRobot.robotPeriodic()
     */
    @Override
    public void robotPeriodic()
    {
        //needs to be here (scheduler handles commands during robot program). 
        Scheduler.getInstance().run();

        //shuffleboard sensor value displays
        SmartDashboard.putNumber("climb encoder", climb_enc.get());
        SmartDashboard.putNumber("arm pot wpi angle", armPot.get());
        SmartDashboard.putNumber("wrist pot wpi angle", wristPot.get());

        SmartDashboard.putNumber("Ultrasonic", ultra.getRangeInches());
        SmartDashboard.putBoolean("LowLowGear", DriveTrain.lowlowgear);
        SmartDashboard.putBoolean("Distance Hatch", ultra.getRangeInches() >= 11 && ultra.getRangeInches() <= 13);
        SmartDashboard.putBoolean("Distance Cargo Loading", ultra.getRangeInches() >= 19 && ultra.getRangeInches() <= 21);
        SmartDashboard.putBoolean("Distance Ship Shoot", ultra.getRangeInches() >= 23 && ultra.getRangeInches() <= 25);
        SmartDashboard.putNumber("Encoder distance", encoder.getDistance());
    }

    /**
     * This is run at the beginning of the autonomous mode of the robot program. Initialization code
     * specific to the autonomous phase should go here. 
     * 
     * @see edu.wpi.first.wpilibj.TimedRobot.autonomousInit()
     */
    @Override
    public void autonomousInit()
    {

    }

    /**
     * This is run periodically during the autonomous phase of the robot program. Periodic code 
     * specific to the autonomous phase should go here. 
     * 
     * @see edu.wpi.first.wpilibj.TimedRobot.autonomousPeriodic()
     */
    @Override
    public void autonomousPeriodic()
    {
        Scheduler.getInstance().run();
    }

    /**
     * This is run at the beginning of the teleoperated phase of the robot program. Initialization specific to the 
     * teleoperated phase should go here.
     * 
     * @see edu.wpi.first.wpilibj.TimedRobot.teleopInit()
     */
    @Override
    public void teleopInit()
    {

    }

    /**
     * This is run periodically during the teleoperated phase of the robot program. Periodic code specific to the
     * teleoperated phase should go here. 
     * 
     * @see edu.wpi.first.wpilibj.TimedRobot.teleopPeriodic()
     */
    @Override
    public void teleopPeriodic()
    {
        Scheduler.getInstance().run();
    }
}
