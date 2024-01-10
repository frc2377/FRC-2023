// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// robot container

package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PneumaticConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.*;



/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /*CANdle*/
  //private final CANdle m_CANdle = new CANdle(Constants.LEDs.kCandelChannel);

  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController manipulator = new CommandXboxController(1);
  /* LEDs */
  private final LEDSubsystem m_led = new LEDSubsystem();
  // mode  varable
  private final DeployMode m_mode = new DeployMode(true,false); // set to cone at start up, set preplace to false on start up
  // interrupt automated drive management
  // mode subsystem
  private final ModeSubsystem m_ModeSubsytem = new ModeSubsystem(m_mode);
  // Rainbow varable
  private final RainbowMode m_rainbow = new RainbowMode(false);
  // Rainbow Subsystem
  private final RainbowSubsytem m_RainbowSubsytem = new RainbowSubsytem(m_rainbow);
  // set up autos for smart dashboard
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  // SwerveDrive
  private static final SwerveSubsystem m_swerve = new SwerveSubsystem();
  // Instantiate the Pneumatics Hub
  private final PneumaticHub m_ph = new PneumaticHub(PneumaticConstants.kPHID);  
  // RobotArm
  private final ArmSubsystem3 m_arm3 = new ArmSubsystem3(m_mode);
  //Claw & Wrist
  private final ClawSubsystem m_claw = new ClawSubsystem(m_ph);
  //Extender
  private final ExtenderSubsystem m_extender = new ExtenderSubsystem();
  //Intake
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem(m_ph);
 
 

  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
   
     // init default commands
    this.initializeDefaultCommands(); 
    //new DefaultIntakeCommand(m_IntakeSubsystem);
    // Configure the button bindings
    this.configureButtonBindings();
    /* Initialize various systems on robotInit. */
    this.initializeStartup();
    /* Initialize autonomous command chooser and display on the SmartDashboard. */
    this.initializeAutoChooser();  

    DriverStation.silenceJoystickConnectionWarning(true);
    
  }

  /**
   * Set default command for subsystems. Default commands are commands that run
   * automatically whenever a subsystem is not being used by another command. If
   * default command is set to null, there will be no default command for the
   * subsystem.
   */
  private  void initializeDefaultCommands()
  {
      m_led.setDefaultCommand(new LEDDefaultCommand(m_led, m_ModeSubsytem, m_RainbowSubsytem)); 
      
      m_swerve.setDefaultCommand(
      new TeleopSwerve(
          m_swerve,
          () -> -1 * driver.getLeftY(),
          () -> -1 * driver.getLeftX(),
          () -> -0.45 * driver.getRightX(),
          () -> driver.leftBumper().getAsBoolean()
          ));
  }
  
  /**
   * Various methods to run when robot is initialized. Cannot put these in
   * robotInit() in Robot.java becaus       1Qe sub systems may not be instantiated at that
   * point.
   */
  private void initializeStartup()
  {
    /* Turn off Limelight LED when first started up so it doesn't blind drive team. */
    //LimelightUtil.turnOffLED();

    // Reset Gyro (Yaw only)
    m_swerve.zeroGyro();
    m_ph.enableCompressorAnalog(PneumaticConstants.kMinPressure,PneumaticConstants.kMaxPressure);
    
   
  }
  
  /**
   * Set options for autonomous command chooser and display them for selection on
   * the SmartDashboard. Using string chooser rather than command chooser because
   * if using a command chooser, will instantiate all the autonomous commands.
   * This may cause problems (e.g. initial trajectory position is from a different
   * command's path).
   */
  private void initializeAutoChooser()
  {
    /* Add options (which autonomous commands can be selected) to chooser. */
    // Load autonomous commands dashboard.
    m_chooser.setDefaultOption("AutoDeploy_with_balance", new DeployHighConeThenBalance(m_swerve,m_arm3,m_extender,m_claw,m_ModeSubsytem,m_IntakeSubsystem)); // our primary auto middle route  
   // m_chooser.addOption("AutoDeploy_no_balance", new DeployHighConeThenMobility(m_swerve,m_arm3,m_extender,m_claw,m_mode)); // our secondary auto
    m_chooser.addOption("Double cube Blue", new DeployTwoCubesBlue(m_swerve, m_IntakeSubsystem,m_arm3, m_extender,m_claw,m_ModeSubsytem));
    m_chooser.addOption("Double cube Red", new DeployTwoCubesRed(m_swerve, m_IntakeSubsystem,m_arm3, m_extender,m_claw,m_ModeSubsytem));
    m_chooser.addOption("ConeCubeBlue", new AutoConeCubeBlue(m_swerve, m_arm3, m_extender, m_claw, m_ModeSubsytem, m_IntakeSubsystem));
    m_chooser.addOption("ConeCubeRed", new AutoConeCubeRed(m_swerve, m_arm3, m_extender, m_claw, m_ModeSubsytem, m_IntakeSubsystem));
    //m_chooser.addOption("TripleCubeAuto", new TripleCubeAuto(m_swerve, m_IntakeSubsystem, m_arm3, m_claw));

    /*
     * Display chooser on SmartDashboard for operators to select which autonomous
     * command to run during the auto period.
     */
    SmartDashboard.putData("Autonomous Commands", m_chooser);
   // SmartDashboard.putNumber("pressure",m_ph.getPressure(0));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  
    ////*******----------DRIVER CONTROLLER -----------------**********
  
    driver.x().whileTrue(new RotateLeft(m_swerve));
    driver.b().whileTrue(new RotateRight(m_swerve));
    driver.y().onTrue(new DeployCubeFar(m_IntakeSubsystem));
    driver.start().onTrue(Commands.runOnce(() -> m_swerve.zeroGyro(), m_swerve)); // reset X/Y only use at start of match
    driver.a().whileTrue(new SquareUpAutoAlign(m_swerve)); // automates the auto alignment with the grid

    // left trigger open intake  
    driver.leftTrigger().onTrue((new DeployIntake2(m_IntakeSubsystem)));
      
    // right bumper close intake
    driver.rightTrigger().onTrue((new RetractIntake(m_IntakeSubsystem)));

    // right bumper drive quarter speed while held
    driver.rightBumper().whileTrue(new QuarterSpeed(
      m_swerve,
      () -> -.55 * driver.getLeftY(),
      () -> -.55 * driver.getLeftX(),
      () -> -0.35 * driver.getRightX(),
      () -> driver.leftBumper().getAsBoolean()
      ));

    // add dpad
    driver.povLeft().whileTrue((new PovMoveLeft(m_swerve)));
    driver.povRight().whileTrue((new PovMoveRight(m_swerve)));
    driver.povDown().whileTrue((new PovMoveBackward(m_swerve)));
    driver.povUp().whileTrue((new PovMoveForward(m_swerve)));
    
  

    ////*******----------MANIPULATOR CONTROLLER -----------------**********
    ////A---------------hybrid deployment sequence----------------------A
     manipulator.a().onTrue(new HybridDeployment(m_arm3, m_claw));
     
     ////X---------------middle deployment sequence using mode----------------------X
   
     manipulator.x().onTrue(new ConditionalCommand(
     new PlaceCargoMidPositionNoMovement(m_swerve, m_arm3, m_extender, m_claw), 
     new PlaceCargoMidPosition(m_swerve, m_arm3, m_extender, m_claw, m_ModeSubsytem), 
     () -> m_mode.getprePlace()));
     
     ////Y---------------placing cargo in top position using mode ----------------------Y
    
     manipulator.y().onTrue(new ConditionalCommand(
      new PlaceCargoTopPositionNoMovement(m_swerve, m_arm3, m_extender, m_claw), 
      new PlaceCargoTopPosition(m_swerve, m_arm3, m_extender, m_claw, m_ModeSubsytem), 
      () -> m_mode.getprePlace()));
 


    ////--------------picking up from Portal----------------------
    manipulator.b().onTrue(new PickupFromPortal(m_arm3, m_claw, m_extender));
      
    ////LTrigger-----------Rainbow Button----------------------------LTrigger
    manipulator.leftTrigger().onTrue(m_RainbowSubsytem.runOnce(() -> m_RainbowSubsytem.toggleRainbow()));
    ////LEFT BUMPER---------------CHANGE ROBOT MODE----------------LEFT BUMPER
    manipulator.leftBumper().onTrue(m_ModeSubsytem.runOnce(() -> m_ModeSubsytem.toggleMode())); //toggle the mode
    manipulator.povDown().onTrue((new ShootFromIntakeLowTelop(m_IntakeSubsystem)));
    manipulator.povLeft().onTrue((new ShootFromIntakeMidTelop(m_IntakeSubsystem)));
    manipulator.povUp().onTrue((new ShootFromIntakeHighTelop(m_IntakeSubsystem)));

    // right bumper move to place setting
    manipulator.rightBumper().onTrue(m_arm3.runOnce(() -> m_arm3.setPositionDegrees(ShoulderConstants.kShoulderConeTopPosition)).alongWith(m_ModeSubsytem.runOnce(() -> m_ModeSubsytem.setPrePlaceTrue())));

    //right trigger move to stow setting
    manipulator.rightTrigger().onTrue(m_arm3.runOnce(() -> m_arm3.setPositionDegrees(ShoulderConstants.kShoulderStowPosition)).alongWith(m_ModeSubsytem.runOnce(() -> m_ModeSubsytem.setPrePlaceFalse())));
  }
                                                                                                                               


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
