package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A wrappper for the WPILIB joystick class so that butoons can be accessed by name rather than
 * their ID;
 */
public class CommandNXT extends CommandJoystick {

  public static final int FIRE_STAGE1 = 1;
  public static final int FIRE_STAGE2 = 2;
  public static final int A2 = 3;
  public static final int B1 = 4;
  public static final int D1 = 5;
  public static final int A3_UP = 6;
  public static final int A3_RIGHT = 7;
  public static final int A3_DOWN = 8;
  public static final int A3_LEFT = 9;
  public static final int A3_IN = 10;
  public static final int A4_UP = 11;
  public static final int A4_RIGHT = 12;
  public static final int A4_DOWN = 13;
  public static final int A4_LEFT = 14;
  public static final int A4_IN = 15;
  public static final int C1_UP = 16;
  public static final int C1_RIGHT = 17;
  public static final int C1_DOWN = 18;
  public static final int C1_LEFT = 19;
  public static final int C1_IN = 20;
  public static final int FIRE_PADDLE_UP = 21;
  public static final int FIRE_PADDLE_DOWN = 22;
  public static final int EN1_UP = 23;
  public static final int EN1_DOWN = 24;
  public static final int SW1_UP = 25;
  public static final int SW1_DOWN = 26;
  public static final int SW2_ID = 2;
  public static final int Stick_X = 0;
  public static final int Stick_Y = 1;
  public static final int A1_X = 3;
  public static final int A1_Y = 4;
  public static final int Stick_Z = 5;

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public CommandNXT(int port) {
    super(port);
  }

  /**
   * Constructs an event instance around a button's digital signal.
   *
   * @return a new Trigger that is true when the first stage is depressed but not when the second is
   */
  public Trigger fireStage1() {
    return this.button(FIRE_STAGE1);
  }

  /**
   * Constructs an event instance around a button's digital signal.
   *
   * @return a new Trigger that is true when the second stage is depressed
   */
  public Trigger fireStage2() {
    return this.button(FIRE_STAGE2);
  }

  /** second paddle pushed up */
  public Trigger firePaddleUp() {
    return this.button(FIRE_PADDLE_UP);
  }

  // second paddle pushed down
  public Trigger firePaddleDown() {
    return this.button(FIRE_PADDLE_DOWN);
  }

  // red button
  public Trigger a2() {
    return this.button(A2);
  }

  // button on top and back of controller
  public Trigger b1() {
    return this.button(B1);
  }

  // button on the bottom and back of controller
  public Trigger d1() {
    return this.button(D1);
  }

  // middle joystick up
  public Trigger a3Up() {
    return this.button(A3_UP);
  }

  // middle joystick right
  public Trigger a3Right() {
    return this.button(A3_RIGHT);
  }

  // middle joystick down
  public Trigger a3Down() {
    return this.button(A3_DOWN);
  }

  // middle joystick left
  public Trigger a3Left() {
    return this.button(A3_LEFT);
  }

  // middle joystick in
  public Trigger a3In() {
    return this.button(A3_IN);
  }

  // top right joystick up
  public Trigger a4Up() {
    return this.button(A4_UP);
  }

  // top right joystick in
  public Trigger a4Right() {
    return this.button(A4_RIGHT);
  }

  // top right joystick down
  public Trigger a4Down() {
    return this.button(A4_DOWN);
  }

  // top rigjt joystick left
  public Trigger a4Left() {
    return this.button(A4_LEFT);
  }

  // top right joystick in
  public Trigger a4In() {
    return this.button(A4_IN);
  }

  // left gray stick up
  public Trigger c1Up() {
    return this.button(C1_UP);
  }

  // left gray stick right
  public Trigger c1Right() {
    return this.button(C1_RIGHT);
  }

  // left gray stick down
  public Trigger c1Down() {
    return this.button(C1_DOWN);
  }

  // left gray stick left
  public Trigger c1Left() {
    return this.button(C1_LEFT);
  }

  // left gray stick in
  public Trigger c1In() {
    return this.button(C1_IN);
  }

  // bottom right wheel up
  public Trigger en1Up() {
    return this.button(EN1_UP);
  }

  // bottom right wheel down
  public Trigger en1Down() {
    return this.button(EN1_DOWN);
  }

  // bottom left wheel up
  public Trigger sw1Up() {
    return this.button(SW1_UP);
  }

  // bottom left wheel down
  public Trigger sw1Down() {
    return this.button(SW1_DOWN);
  }

  // bottom middle wheel
  public double SW2() {
    return this.getRawAxis(SW2_ID);
  }

  // main stick forward and backward
  public double StickY() {
    return this.getRawAxis(Stick_Y);
  }

  // main stick left and right
  public double StickX() {
    return this.getRawAxis(Stick_X);
  }

  // main stick rotation
  public double StickZ() {
    return this.getRawAxis(Stick_Z);
  }

  // top left stick left and right
  public double A1X() {
    return this.getRawAxis(A1_X);
  }

  // top left stick up and down
  public double A1Y() {
    return this.getRawAxis(A1_Y);
  }
}
