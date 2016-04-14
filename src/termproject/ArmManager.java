package termproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class ArmManager {
	private EV3LargeRegulatedMotor motor;

	public ArmManager(EV3LargeRegulatedMotor armmotor) {
		this.motor = armmotor;
	}
	
	/**
	 * collect the ball, assuming the arm is aligned with a ball in the ball holder
	 */
	public void collectBall() {
		motor.setSpeed(Constants.GETBALL_ARMSPEED);
		motor.setAcceleration(Constants.GETBALL_ARMACCELERATION);
		
		//
		motor.rotate(Constants.GETBALL_TAPANGLE);
		
		//push the ball into the holder and bend the "elbow" so that the arm is ready to fire
		motor.setSpeed(Constants.GETBALL_PUSHSPEED);
		motor.rotate(Constants.GETBALL_HOLDERPUSHANGLE);
		
		//prepare for slap
		motor.rotate(Constants.FIRE_WINDUPANLGE);
	}
	
	/**
	 * fire the ball by slapping at at high speed
	 */
	public void fire() {
		motor.setSpeed(Constants.FIRE_ARMSPEED);
		motor.setAcceleration(Constants.FIRE_ARMACCELERATION);
		
		motor.rotate(Constants.FIRE_SLAPSHOTANGLE);
		
		//reset the arm to ball-collection position
		reset();
	}
	
	private void reset() {
		motor.setSpeed(Constants.ARMRESET_SPEED);
		motor.setAcceleration(Constants.FIRE_ARMACCELERATION);
		
		//reset the arm to collect another ball by rotate backwards by the sum of all angle rotated so far
		motor.rotate(-(Constants.GETBALL_TAPANGLE + Constants.GETBALL_HOLDERPUSHANGLE + 
				Constants.FIRE_WINDUPANLGE + Constants.FIRE_SLAPSHOTANGLE));
	}
	
	
}
