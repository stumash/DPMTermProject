package termproject;

/**
 * This class contains the main method and all the high-level gameplay logic
 * @author Stuart Mashaal and Mathieu Savoie
 *
 */
public class Main {
	private enum State {INIT, LOCALIZE, GOTOSTART,
		GETBALL, GOTONET, PREPARESHOT, FIRE, 
		DEFEND, TRAVELLING, OBSTACLEAVOIDANCE};
	private static State state = State.INIT; //the current behavior state
	private static State prevState; //the previous behavior state
	
		
	public static void main(String[] args) {
		
		//the main run-loop
		while (true) {
		switch (state) {
		case INIT:
			break;
		case LOCALIZE:
			break;
		case GOTOSTART:
			break;
		case TRAVELLING:
			break;
		case DEFEND:
			break;
		case GETBALL:
			break;
		case GOTONET:
			break;
		case PREPARESHOT:
			break;	
		case FIRE:
			break;
		case OBSTACLEAVOIDANCE:
			break;		
		}
		}
	}
	
	//sets prevState to state and then sets state to s
	private static void setState(State s) {
		prevState = state;
		state = s;
	}
}
