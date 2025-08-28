#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

//drivetrain motor group
pros::MotorGroup right_motors({3, -4, 1}, pros::MotorGearset::blue);
pros::MotorGroup left_motors({-5, 11, -6}, pros::MotorGearset::blue);

// pros::MotorGroup intake({13, -3},
//                             pros::MotorGearset::blue);
// intertial
pros::Imu imu(15);

//odometry
//vertical tracking wheel encoder
pros::Rotation vertical_encoder(-5); 
// vertical tracking wheel., front of the bot touched with offset of 4
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 1.375); //0.75, 2 came forward but clamp right of y,
//horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(7); 
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -5.5);

//intake motors
pros::Motor intake1(20, pros::MotorGearset::green);
pros::Motor intake2(9, pros::MotorGearset::green);
pros::Motor intake3(19, pros::MotorGearset::green);
pros::Motor intake4(13, pros::MotorGearset::green);

//pnuematics
pros::adi::DigitalOut flap1('A', false);
pros::adi::DigitalOut flap2('B', false);
pros::adi::DigitalOut scraper1('C',false);
pros::adi::DigitalOut scraper2('D',false);
pros::adi::DigitalOut ramp1('E', true);
pros::adi::DigitalOut ramp2('F', true);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            9.5, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(3.45, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             15, // derivative gain (kD)
                                             3, // 3 anti windup old = 3
                                             1, // 1 small error range, in degrees old = 1
                                             100, // 100 small error range timeout, in milliseconds old = 100
                                             3, // 3 large error range, in degrees old = 3
                                             500, // 500 large error range timeout, in milliseconds old = 500
                                             0 // 0 maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        linearController, // lateral PID settings
                        angularController, // angular PID settings
                        sensors // odometry sensors
);



/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize(); // initialize brain screen
	// pros::lcd::set_text(1, "Hello PROS User!");
	// pros::lcd::register_btn1_cb(on_center_button);
    chassis.calibrate(); // calibrate sensors

	// print position to brain screen
    pros::Task screen_task([&]() {
	while (true) {
		// print robot location to the brain screen
		pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
		pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
		pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
		
		// print measurements from the rotation sensor
        pros::lcd::print(3, "Rotation Sensor: %i", vertical_encoder.get_position());
		pros::lcd::print(4, "Rotation Sensor: %i", horizontal_encoder.get_position());

		//print inertial
		//pros::lcd::print(5, "IMU get heading: %f degrees\n", imu.get_heading());
		pros::lcd::print(5, "IMU: %f", imu.get_heading());
		
		// delay to save resources
		pros::delay(50);
	}
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

  void intake_Top_Move(){
 	//intake
	intake3.move_velocity(-200);
    intake1.move_velocity(-200);
    intake2.move_velocity(200);
    intake4.move_velocity(-200);
 	// seconds = seconds*1000;
 	// pros::delay(seconds); // 1000 miliseconds is 1 second
    // intake3.move_velocity(0);
    // intake1.move_velocity(0);
    // intake2.move_velocity(0);
    // intake4.move_velocity(0);
 }

   void intake_stop(){
 	
    intake3.move_velocity(0);
    intake1.move_velocity(0);
    intake2.move_velocity(0);
    intake4.move_velocity(0);
 }

void autonomous() {
    chassis.setPose(0, 0, 0, 0);
    chassis.turnToHeading(-90, 1000);
    chassis.moveToPose(21.5, 0, -90, 5000);
    chassis.turnToHeading(180, 500);
    chassis.moveToPose(21.5,-4, 180, 500);
    chassis.turnToHeading(0, 500);
    chassis.moveToPose(21.5,4,0,500);
    //intake_Top_Move();
    //pros::delay(1000);
    chassis.turnToHeading(180, 500);
    chassis.moveToPose(21.5, 0, 180, 500);
   
    chassis.moveToPose(2, 0, 180, 500);
    chassis.turnToHeading(90, 500);
    chassis.moveToPose(2, 24, 90, 1000);
    chassis.moveToPose(2, 20,90, 200);
    chassis.turnToHeading(90,300);
    chassis.turnToHeading(135,50);
    chassis.moveToPose(1, 30, 135, 2000);
    chassis.turnToHeading(180, 200);
    chassis.moveToPose(-60, 0, float theta, int timeout)


    
   







    
    






}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller controller(pros::E_CONTROLLER_MASTER);

	while (true) {
        //pros::task_t(NULL);
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

		//intake

		//top goal
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake3.move_velocity(-200);
            intake1.move_velocity(-200);
            intake2.move_velocity(200);
            intake4.move_velocity(-200);
		}
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake3.move_velocity(200);
            intake1.move_velocity(200);
            intake2.move_velocity(-200);
            intake4.move_velocity(200);
        }

		//middle goal
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake3.move_velocity(-200);
            intake1.move_velocity(200);
            intake2.move_velocity(-200);
            intake4.move_velocity(-200);
		}
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            intake3.move_velocity(200);
            intake1.move_velocity(-200);
            intake2.move_velocity(200);
            intake4.move_velocity(200);
        }
		else{
			intake3.move_velocity(0);
            intake1.move_velocity(0);
            intake2.move_velocity(0);
            intake4.move_velocity(0);
		}

	
        
		//scraper
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			scraper1.set_value(true);
			scraper2.set_value(true);
		}
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			scraper1.set_value(false);
			scraper2.set_value(false);
		
		}

		//flap
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			flap1.set_value(true);
			flap2.set_value(true);
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			flap1.set_value(false);
			flap2.set_value(false);
        }	
		
		//delay
		pros::delay(25); 
	}//end of while	

}