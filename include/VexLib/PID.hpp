#ifndef PID_C_HPP
#define PID_C_HPP

namespace VexLib{

struct PIDFCoefficients {
	double P, I, D, F;
};

class PIDFController {
	PIDFCoefficients coeff;
	double Isum = 0;
	double minSO, maxSO;
	double frequency = 50;
	double lastTime = -1;
	double lastError = 0;
	double lastReturn = 0;

public:
	explicit PIDFController(PIDFCoefficients = {0, 0, 0, 0});
	
	/**
	 * @brief sets the controller to run at a specific frequency 
	 * */
	void setControllerFrequency(double);
	/** @brief sets the maximum value the system can take to change its state
	 * @param min the minimum value
	 * @param max the maximum value
	 * */
	void setMaxSystemOutput(double min, double max);
	
	/** @brief resets the controller to restart the estimating proccess (mostly used when changeing trajectories)
	 *
	 * */
	void resetTracking();

	/**
	 * @brief makes a PIDF loop
	 * @param error the error of the system
	 * @return the power that the system should respond with
	 * */
	double getPowerOutput(double error);
	/** 
	 * @brief makes a PIDF loop without calculatind the derivative, assuming it is the velocity of the system
	 * @param error the error of the system
	 * @param velocity the velocity of the system
	 * @return the power that the system should respond with
	 * */
	double getPowerOutputWithVelo(double error, double velocity);
	/** 
	 * @breif sets the pidf values of the controller
	 * */
	void setPIDFCoefficinet(PIDFCoefficients);

	/**
	 * @breif returns a refference to the current PIDF coefficients
	 * */
	PIDFCoefficients& getPIDFCoefficients();
};


}; // namepsace VexLib

#endif
