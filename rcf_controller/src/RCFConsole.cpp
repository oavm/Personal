
#include <rcf_controller/RCFController.h>
#include <dls_controller/support/ConsoleUtility.h>
#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>

using namespace iit;
using namespace HyQ2Max;


namespace dls_controller
{
	
bool RCFController::writeDebugVariablesOut(std::vector<double>& variables)
{
    variables[0] = bs->getRotationRate_B()[Roll];
    variables[1] = bs->getRotationRate_B()[Pitch];

	variables[2] = stanceLegs[dog::LF];
	variables[3] = stanceLegs[dog::RF];
	variables[4] = stanceLegs[dog::LH];
	variables[5] = stanceLegs[dog::RH];

	return false;
}

void RCFController::manPrint()
{
	std::cout << "Menu options:"<< std::endl;
	if(menuFunctions.size() == 0)
	{
		std::cout << "No prompts available!" << std::endl;
	}else{
		for(int i = 0; i < menuFunctions.size(); i++)
		{
			std::cout << menuFunctions[i].command << " - " << menuFunctions[i].comment << std::endl;
		}
	}
}

bool RCFController::consoleCallFunction(std::string func_name)
{
	for(int i = 0; i < menuFunctions.size(); i++)
	{
		if(func_name == menuFunctions[i].command)
		{
			//mtx.lock();//Make function execution thread safe
			(this->*menuFunctions[i].func_ptr)();
			//mtx.unlock();
		}
	}
	return true;
}

bool RCFController::addPromptCommands()
{
	menuFunctions.clear();
	addFunction("man", "Print menu options", &RCFController::manPrint);
	addFunction("ctp", "Print menu options", &RCFController::changeRCF_Parameters);
	addFunction("changeTaskParameters", "Print menu options", &RCFController::changeRCF_Parameters);
	//addFunction("resetOdom", "Reset RCF odometry", &RCFController::resetRCF_Odometry);
	addFunction("walk_adapt", "toggle walk adaptation", &RCFController::toggleWalkAdaptationFlag);
	addFunction("adaptiveWalkingTrot", "toggle walk adaptation", &RCFController::toggleWalkAdaptationFlag);
	addFunction("trunk_cont", "toggle trunk control", &RCFController::toggleTrunkControl);
	addFunction("trunkController", "toggle trunk control", &RCFController::toggleTrunkControl);
	addFunction("prec", "toggle push recovery mode", &RCFController::togglePushRecoveryFlag);
	addFunction("pushRecovery", "toggle push recovery mode", &RCFController::togglePushRecoveryFlag);
	addFunction("kadj", "toggle kinematic adjustment",	&RCFController::toggleKinematicAdjustmentFlag);
	addFunction("kinematicAdjustment", "toggle kinematic adjustment",  &RCFController::toggleKinematicAdjustmentFlag);
	addFunction("ictp", "interactively change task parameters", &RCFController::interactiveChangeTrotTask);
	addFunction("ictpDemos", "interactively change task parameters for demos", &RCFController::interactiveChangeTaskForDemos);
	addFunction("interactiveChangeTaskParameters", "interactively change task parameters", &RCFController::interactiveChangeTrotTask);
	addFunction("jctp", "joy change task parameters", &RCFController::joyChangeTrotTask);
	addFunction("jctpGuess", "joy change task parameters", &RCFController::joyChangeTrotTaskForVisitors);
	addFunction("cptt", "change gait pattern", &RCFController::changeGaitPattern);
	addFunction("changeGaitPattern", "change gait pattern", &RCFController::changeGaitPattern);
	addFunction("changeTrunkControllerGains", "change attitude control gains",
			&RCFController::changeTrunkControllerGains);
	addFunction("ccg", "change attitude control gains", &RCFController::changeTrunkControllerGains);
	addFunction("changeKinematicAdjustment", "Change kinematic dependency",
			&RCFController::changeKinematicDependency);
	addFunction("changeCOG", "Change Center of Gravity Offset",
			&RCFController::changeCOG);
	addFunction("changeCPGParams", "Change CPG convergence gains",
			&RCFController::changeCPG_Params);
	addFunction("changePushRecovery", "Change Push Recovery Parameters",
			&RCFController::changePrecParameters);
	addFunction("changeControllerOffsets", "Change Trunk Controller Offset",
			&RCFController::changeTrunkOffset);
	addFunction("cor", "Limit Cycles Origins", &RCFController::changeOrigins);
	addFunction("changeOrigins", "Limit Cycles Origins", &RCFController::changeOrigins);
	addFunction("ooo", "Walk stopping", &RCFController::walkStopping);
	addFunction("stopWalking", "Walk stopping", &RCFController::walkStopping);
	addFunction("stw", "Walk starting", &RCFController::walkStarting);
	addFunction("startWalking", "Walk starting", &RCFController::walkStarting);
	addFunction("wst", "Control status", &RCFController::controlStatus);
	addFunction("controlStatus", "Control status", &RCFController::controlStatus);
	addFunction("wman", "WCPG menu", &RCFController::menuOptions);
	addFunction("wor", "Limit Cycles Origins", &RCFController::whereOrigins);
	addFunction("whereOrigins", "Limit Cycles Origins", &RCFController::whereOrigins);
	addFunction("where_origins", "Limit Cycles Origins", &RCFController::whereOrigins);
	addFunction("narrowStance", "Narrow stance", &RCFController::narrowStance);

	addFunction("wideStance", "Wide stance", &RCFController::wideStance);
	addFunction("changePushUp", "Change push up parameters", &RCFController::changeSinParameters);
	addFunction("changeInternalForces", "Change push up parameters", &RCFController::changeSinParametersForce);
	addFunction("pushUp", "toggle push up task", &RCFController::togglePushUpTask);
	addFunction("internalForces", "toggle push up task", &RCFController::toggleInternalForcesTask);
	addFunction("changeChirpParameters", "Change chirp parameters", &RCFController::changeChirpParameters);
	addFunction("changeSquareWaveParameters", "Change square wave parameters", &RCFController::changeSquareWaveParameters);
	addFunction("changeTriangularWaveParameters", "Change triangular wave parameters", &RCFController::changeTriangularWaveParameters);
	addFunction("changeSineWaveParameters", "Change sine wave parameters", &RCFController::changeSineWaveParameters);

	addFunction("toggleChirp", "toggle chirp task", &RCFController::toggleChirpTask);
	addFunction("toggleSquareWave", "toggle chirp task", &RCFController::toggleSquareWaveTask);
	addFunction("toggleTriangularWave", "toggle triangular task", &RCFController::toggleTriangularWaveTask);
	addFunction("toggleSineWave", "toggle sine task", &RCFController::toggleSineWaveTask);

	addFunction("startRunningTrot", "Running trot starting", &RCFController::startRunningTrot);
	addFunction("changeStanceImpedance", "Change stance phase PID gains", &RCFController::changeLegImpedanceDuringStancePhase);
	addFunction("changePIDManager", "Change PID Manager params", &RCFController::changePID_Manager);
	addFunction("whereIMU", "Print current imu data", &RCFController::imuPrint);
	addFunction("where_foot_ft", "Print current foot ft sensor", &RCFController::footSensorPrint);
	addFunction("where_des", "Print current desired joint state", &RCFController::desJointStatePrint);
	addFunction("where_js", "Print current joint state", &RCFController::jointStatePrint);
	addFunction("jump", "Trigger Jump", &RCFController::toggleJump);
	addFunction("goSquat","Go to squat position", &RCFController::goSquat);
	addFunction("goBelly","Go to squat position", &RCFController::goBelly);
	addFunction("startSelfRighting", "Start self-righting", &RCFController::startSelfRighting);
	addFunction("startLayDown", "Start sit down", &RCFController::startLayDown);
	addFunction("resetOrigins", "Reset CPG origins", &RCFController::resetOriginsForTrotting);
	addFunction("setPID", "Set PID values", &RCFController::setPID);
	addFunction("toggleForcedStance", "toggle the forced stance", &RCFController::toggleForcedStance);
	addFunction("setup", "RCF general setup", &RCFController::RCF_Setup);
	addFunction("ter", "Toggle terrain estimator", &RCFController::toggleTerrainEstimationFlag);
	addFunction("terrainEstimator", "Toggle terrain estimator", &RCFController::toggleTerrainEstimationFlag);
	addFunction("resetRobotDesiredPosition", "Reset robot desired position in the WF", &RCFController::resetRobotDesiredPosition);
	addFunction("togglePositionControl", "Toggle robot position control", &RCFController::togglePositionControl);
	addFunction("changeTerrainAdjustments", "Change terrain adjustment parameters", &RCFController::changeTerrainAdjustments);
	addFunction("jointErrors", "Print current joint state errors", &RCFController::jointErrorsPrint);
	addFunction("whereTrunkGains", "Print trunk controller gains", &RCFController::showTrunkGains);
	addFunction("setImpedanceRoughTerrain", "Set default impedances for rough terrain", &RCFController::setImpedanceRoughTerrain);
	addFunction("resetImpedance", "Print trunk controller gains", &RCFController::resetImpedance);
	addFunction("joy", "Print trunk controller gains", &RCFController::toggleJoystick);




	std::vector<std::string> tempFuncNameVector;
	for(int i = 0; i < menuFunctions.size(); i++ )
	{
		tempFuncNameVector.push_back(menuFunctions[i].command);
	}
	consoleSetFunctionList(tempFuncNameVector);
}


bool RCFController::addFunction(const std::string command,const std::string comment,
	const RCFController::function_pointer funcPtr)
{
	console_command temp = {command,comment,funcPtr};
	menuFunctions.push_back(temp);

	return true;
}

///*********************************************************************************************///

// void RCFController::resetRCF_Odometry(void) {

// 	RCFOdometry.resetOdometry();

// 	std::cout << "RCF odometry reseted!!!" << std::endl;
// }

void RCFController::joyChangeTrotTask(void) {
	std::cout << "Joy CTP " << std::endl;
	std::cout << "Joystick X to exit" << std::endl;
	static Eigen::Matrix<double, 6, 1> axes_memory;
	static Eigen::Matrix<int, 12, 1> buttons_memory;
	static bool roughTerrainMode = false;
	static double VfX_buffer = 0.01;
	static double VfY_buffer = 0.0;
	static double Psit_buffer = 0.0;

/*    **********  Using wireless CSL gamepad for PS3 **********

    Axes channels     /     Commands
         0                Left Analog Left/Right
         1                Left Analog Up/Down
         2                Right Analog Left/Right
         3                Right Analog Up/Down
         4                Left  left and right cross buttons
         5                Left  up and down cross buttons
         6                Not used

    Button channels    /     Commands
          0                  Button 1
          1                  Button 2
          2                  Button 3
          3                  Button 4
          4                  Button L1
          5                  Button R1
          6                  Button L2
          7                  Button R2
          8                  Button SELECT
          9                  Button START
          10                 Button of left analog joystick
          11                 Button of right analog joystick
*/

	while(1)
	{
	    //Longitudinal velocity and turning rate
	    if(roughTerrainMode){
	        //Longitudinal velocity
	        VfX_buffer = 0.5 * joystick_axes(1);
	        //Angular velocity (robot turning)
	        Psit_buffer =  joystick_axes(2)* 45 * M_PI / 180 ;
	    }
	    else{
	        VfX_buffer = 0.75 * joystick_axes(1);
	        Psit_buffer =  joystick_axes(2)* 60 * M_PI / 180 ;
	    }

	    //Dead zone for VfX
	    if(VfX_buffer > -0.025 && VfX_buffer < 0.025)
	        VfX_buffer = 0.005;

	    //Lateral velocity
	    VfY_buffer = 0.5 * joystick_axes(0);

	    //Dead zone for VfY
	    if(VfY_buffer > -0.025 && VfY_buffer < 0.025)
	        VfY_buffer = 0.0;





	    //Update velocities when there is swing phase to avoid foot frontal
	    //collision and the CPG get out of phase when dutyF is greater than 0.5
	    if(!frontalCollisionFlag){
	        if(forwVel > 0.025 || WCPGb[dog::LF](rbd::Z) > 0.33*stepHeight || WCPGb[dog::RF](rbd::Z) > 0.33*stepHeight){
	            VfX_New = VfX_buffer;
	            VfY_New = VfY_buffer;
	            PsitNew = Psit_buffer;

	            forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
	            stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
	        }
	    } else{
            VfX_New = -0.01;
            VfY_New = 0.0;
            PsitNew = 0.0;

            forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
            stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
	    }

	    //Trunk controller
	    if(joystick_buttons(0) && (joystick_buttons(0) != buttons_memory(0))){
	        toggleTrunkControl();
	    }

	    //Kinematic adjustment
	    if(joystick_buttons(1) && (joystick_buttons(1) != buttons_memory(1))){
	        toggleKinematicAdjustmentFlag();
	    }

        //Exit jctp
        if(joystick_buttons(2))
        {
            //walkStarting();
            break;
        }

	    //Push recovery
	    if(joystick_buttons(3) && (joystick_buttons(3) != buttons_memory(3))){
	        togglePushRecoveryFlag();
	    }

	    //Terrain estimator
	    if(joystick_buttons(4) && (joystick_buttons(4) != buttons_memory(4))){
	        toggleTerrainEstimationFlag();
	    }

        //Decrease trunk height
        if(joystick_buttons(5) == 1  && (joystick_buttons(5) != buttons_memory(5))) {
            P0_Local_HF_New[dog::LF](rbd::Z) = -0.52;
            P0_Local_HF_New[dog::RF](rbd::Z) = -0.52;
            P0_Local_HF_New[dog::LH](rbd::Z) = -0.52;
            P0_Local_HF_New[dog::RH](rbd::Z) = -0.52;

            originChangingRate = filterResponse::slowC;
        }

        //Adaptive walking trot
        if(joystick_buttons(6) && (joystick_buttons(6) != buttons_memory(6))){
            toggleWalkAdaptationFlag();
        }

        //Reset trunk height
        if(joystick_buttons(7) == 1  && (joystick_buttons(7) != buttons_memory(7))) {
            P0_Local_HF_New[dog::LF](rbd::Z) = -0.6;
            P0_Local_HF_New[dog::RF](rbd::Z) = -0.6;
            P0_Local_HF_New[dog::LH](rbd::Z) = -0.6;
            P0_Local_HF_New[dog::RH](rbd::Z) = -0.6;

            originChangingRate = filterResponse::mediumC;
        }

        //Change between flat/rought terrain gait params
        if(joystick_buttons(8) == 1  && (joystick_buttons(8) != buttons_memory(8))) {
            if(roughTerrainMode){
                stepFrequencyNew = 1.7;
                dutyF_New = 0.5;
                roughTerrainMode = false;
                std::cout << "Selected gait params for flat terrain!!!" << std::endl;
            }
            else {
                stepFrequencyNew = 1.2;
                stepHeightNew = 0.13;
                dutyF_New = 0.65;
                setImpedanceRoughTerrain();
                roughTerrainMode = true;
                std::cout << "Selected gait params for rough terrain!!!" << std::endl;
            }
        }


        //Increase step height
        if(joystick_axes(5) == 1.0  && (joystick_axes(5) != axes_memory(5))) {
            stepHeightNew += 0.01;
            if(stepHeightNew > 0.2)
                stepHeightNew = 0.2;
        }

        //Decrease step height
        if(joystick_axes(5) == -1.0  && (joystick_axes(5) != axes_memory(5))) {
            stepHeightNew = 0.10;
        }

        //Start walking/trotting
        if(joystick_buttons(9) && (joystick_buttons(9) != buttons_memory(9))) {
            walkStarting();
        }

	    //OOO = stop walking/trotting
	    if(joystick_buttons(10) && (joystick_buttons(10) != buttons_memory(10))){
	        walkStopping();
	    }


	    //Update last input commands
	    axes_memory = joystick_axes;
	    buttons_memory = joystick_buttons;
	}
}


///*********************************************************************************************///
///*********************************************************************************************///
///*********************************************************************************************///
void RCFController::joyChangeTrotTaskForVisitors(void) {
    std::cout << "Joy CTP " << std::endl;
    std::cout << "Joystick X to exit" << std::endl;
    static Eigen::Matrix<double, 6, 1> axes_memory;
    static Eigen::Matrix<int, 12, 1> buttons_memory;
    static bool roughTerrainMode = false;
    static double VfX_buffer = 0.01;
    static double VfY_buffer = 0.0;
    static double Psit_buffer = 0.0;

/*    **********  Using wireless CSL gamepad for PS3 **********

    Axes channels     /     Commands
         0                Left Analog Left/Right
         1                Left Analog Up/Down
         2                Right Analog Left/Right
         3                Right Analog Up/Down
         4                Left  left and right cross buttons
         5                Left  up and down cross buttons
         6                Not used

    Button channels    /     Commands
          0                  Button 1
          1                  Button 2
          2                  Button 3
          3                  Button 4
          4                  Button L1
          5                  Button R1
          6                  Button L2
          7                  Button R2
          8                  Button SELECT
          9                  Button START
          10                 Button of left analog joystick
          11                 Button of right analog joystick
*/

    while(1)
    {
        //Longitudinal velocity and turning rate
        VfX_buffer = 0.4 * joystick_axes(1);
        Psit_buffer =  joystick_axes(2)* 30 * M_PI / 180 ;

        //Dead zone for VfX
        if(VfX_buffer > -0.025 && VfX_buffer < 0.025)
            VfX_buffer = 0.005;

        //Lateral velocity
        VfY_buffer = 0.3 * joystick_axes(0);

        //Dead zone for VfY
        if(VfY_buffer > -0.025 && VfY_buffer < 0.025)
            VfY_buffer = 0.0;



        //Update velocities when there is swing phase to avoid foot frontal
        //collision and the CPG get out of phase when dutyF is greater than 0.5
        if(!frontalCollisionFlag){
            if(forwVel > 0.025 || WCPGb[dog::LF](rbd::Z) > 0.33*stepHeight || WCPGb[dog::RF](rbd::Z) > 0.33*stepHeight){
                VfX_New = VfX_buffer;
                VfY_New = VfY_buffer;
                PsitNew = Psit_buffer;

                forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
                stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
            }
        } else{
            VfX_New = -0.01;
            VfY_New = 0.0;
            PsitNew = 0.0;

            forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
            stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
        }


        //Exit jctp
        if(joystick_buttons(2))
        {
            //walkStarting();
            break;
        }


        //Decrease trunk height
        if(joystick_buttons(5) == 1  && (joystick_buttons(5) != buttons_memory(5))) {
            P0_Local_HF_New[dog::LF](rbd::Z) = -0.52;
            P0_Local_HF_New[dog::RF](rbd::Z) = -0.52;
            P0_Local_HF_New[dog::LH](rbd::Z) = -0.52;
            P0_Local_HF_New[dog::RH](rbd::Z) = -0.52;

            originChangingRate = filterResponse::slowC;
        }

        //Reset trunk height
        if(joystick_buttons(7) == 1  && (joystick_buttons(7) != buttons_memory(7))) {
            P0_Local_HF_New[dog::LF](rbd::Z) = -0.6;
            P0_Local_HF_New[dog::RF](rbd::Z) = -0.6;
            P0_Local_HF_New[dog::LH](rbd::Z) = -0.6;
            P0_Local_HF_New[dog::RH](rbd::Z) = -0.6;

            originChangingRate = filterResponse::mediumC;
        }


        //Increase step height
        if(joystick_axes(5) == 1.0  && (joystick_axes(5) != axes_memory(5))) {
            stepHeightNew += 0.01;
            if(stepHeightNew > 0.15)
                stepHeightNew = 0.15;
        }

        //Decrease step height
        if(joystick_axes(5) == -1.0  && (joystick_axes(5) != axes_memory(5))) {
            stepHeightNew = 0.10;
        }

        //Start walking/trotting
        if(joystick_buttons(9) && (joystick_buttons(9) != buttons_memory(9))) {
            walkStarting();
        }

        //OOO = stop walking/trotting
        if(joystick_buttons(10) && (joystick_buttons(10) != buttons_memory(10))){
            walkStopping();
        }


        //Update last input commands
        axes_memory = joystick_axes;
        buttons_memory = joystick_buttons;
    }
}


///*********************************************************************************************///
///*********************************************************************************************///
///*********************************************************************************************///
void RCFController::interactiveChangeTrotTask(void) {
	std::cout << "Poor man's ICTP " << std::endl;
	std::cout << "w/s to +/- step frequency " << std::endl;
	std::cout << "a/z to +/- duty factor " << std::endl;
	std::cout << "f/b to +/- forward velocity " << std::endl;
	std::cout << "c/v to +/- lateral velocity " << std::endl;
	std::cout << "0 to trop in place" << std::endl;
	std::cout << "q to exit" << std::endl;

	/* use system call to make terminal send all keystrokes directly to stdin */
	int temp = system ("/bin/stty raw");

	int c;
	while(1)
	{
		c=getchar();
		//mtx.lock();
		std::string user_text(1, c);
		//If it returns false, break out of loop:
		if(!ICTPLogic(user_text))break;
		//mtx.unlock();
	}

	/* use system call to set terminal behaviour to default behaviour again*/
	temp = system ("/bin/stty cooked");

}


///*********************************************************************************************///
///*********************************************************************************************///
///*********************************************************************************************///
void RCFController::interactiveChangeTaskForDemos(void) {
    std::cout << "Poor man's ICTP " << std::endl;
    std::cout << "j/l to +/- trunk height [k for default] " << std::endl;
    std::cout << "f/b to +/- forward velocity " << std::endl;
    std::cout << "c/v to +/- lateral velocity " << std::endl;
    std::cout << "1/3 to +/- trunk pitch [2 for default]" << std::endl;
    std::cout << "7/9 to +/- trunk roll [8 for default]" << std::endl;
    std::cout << "0 to trop in place" << std::endl;
    std::cout << "q to exit" << std::endl;

    /* use system call to make terminal send all keystrokes directly to stdin */
    int temp = system ("/bin/stty raw");

    int c;
    while(1)
    {
        c=getchar();
        //mtx.lock();
        std::string user_text(1, c);
        //If it returns false, break out of loop:
        if(!ICTPLogicForDemos(user_text))break;
        //mtx.unlock();
    }

    /* use system call to set terminal behaviour to default behaviour again*/
    temp = system ("/bin/stty cooked");

}


///*********************************************************************************************///
///*********************************************************************************************///
///*********************************************************************************************///
bool RCFController::ICTPLogic(const std::string& str) {
	if(str == "w"){
		if (stepFrequencyNew <= 2.95) {
			stepFrequencyNew += 0.05;
			stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
			//cout << endl << "Step frequency:  " << stepFrequencyNew << endl;
		}
	}else if(str == "s"){
		if (stepFrequencyNew >= 1.05) {
			stepFrequencyNew -= 0.05;
			stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
			//cout << endl << "Step frequency:  " << stepFrequencyNew << endl;
		}
	}else if(str == "z"){
		if (dutyF_New > 0.3) {
			dutyF_New -= 0.05;
			stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
			//cout << endl << "Duty factor:  " << dutyF_New << endl;
		}
	}else if(str == "a"){
		if (dutyF_New < 0.8) {
			dutyF_New += 0.05;
			stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
			//cout << endl << "Duty factor:  " << dutyF_New << endl;
		}
	}else if(str == "f"){
		if (walkingTrotFlag) {
			VfX_New += forwVelInc;
			forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
			stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
		}
		if (runningTrotFlag) {
			if (!inProfileTransitionFlag) {
				switchRunningTrotProfile(Throttle::Faster);
				profileTransitionTime = 0.0;
			}
		}
	}else if(str == "b"){
		if (walkingTrotFlag) {
			VfX_New -= forwVelInc;
			forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
			stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
		}
		if (runningTrotFlag) {
			if (!inProfileTransitionFlag) {
				switchRunningTrotProfile(Throttle::Faster);
				profileTransitionTime = 0.0;
			}
		}
	}else if(str == "c"){
		if (VfX_New != 0.0) {
			VfY_New += forwVelInc;
			forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
			stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
		}
	}else if(str == "v"){
		if (VfX_New != 0.0) {
			VfY_New -= forwVelInc;
			forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
			stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
		}
    }else if(str == "n"){
        if (VfX_New != 0.0) {
            VfY_New = 0.0;
            forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
            stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
        }
	}else if(str == "0"){
        if (walkingTrotFlag){
            VfX_New = 0.0075;
            VfY_New = 0.0;
            PsitNew = 0.0;
            forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
            stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
        }
        if (runningTrotFlag)
            startRunningTrot();
	}else if(str == "1"){
		eulerAnglesNew(1) -= +0.0872;
	}else if(str == "2"){
		eulerAnglesNew(1) = 0.0;
	}else if(str == "3"){
		eulerAnglesNew(1) -= -0.0872;
	}else if(str == "7"){
		eulerAnglesNew(0) -= +0.04;
	}else if(str == "8"){
		eulerAnglesNew(0) = 0.0;
	}else if(str == "9"){
		eulerAnglesNew(0) -= -0.04;
	}else if(str == "4"){
		PsitNew += PsitInc;
	}else if(str == "5"){
		PsitNew = 0.0;
	}else if(str == "6"){
		PsitNew -= PsitInc;
	// }else if(str == "7"){
	// 	terrainPitchNew -= +0.0872;
	// }else if(str == "8"){
	// 	terrainPitchNew = 0.0;
	// }else if(str == "9"){
	//	terrainPitchNew -= -0.0872;
	}else if(str == "j"){
		P0_Local_HF_New[dog::LF][2] -= 0.01;
		P0_Local_HF_New[dog::RF][2] -= 0.01;
		P0_Local_HF_New[dog::LH][2] -= 0.01;
		P0_Local_HF_New[dog::RH][2] -= 0.01;
		//terrainPitchNew -= +0.0872;
	}else if(str == "k"){
		P0_Local_HF_New[dog::LF][2] = -0.60;
		P0_Local_HF_New[dog::RF][2] = -0.60;
		P0_Local_HF_New[dog::LH][2] = -0.60;
		P0_Local_HF_New[dog::RH][2] = -0.60;
		//terrainPitchNew = 0.0;
	}else if(str == "l"){
		P0_Local_HF_New[dog::LF][2] += 0.01;
		P0_Local_HF_New[dog::RF][2] += 0.01;
		P0_Local_HF_New[dog::LH][2] += 0.01;
		P0_Local_HF_New[dog::RH][2] += 0.01;
	}else if(str == ","){
		walkStopping();
	}else if(str == "q"){
		return false;
	}else if(str == "Q"){
		return false;
	}
	return true;
}

///*********************************************************************************************///
///*********************************************************************************************///
///*********************************************************************************************///
bool RCFController::ICTPLogicForDemos(const std::string& str) {

    if(str == "f"){
        if (walkingTrotFlag) {
            VfX_New += forwVelInc;
            forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
            stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
        }
        if (runningTrotFlag) {
            if (!inProfileTransitionFlag) {
                switchRunningTrotProfile(Throttle::Faster);
                profileTransitionTime = 0.0;
            }
        }
    }else if(str == "b"){
        if (walkingTrotFlag) {
            VfX_New -= forwVelInc;
            forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
            stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
        }
        if (runningTrotFlag) {
            if (!inProfileTransitionFlag) {
                switchRunningTrotProfile(Throttle::Faster);
                profileTransitionTime = 0.0;
            }
        }
    }else if(str == "c"){
        if (VfX_New != 0.0) {
            VfY_New += forwVelInc;
            forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
            stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
        }
    }else if(str == "v"){
        if (VfX_New != 0.0) {
            VfY_New -= forwVelInc;
            forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
            stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
        }
    }else if(str == "0"){
        if (walkingTrotFlag){
            VfX_New = 0.0075;
            VfY_New = 0.0;
            PsitNew = 0.0;
            forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
            stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
        }
        if (runningTrotFlag)
            startRunningTrot();
    }else if(str == "1"){
        eulerAnglesNew(1) -= +0.0872;
    }else if(str == "2"){
        eulerAnglesNew(1) = 0.0;
    }else if(str == "3"){
        eulerAnglesNew(1) -= -0.0872;
    }else if(str == "7"){
        eulerAnglesNew(0) -= +0.04;
    }else if(str == "8"){
        eulerAnglesNew(0) = 0.0;
    }else if(str == "9"){
        eulerAnglesNew(0) -= -0.04;
    }else if(str == "4"){
        PsitNew += PsitInc;
    }else if(str == "5"){
        PsitNew = 0.0;
    }else if(str == "6"){
        PsitNew -= PsitInc;
    }else if(str == "j"){
        P0_Local_HF_New[dog::LF][2] -= 0.05;
        if(P0_Local_HF_New[dog::LF][2] < -0.65)
            P0_Local_HF_New[dog::LF][2] = -0.65;
        P0_Local_HF_New[dog::RF][2] -= 0.05;
        if(P0_Local_HF_New[dog::RF][2] < -0.65)
            P0_Local_HF_New[dog::RF][2] = -0.65;
        P0_Local_HF_New[dog::LH][2] -= 0.05;
        if(P0_Local_HF_New[dog::LH][2] < -0.65)
            P0_Local_HF_New[dog::LH][2] = -0.65;
        P0_Local_HF_New[dog::RH][2] -= 0.05;
        if(P0_Local_HF_New[dog::RH][2] < -0.65)
            P0_Local_HF_New[dog::RH][2] = -0.65;
    }else if(str == "k"){
        P0_Local_HF_New[dog::LF][2] = -0.60;
        P0_Local_HF_New[dog::RF][2] = -0.60;
        P0_Local_HF_New[dog::LH][2] = -0.60;
        P0_Local_HF_New[dog::RH][2] = -0.60;
    }else if(str == "l"){
        P0_Local_HF_New[dog::LF][2] += 0.05;
        if(P0_Local_HF_New[dog::LF][2] > -0.15)
            P0_Local_HF_New[dog::LF][2] = -0.15;
        P0_Local_HF_New[dog::RF][2] += 0.05;
        if(P0_Local_HF_New[dog::RF][2] > -0.15)
            P0_Local_HF_New[dog::RF][2] = -0.15;
        P0_Local_HF_New[dog::LH][2] += 0.05;
        if(P0_Local_HF_New[dog::LH][2] > -0.15)
            P0_Local_HF_New[dog::LH][2] = -0.15;
        P0_Local_HF_New[dog::RH][2] += 0.05;
        if(P0_Local_HF_New[dog::RH][2] > -0.15)
            P0_Local_HF_New[dog::RH][2] = -0.15;
    }else if(str == ","){
        walkStopping();
    }else if(str == "q"){
        return false;
    }else if(str == "Q"){
        return false;
    }
    return true;
}

///*********************************************************************************************///

void RCFController::changeTrunkControllerGains() {
	int aux_int;
	double auxd;

	newline::getDouble("KpTrunkRoll [0.0, 5000]", KpTrunkRoll, auxd);
	if ((auxd < 0.0) || (auxd > 5000)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1) {
		    KpTrunkRollDefault = auxd;
		    //IF(attitudeFlag): For safety reasons when turning the trunk controller ON.
		    //This way the user can change the gains when the trunk controller is OFF.
		    if(attitudeFlag)
		        KpTrunkRollNew = KpTrunkRollDefault;
		}
	}

	newline::getDouble("KdTrunkRoll [0.0, 1000]", KdTrunkRoll, auxd);
	if ((auxd < 0.0) || (auxd > 1000)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1) {
		    KdTrunkRollDefault = auxd;
		    if(attitudeFlag)
		        KdTrunkRollNew = KdTrunkRollDefault;
		}
	}

	newline::getDouble("KpTrunkPitch [0.0, 5000]", KpTrunkPitch, auxd);
	if ((auxd < 0.0) || (auxd > 5000)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1) {
		    KpTrunkPitchDefault = auxd;
		    if(attitudeFlag)
		        KpTrunkPitchNew = KpTrunkPitchDefault;
		}
	}

	newline::getDouble("KdTrunkPitch [0.0, 1000]", KdTrunkPitch, auxd);
	if ((auxd < 0.0) || (auxd > 1000)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1) {
		    KdTrunkPitchDefault = auxd;
		    if(attitudeFlag)
		        KdTrunkPitchNew = KdTrunkPitchDefault;
		}
	}

	newline::getDouble("KpTrunkYaw [0.0, 200]", KpTrunkYaw, auxd);
	if ((auxd < 0.0) || (auxd > 200)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1) {
		    KpTrunkYawDefault = auxd;
		    if(attitudeFlag)
		        KpTrunkYawNew = KpTrunkYawDefault;
		}
	}

	newline::getDouble("KdTrunkYaw [0.0, 1000]", KdTrunkYaw, auxd);
	if ((auxd < 0.0) || (auxd > 1000)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1) {
		    KdTrunkYawDefault = auxd;
		    if(attitudeFlag)
		        KdTrunkYawNew = KdTrunkYawDefault;
		}
	}

	newline::getDouble("KpTrunkX [0.0, 10000]", KpTrunkX, auxd);
	if ((auxd < 0.0) || (auxd > 10000)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1) {
		    KpTrunkX_Default = auxd;
		    if(attitudeFlag)
		        KpTrunkX_New = KpTrunkX_Default;
		}
	}

	newline::getDouble("KdTrunkX [0.0, 10000]", KdTrunkX, auxd);
	if ((auxd < 0.0) || (auxd > 10000)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1) {
		    KdTrunkX_Default = auxd;
		    if(attitudeFlag)
		        KdTrunkX_New = KdTrunkX_Default;
		}
	}

	newline::getDouble("KpTrunkY [0.0, 10000]", KpTrunkY, auxd);
	if ((auxd < 0.0) || (auxd > 10000)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1) {
		    KpTrunkY_Default = auxd;
		    if(attitudeFlag)
		        KpTrunkY_New = KpTrunkY_Default;
		}
	}

	newline::getDouble("KdTrunkY [0.0, 10000]", KdTrunkY, auxd);
	if ((auxd < 0.0) || (auxd > 10000)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1) {
		    KdTrunkY_Default = auxd;
		    if(attitudeFlag)
		        KdTrunkY_New = KdTrunkY_Default;
		}
	}

	newline::getDouble("KpTrunkZ [0.0, 20000]", KpTrunkZ, auxd);
	if ((auxd < 0.0) || (auxd > 20000)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1) {
		    KpTrunkZ_Default = auxd;
		    if(attitudeFlag)
		        KpTrunkZ_New = KpTrunkZ_Default;
		}
	}

	newline::getDouble("KdTrunkz [0.0, 10000]", KdTrunkZ, auxd);
	if ((auxd < 0.0) || (auxd > 10000)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1) {
		    KdTrunkZ_Default = auxd;
		    if(attitudeFlag)
		        KdTrunkZ_New = KdTrunkZ_Default;
		}
	}

	newline::getDouble("Body_weight [0.0, 120]", bodyWeight, auxd);
	if ((auxd < 0.0) || (auxd > 120)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1) {
		    bodyWeightDefault = auxd;
		    if(attitudeFlag)
		        bodyWeightNew = bodyWeightDefault;
		}
	}

	newline::getDouble("Delta force [0.0, 200]", D_Force, auxd);
	if ((auxd < 0.0) || (auxd > 200)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			D_Force = auxd;
	}

	newline::getDouble("Delta COG X [-0.2, 0.2]", deltaCOG(rbd::X), auxd);
	if ((auxd < -0.2) || (auxd > 0.20)) {
		printf("unsafe value\n");
	} else {
		deltaCOG(rbd::X) = auxd;
	}

	newline::getDouble("Delta COG Y [-0.1, 0.1]", deltaCOG(rbd::Y), auxd);
	if ((auxd < -0.1) || (auxd > 0.1)) {
		printf("unsafe value\n");
	} else {
		deltaCOG(rbd::Y) = auxd;
	}

	newline::getDouble("Delta COG Z [-0.1, 0.1]", deltaCOG(rbd::Z), auxd);
	if ((auxd < -0.1) || (auxd > 0.1)) {
		printf("unsafe value\n");
	} else {
		deltaCOG(rbd::Z) = auxd;
	}

	aux_int = 0;
	if (desStateFlag) {
		aux_int = 1;
	}
	newline::getInt("Use desired state [1 for yes]", aux_int, aux_int);
	if (aux_int == 1)
		desStateFlag = true;
	if (aux_int == 0)
		desStateFlag = false;

	aux_int = 0;
	if (integralAction == true) {
		aux_int = 1;
	}
	cout << "Int Act Roll: " << integralActionRoll << endl;
	cout << "Int Act Pitch: " << integralActionPitch << endl;
	newline::getInt("Use integral action [1 for yes]", aux_int, aux_int);
	if (aux_int == 1)
		integralAction = true;
	if (aux_int == 0)
		integralAction = false;

}

///*********************************************************************************************///

void RCFController::changeKinematicDependency() {
	double auxd1;
	int aux_int;

	newline::getDouble("kadj roll [0, 1]", kadjRoll, auxd1);
	if ((auxd1 < 0) || (auxd1 > 1)) {
		printf("Value out of range\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			kadjRollNew = auxd1;
	}

	newline::getDouble("kadj pitch [0, 1]", kadjPitch, auxd1);
	if ((auxd1 < 0) || (auxd1 > 1)) {
		printf("Value out of range\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			kadjPitchNew = auxd1;
	}

	newline::getDouble("Max Roll ([0, 20] in degrees)", rollMaxKadj * 180 / 3.14,
			auxd1);
	if ((auxd1 < 0) || (auxd1 > 20)) {
		printf("Value out of range\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			rollMaxKadjNew = auxd1 * 3.14 / 180;
	}

	newline::getDouble("Max Pitch ([0, 15] in degrees)", pitchMaxKadj * 180 / 3.14,
			auxd1);
	if ((auxd1 < 0) || (auxd1 > 15)) {
		printf("Value out of range\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			pitchMaxKadjNew = auxd1 * 3.14 / 180;
	}

}

///*********************************************************************************************///

void RCFController::changeCOG() {
	double auxd1;
	int aux_int;

	newline::getDouble("COG X [-0.2, 0.2]", deltaCOG(rbd::X), auxd1);
	if ((auxd1 < -0.2) || (auxd1 > 0.2)) {
		printf("Value out of range\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			deltaCOG(rbd::X) = auxd1;
	}

	newline::getDouble("COG Y [-0.1, 0.1]", deltaCOG(rbd::Y), auxd1);
	if ((auxd1 < -0.1) || (auxd1 > 0.1)) {
		printf("Value out of range\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			deltaCOG(rbd::Y) = auxd1;
	}

	newline::getDouble("COG Z [-0.1, 0.1]", deltaCOG(rbd::Z), auxd1);
	if ((auxd1 < -0.1) || (auxd1 > 0.1)) {
		printf("Value out of range\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			deltaCOG(rbd::Z) = auxd1;
	}
}

///*********************************************************************************************///

void RCFController::changeCPG_Params() {
	double auxd;
	int aux_int;

	newline::getDouble("Alpha and Gamma [0, 200]", alphaGamma, auxd);
	if ((auxd < 0) || (auxd > 200)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			alphaGammaNew = auxd;
	}

	newline::getDouble("Kc gain (non-linear filter) (0, 80]", KcNew, auxd);
	if ((auxd <= 0) || (auxd > 80)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			KcNew = auxd;
	}

	newline::getDouble("Kbp [0, 100]", Kbp, auxd);
	if ((auxd < 0) || (auxd > 100)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			KbpNew = auxd;
	}

	newline::getDouble("Kbf [0, 100]", Kbf, auxd);
	if ((auxd < 0) || (auxd > 100)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			KbfNew = auxd;
	}

	newline::getDouble("Kbv [0, 100]", Kbv, auxd);
	if ((auxd < 0) || (auxd > 100)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			KbvNew = auxd;
	}

	newline::getDouble("Kvf [1, 5]", Kvf, auxd);
	if ((auxd < 1) || (auxd > 5)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			KvfNew = auxd;
	}

	newline::getInt("Height modulation [0, 1]", CPG_StepHeightModulationFlag, aux_int);
	    if ((aux_int == 0) || (aux_int == 1)) {
	        if(aux_int == 0 && aux_int != CPG_StepHeightModulationFlag) {
	            CPG_StepHeightModulationFlag = false;
	            for (int leg = dog::LF; leg <= dog::RH; leg++) {
	            CPG[leg].enableHeightModulation(CPG_StepHeightModulationFlag);
	            }
	            printf("CPG height modulation OFF!!! \n");
	        } else {
	            CPG_StepHeightModulationFlag = true;
	            for (int leg = dog::LF; leg <= dog::RH; leg++) {
	            CPG[leg].enableHeightModulation(CPG_StepHeightModulationFlag);
	            }
	            printf("CPG height modulation ON!!! \n");
	        }
	    }


}

///*********************************************************************************************///

void RCFController::changePrecParameters(){
	double auxd;
	int auxd2;
	double auxd3;

	newline::getDouble("precX [0,1]", precX, auxd);
	if ((auxd >= 0) && (auxd <= 1)) {
		precX = auxd;
		PushRecovery.setRecoveryGains(precX, precY, precYaw);
	} else {
		printf("It must be between 0 and 1");
	}

	newline::getDouble("precY [0,1]", precY, auxd);
	if ((auxd >= 0) && (auxd <= 1)) {
		precY = auxd;
		PushRecovery.setRecoveryGains(precX, precY, precYaw);
	} else {
		printf("It must be between 0 and 1");
	}

	newline::getDouble("precYaw [0,1]", precYaw, auxd);
	if ((auxd >= 0) && (auxd <= 1)) {
		precYaw = auxd;
		PushRecovery.setRecoveryGains(precX, precY, precYaw);
	} else {
		printf("It must be between 0 and 1");
	}

	newline::getDouble("Trunk weight [Kg]", precTrunkWeight, auxd3);
	if ((auxd3 > 0) && (auxd3 < 100)) {
		precTrunkWeight = auxd3;
		PushRecovery.setTrunkParameters(precTrunkWeight, 6.7, 0.747 / 2,
				0.414 / 2);
	} else {
		printf("\n It must be between 0 and 100 Kg!!!\n\n");
	}

	newline::getDouble("Min cut-off freq.", minCutOffFreq, auxd3);
	if ((auxd3 > 0) && (auxd3 < maxCutOffFreq)) {
		minCutOffFreq = auxd3;
	} else {
		printf("\n It must be > 0 and < maxCutOffFreq !!!\n\n");
	}

	newline::getDouble("Max cut-off freq.", maxCutOffFreq, auxd3);
	if (auxd3 > minCutOffFreq) {
		maxCutOffFreq = auxd3;
	} else {
		printf("\n It must be greater than minimum cut-off frequency !!!\n\n");
	}

	newline::getDouble("Use variable cut-off freq [0 1]", useVariableCutOffFreq,
			auxd3);
	if ((auxd3 == 0) || (auxd3 == 1)) {
		if (auxd3) {
			useVariableCutOffFreq = true;
		} else {
			useVariableCutOffFreq = false;
		}
	} else {
		printf("\n It must be 0 or 1 !!!\n\n");
	}

	newline::getDouble("Desired PR stance period", desiredTs_pRec, auxd3);
	if (auxd3 >= 0) {
		desiredTs_pRec = auxd3;
	} else {
		printf("\n It must be >= 0!!!\n\n");
	}

	newline::getInt("Enable PR in X [0,1]", PushRecovery.getOnOffPushRecoveryFlagX(),
			auxd2);
	if (auxd2 == 1) {
		precEnableX = true;
		PushRecovery.enablePushRecoveryX(true);
	}
	if (auxd2 == 0) {
		precEnableX = false;
		PushRecovery.enablePushRecoveryX(false);
	}

	newline::getInt("Enable PR in Y [0,1]", PushRecovery.getOnOffPushRecoveryFlagY(),
			auxd2);
	if (auxd2 == 1) {
		precEnableY = true;
		PushRecovery.enablePushRecoveryY(true);
	}
	if (auxd2 == 0) {
		precEnableY = false;
		PushRecovery.enablePushRecoveryY(false);
	}

	newline::getInt("Enable PR in Psi [0,1]",
			PushRecovery.getOnOffPushRecoveryFlagPsi(), auxd2);
	if (auxd2 == 1) {
		precEnablePsi = true;
		PushRecovery.enablePushRecoveryPsi(true);
	}
	if (auxd2 == 0) {
		precEnablePsi = false;
		PushRecovery.enablePushRecoveryPsi(false);
	}

}

///*********************************************************************************************///

void RCFController::changeTrunkOffset() {
	double auxd;
	int aux_int;

	newline::getDouble("X Force [-100, 100]", xForceOffset, auxd);
	if ((auxd < -100) || (auxd > 100)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			xForceOffsetNew = auxd;
	}

	newline::getDouble("Y Force [-100, 100]", yForceOffset, auxd);
	if ((auxd < -100) || (auxd > 100)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			yForceOffsetNew = auxd;
	}

	newline::getDouble("Z Force [-100, 100]", zForceOffset, auxd);
	if ((auxd < -100) || (auxd > 100)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			zForceOffsetNew = auxd;
	}

	newline::getDouble("X Moment [-50, 50]", xMomentOffset, auxd);
	if ((auxd < -50) || (auxd > 50)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			xMomentOffsetNew = auxd;
	}

	newline::getDouble("Y Moment [-50, 50]", yMomentOffset, auxd);
	if ((auxd < -50) || (auxd > 50)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			yMomentOffsetNew = auxd;
	}

	newline::getDouble("Z Moment [-50, 50]", zMomentOffset, auxd);
	if ((auxd < -50) || (auxd > 50)) {
		printf("unsafe value\n");
	} else {
		aux_int = 0;
		newline::getInt("   Are you SURE??? [1 for yes]", aux_int, aux_int);
		if (aux_int == 1)
			zMomentOffsetNew = auxd;
	}

}

///*********************************************************************************************///

void RCFController::changeOrigins() {

	int option = 99;
	double auxd1, auxd2;

	newline::getInt("Choose Leg Number [0 for all]", option, option);

	switch (option) {
	case 1:
		newline::getDouble("LF - X position", P0_Local_HF[dog::LF](rbd::X), auxd1);
		if (auxd1 < -0.1 || auxd1 > 0.1) {
			printf("unsafe input\n");
		} else {
			newline::getDouble("   Are you SURE??? -----> [Enter 1 for YES!]", 0,
					auxd2);
			if (auxd2 == 1)
				P0_Local_HF_New[dog::LF](rbd::X) += auxd1;
		}

		newline::getDouble("LF - Y position", P0_Local_HF[dog::LF](rbd::Y), auxd1);
		if (auxd1 < -0.1 || auxd1 > 0.1) {
			printf("unsafe input\n");
		} else {
			newline::getDouble("   Are you SURE??? -----> [Enter 1 for YES!]", 0,
					auxd2);
			if (auxd2 == 1)
				P0_Local_HF_New[dog::LF](rbd::Y) += auxd1;
		}

		newline::getDouble("LF - Z position", P0_Local_HF[dog::LF](rbd::Z), auxd1);
		if (auxd1 < -0.1 || auxd1 > 0.1) {
			printf("unsafe input\n");
		} else {
			newline::getDouble("   Are you SURE??? -----> [Enter 1 for YES!]", 0,
					auxd2);
			if (auxd2 == 1)
				P0_Local_HF_New[dog::LF](rbd::Z) += auxd1;
		}

		break;

	case 2:
		newline::getDouble("RF - X position", P0_Local_HF[dog::RF](rbd::X), auxd1);
		if (auxd1 < -0.1 || auxd1 > 0.1) {
			printf("unsafe input\n");
		} else {
			newline::getDouble("   Are you SURE??? -----> [Enter 1 for YES!]", 0,
					auxd2);
			if (auxd2 == 1)
				P0_Local_HF_New[dog::RF](rbd::X) += auxd1;
		}

		newline::getDouble("RF - Y position", P0_Local_HF[dog::RF](rbd::Y), auxd1);
		if (auxd1 < -0.1 || auxd1 > 0.1) {
			printf("unsafe input\n");
		} else {
			newline::getDouble("   Are you SURE??? -----> [Enter 1 for YES!]", 0,
					auxd2);
			if (auxd2 == 1)
				P0_Local_HF_New[dog::RF](rbd::Y) += auxd1;
		}

		newline::getDouble("RF - Z position", P0_Local_HF[dog::RF](rbd::Z), auxd1);
		if (auxd1 < -0.1 || auxd1 > 0.1) {
			printf("unsafe input\n");
		} else {
			newline::getDouble("   Are you SURE??? -----> [Enter 1 for YES!]", 0,
					auxd2);
			if (auxd2 == 1)
				P0_Local_HF_New[dog::RF](rbd::Z) += auxd1;
		}

		break;

	case 3:
		newline::getDouble("LH - X position", P0_Local_HF[dog::LH](rbd::X), auxd1);
		if (auxd1 < -0.1 || auxd1 > 0.1) {
			printf("unsafe input\n");
		} else {
			newline::getDouble("   Are you SURE??? -----> [Enter 1 for YES!]", 0,
					auxd2);
			if (auxd2 == 1)
				P0_Local_HF_New[dog::LH](rbd::X) += auxd1;
		}

		newline::getDouble("LH - Y position", P0_Local_HF[dog::LH](rbd::Y), auxd1);
		if (auxd1 < -0.1 || auxd1 > 0.1) {
			printf("unsafe input\n");
		} else {
			newline::getDouble("   Are you SURE??? -----> [Enter 1 for YES!]", 0,
					auxd2);
			if (auxd2 == 1)
				P0_Local_HF_New[dog::LH](rbd::Y) += auxd1;
		}

		newline::getDouble("LH - Z position", P0_Local_HF[dog::LH](rbd::Z), auxd1);
		if (auxd1 < -0.1 || auxd1 > 0.1) {
			printf("unsafe input\n");
		} else {
			newline::getDouble("   Are you SURE??? -----> [Enter 1 for YES!]", 0,
					auxd2);
			if (auxd2 == 1)
				P0_Local_HF_New[dog::LH](rbd::Z) += auxd1;
		}

		break;

	case 4:
		newline::getDouble("RH - X position", P0_Local_HF[dog::RH](rbd::X), auxd1);
		if (auxd1 < -0.1 || auxd1 > 0.1) {
			printf("unsafe input\n");
		} else {
			newline::getDouble("   Are you SURE??? -----> [Enter 1 for YES!]", 0,
					auxd2);
			if (auxd2 == 1)
				P0_Local_HF_New[dog::RH](rbd::X) += auxd1;
		}

		newline::getDouble("RH - Y position", P0_Local_HF[dog::RH](rbd::Y), auxd1);
		if (auxd1 < -0.1 || auxd1 > 0.1) {
			printf("unsafe input\n");
		} else {
			newline::getDouble("   Are you SURE??? -----> [Enter 1 for YES!]", 0,
					auxd2);
			if (auxd2 == 1)
				P0_Local_HF_New[dog::RH](rbd::Y) += auxd1;
		}

		newline::getDouble("RH - Z position", P0_Local_HF[dog::RH](rbd::Z), auxd1);
		if (auxd1 < -0.1 || auxd1 > 0.1) {
			printf("unsafe input\n");
		} else {
			newline::getDouble("   Are you SURE??? -----> [Enter 1 for YES!]", 0,
					auxd2);
			if (auxd2 == 1)
				P0_Local_HF_New[dog::RH](rbd::Z) += auxd1;
		}

		break;

	case 0:
		newline::getDouble("Z offset", P0_Local_HF[dog::LF](rbd::Z), auxd1);
		if (auxd1 < -0.2 || auxd1 > 0.2) {
			printf("unsafe offset\n");
		} else {
			newline::getDouble("   Are you SURE??? -----> [Enter 1 for YES!]", 0,
					auxd2);
			if (auxd2 == 1) {
				P0_Local_HF_New[dog::LF](rbd::Z) += -auxd1;
				P0_Local_HF_New[dog::RF](rbd::Z) += -auxd1;
				P0_Local_HF_New[dog::LH](rbd::Z) += -auxd1;
				P0_Local_HF_New[dog::RH](rbd::Z) += -auxd1;
			}
		}

		break;

	case 10:
	    newline::getDouble("Symmetric X offset", P0_Local_HF[dog::LF](rbd::X), auxd1);
	    if (auxd1 < -0.1 || auxd1 > 0.1) {
	        printf("unsafe offset\n");
	    } else {
	        newline::getDouble("   Are you SURE??? -----> [Enter 1 for YES!]", 0,
	                auxd2);
	        if (auxd2 == 1) {
	            P0_Local_HF_New[dog::LF](rbd::X) += auxd1;
	            P0_Local_HF_New[dog::RF](rbd::X) += auxd1;
	            P0_Local_HF_New[dog::LH](rbd::X) -= auxd1;
	            P0_Local_HF_New[dog::RH](rbd::X) -= auxd1;
	        }
	    }

	default:
		printf("Wrong leg!\n");
	}

	originChangingRate = filterResponse::slowC;
}

///*********************************************************************************************///

void RCFController::controlStatus() //Display in the screen all the main functions that are ON
{
	static double resFreq = 0.0;
	static double linearStiffness = 1000.0;
	static double resonantFrequency = 1.0;

	if (walkingTrotFlag)
		printf("\n    Walking trot gait\n\n");

	if (runningTrotFlag)
		printf("\n    Running trot gait\n\n");

	if (freezeRatioFlag)
		printf("    Ls/Vf Ratio frozen\n\n");

	if (attitudeFlag)
		printf("    Trunk control\n\n");

	if (walkAdaptationFlag)
		printf("    Adaptive walk\n\n");

	if (pushRecoveryFlag)
		printf("    Push recovery\n\n");

	if (kinAdjustmentFlag)
		printf("    Kinematic adjustment\n\n");

	if (terrainEstimationFlag)
		printf("    Terrain estimation\n\n");

	if (forcedStanceFlag)
		printf("    Forced stance\n\n");

	if (useStanceHolderFlag)
	    printf("    Stance holder\n\n");

	if (positionControlFlag)
	    printf("    Robot position control\n\n");

	if (useRCF_StateEstimator)
	        printf("    RCF Kin+IMU vel state estimator\n\n");

	if (useCollisionDetectionFlag)
	        printf("    Collision detection\n\n");

	if (useStepReactionFlag)
	        printf("    Step reaction\n\n");



	resFreq = 1.0 / 2.0 / 3.1415 * sqrt(20 * zeroGainTh[dog::LF_KFE] / 30)
			* sqrt(	1 - 20 * zeroGainThd[dog::LF_KFE] * 20 * zeroGainThd[dog::LF_KFE] / 4 / 30 / 20
									/ zeroGainTh[dog::LF_KFE]);

	resonantFrequency = 1/(2*stepLength/forwVel);

	linearStiffness = bodyWeight * (2 * 3.14 * resonantFrequency) * (2 * 3.14 * resonantFrequency);

	printf(
			"                                            Desired Step Frequency: %f\n\n",
			stepFrequency);
	printf(
			"                                             Actual Step Frequency: %f\n\n",
			actualStepFrequency);
	cout	<< "                                                 CPG stance period: "
			<< stepLength / forwVel << endl;
	cout	<< "                                           Resonance stance period: "
			<< 0.5 / resFreq << endl;
	cout    << "                                              Actual stance period: "
			<< actualStancePeriod << endl << endl;
	cout    << "                                     Ideal linear spring stiffness: "
			<< linearStiffness << endl << endl;

}

///*********************************************************************************************///

void RCFController::menuOptions() {
	printf(
			"             ctp -- Change WCPG parameters\n"
					"             stw -- Start walking using default parameters\n"
					"              stw_rough -- Start walking using rough terrain gait setup\n"
					"             ooo -- Stop walking using default parameters\n"
					"             frt -- Toggle frozen ratio between Vf and Ls\n"
					"             wor -- Print position of limit cycle origin\n"
					"             cor -- Change limit cycle origin (z direction)\n"
					"            prec -- Toggle push recobery mode\n"
					"            kadj -- Toggle kinematic adjustment\n"
					"            ictp -- Interactive ctp\n"
					"           change_Vf -- Change forward velocity\n"
					"           change_Ls -- Change step length\n"
					"           change_Fc -- Change step height\n"
					"           change_Dc -- Change duty cycle\n"
					"            att_cont -- Toggle attitude control\n"
					"          walk_adapt -- Toggle adaptive walking\n");
}

///*********************************************************************************************///

void RCFController::whereOrigins() {

	printf("\n    LF: [%f     %f     %f]\n\n", P0_Local_HF[dog::LF](rbd::X),
			P0_Local_HF[dog::LF](rbd::Y), P0_Local_HF[dog::LF](rbd::Z));

	printf("\n    RF: [%f     %f     %f]\n\n", P0_Local_HF[dog::RF](rbd::X),
			P0_Local_HF[dog::RF](rbd::Y), P0_Local_HF[dog::RF](rbd::Z));

	printf("\n    LH: [%f     %f     %f]\n\n", P0_Local_HF[dog::LH](rbd::X),
			P0_Local_HF[dog::LH](rbd::Y), P0_Local_HF[dog::LH](rbd::Z));

	printf("\n    RH: [%f     %f     %f]\n\n", P0_Local_HF[dog::RH](rbd::X),
			P0_Local_HF[dog::RH](rbd::Y), P0_Local_HF[dog::RH](rbd::Z));

}

///*********************************************************************************************///

void RCFController::narrowStance() {
	double stanceOffset = 0.08 / 2;

	P0_Local_HF_New[dog::LF](rbd::Y) -= stanceOffset;
	P0_Local_HF_New[dog::RF](rbd::Y) += stanceOffset;
	P0_Local_HF_New[dog::LH](rbd::Y) -= stanceOffset;
	P0_Local_HF_New[dog::RH](rbd::Y) += stanceOffset;
	originChangingRate = filterResponse::slowC;

}

///*********************************************************************************************///

void RCFController::wideStance() {
	double stanceOffset = 0.08 / 2;

	P0_Local_HF_New[dog::LF](rbd::Y) += stanceOffset;
	P0_Local_HF_New[dog::RF](rbd::Y) -= stanceOffset;
	P0_Local_HF_New[dog::LH](rbd::Y) += stanceOffset;
	P0_Local_HF_New[dog::RH](rbd::Y) -= stanceOffset;
	originChangingRate = filterResponse::slowC;

}

///*********************************************************************************************///

void RCFController::changeSinParameters() {
	double auxd;

	newline::getDouble("Desired amplitude [0.0, 0.1]", sinAmplitude, auxd);
	if ((auxd > 0.0) && (auxd <= 0.3))
		sinAmplitude = auxd;

	newline::getDouble("Desired frequency [0.0, 2.0]", sinFrequency, auxd);
	if ((auxd > 0.0) && (auxd <= 2))
		sinFrequency = auxd;
}

///*********************************************************************************************///

void RCFController::changeSinParametersForce() {
    double auxd;

    newline::getDouble("Desired amplitude [0.0, 200]", sinForceAmplitude, auxd);
    if ((auxd > 0.0) && (auxd <= 200))
        sinForceAmplitude = auxd;

    newline::getDouble("Desired frequency [0.0, 10.0]", sinForceFrequency, auxd);
    if ((auxd > 0.0) && (auxd <= 10))
        sinForceFrequency = auxd;
}

///*********************************************************************************************///

void RCFController::changeChirpParameters() {
    double auxd;

    newline::getDouble("Chirp amplitude [0.0, 100]", chirpAmplitude, auxd);
    if ((auxd > 0.0) && (auxd <= 100))
        chirpAmplitude = auxd;

    newline::getDouble("Initial frequency [0.0, 100.0)", initialChirpFrequency, auxd);
    if ((auxd > 0.0) && (auxd < 100))
        initialChirpFrequency = auxd;

    newline::getDouble("Final frequency [f0, 100.0]", finalChirpFrequency, auxd);
    if ((auxd > initialChirpFrequency) && (auxd <= 100))
        finalChirpFrequency = auxd;

    newline::getDouble("Chirp duration [1, 20.0]", chirpDuration, auxd);
    if ((auxd > 1) && (auxd <= 20))
        chirpDuration = auxd;

    newline::getDouble("Gain leg 1 [0, 1]", internalForceGain[dog::LF], auxd);
    if ((auxd >= 0.0) && (auxd <= 1.0))
        internalForceGain[dog::LF] = auxd;

    newline::getDouble("Gain leg 2 [0, 1]", internalForceGain[dog::RF], auxd);
    if ((auxd >= 0.0) && (auxd <= 1.0))
        internalForceGain[dog::RF] = auxd;

    newline::getDouble("Gain leg 3 [0, 1]", internalForceGain[dog::LH], auxd);
    if ((auxd >= 0.0) && (auxd <= 1.0))
        internalForceGain[dog::LH] = auxd;

    newline::getDouble("Gain leg 4 [0, 1]", internalForceGain[dog::RH], auxd);
    if ((auxd >= 0.0) && (auxd <= 1.0))
        internalForceGain[dog::RH] = auxd;

    chirpRate = (finalChirpFrequency - initialChirpFrequency) / chirpDuration;
}

///*********************************************************************************************///

void RCFController::changeSquareWaveParameters() {
    double auxd;

    newline::getDouble("Square wave amplitude [0.0, 100]", squareWaveAmplitude, auxd);
    if ((auxd > 0.0) && (auxd <= 100))
        squareWaveAmplitude = auxd;

    newline::getDouble("Square wave period (0, 5.0]", squareWavePeriod, auxd);
    if ((auxd > 0) && (auxd <= 5))
        squareWavePeriod = auxd;
}

///*********************************************************************************************///

void RCFController::changeTriangularWaveParameters() {
    double auxd;

    newline::getDouble("Triangular wave amplitude [0.0, 100]", triangularWaveAmplitude, auxd);
    if ((auxd > 0.0) && (auxd <= 100))
        triangularWaveAmplitude = auxd;

    newline::getDouble("Triangular wave period (0, 20.0]", triangularWavePeriod, auxd);
    if ((auxd > 0) && (auxd <= 20))
        triangularWavePeriod = auxd;
}

///*********************************************************************************************///

void RCFController::changeSineWaveParameters() {
    double auxd;

    newline::getDouble("Sine wave amplitude [0.0, 100]", sineWaveAmplitude, auxd);
    if ((auxd > 0.0) && (auxd <= 100))
        sineWaveAmplitude = auxd;

    newline::getDouble("Sine wave period (0, 20.0]", sineWavePeriod, auxd);
    if ((auxd > 0) && (auxd <= 20))
        sineWavePeriod = auxd;
}

///*********************************************************************************************///

void RCFController::togglePushUpTask() {
	sinOnOffFlag = !sinOnOffFlag;
}

///*********************************************************************************************///

void RCFController::toggleInternalForcesTask() {
    sinOnOffForceFlag = !sinOnOffForceFlag;
}

///*********************************************************************************************///

void RCFController::toggleChirpTask() {
    chirpOnOffFlag = !chirpOnOffFlag;

    //Reset chirp current time
    if(chirpOnOffFlag){ chirpTime0 = myTime;}
}

///*********************************************************************************************///

void RCFController::toggleSquareWaveTask() {
    squareWaveOnOffFlag = !squareWaveOnOffFlag;

    //Reset chirp current time
    if(squareWaveOnOffFlag){
        squareWaveOutput = 0.0;
        squareWaveOutputFiltered = 0.0;
        squareTime0 = myTime;
    }
}

///*********************************************************************************************///

void RCFController::toggleTriangularWaveTask() {
    triangularWaveOnOffFlag = !triangularWaveOnOffFlag;

    //Reset chirp current time
    if(triangularWaveOnOffFlag){
        triangularWaveOutput = 0.0;
        triangularTime0 = myTime;
    }
}

///*********************************************************************************************///

void RCFController::toggleSineWaveTask() {
    sineWaveOnOffFlag = !sineWaveOnOffFlag;

    //Reset chirp current time
    if(sineWaveOnOffFlag){
        sineWaveOutput = 0.0;
        sineTime0 = myTime;
    }
}

///*********************************************************************************************///

void RCFController::startRunningTrot() {

	runningTrot(0);

	PsitNew = 0.0;

	//trunk controller tunning
	KpTrunkRollNew = 2000;
	KdTrunkRollNew = 200;
	KpTrunkPitchNew = 2000;
	KdTrunkPitchNew = 200;
	KdTrunkZ_New = 0.0;
	KpTrunkZ_New = 3000;

	KvfNew = 2.0;


}

///*********************************************************************************************///
// Currently stepFrequencyNew is overwritten, so the function is useless
void RCFController::changeStepFrequency(void) {
	double auxd;

	newline::getDouble("Step Frequency [1.0, 3.0]", stepFrequency, auxd);
	if ((auxd < 1.0) || (auxd > 3.0)) {
		printf("unsafe value\n");
	} else {
		stepFrequencyNew = auxd;

		stepFrequencyNew = forwVelNew * dutyF_New / stepFrequencyNew;
	}

}

///*********************************************************************************************///

void RCFController::changeForwardVelocity(void) {

	double auxd;

	newline::getDouble("Forward Velocity [0-3]", VfX, auxd);
	if ((auxd <= 0.0) || (auxd > 3.0)) {
		printf("unsafe freq\n");
	} else {
		VfX_New = auxd;
		forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
	}

	stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
}

///*********************************************************************************************///

void RCFController::changeDutyCycle(void) {

	double auxd;

	newline::getDouble("DutyF [0.3, 0.9]", dutyF, auxd);
	if ((auxd < 0.3) || (auxd > 0.9)) {
		printf("unsafe value\n");
	} else {
		dutyF_New = auxd;

		stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
	}

}


///*********************************************************************************************///

void RCFController::changeTerrainAdjustments(void) {

    double auxd;

    newline::getDouble("Adjustment factor [0.0, 1.0]", terrainBasedAdjustment.adjustmentFactor, auxd);
    if ((auxd < 0.0) || (auxd > 1.0)) {
        printf("unsafe value\n");
    } else {
        terrainBasedAdjustment.adjustmentFactor = auxd;
    }

    newline::getDouble("Filter time [0.0, 10.0]", terrainBasedAdjustment.filterTimeConstant, auxd);
    if ((auxd < 0.0) || (auxd > 10.0)) {
        printf("unsafe value\n");
    } else {
        terrainBasedAdjustment.filterTimeConstant = auxd;
    }

}

///*********************************************************************************************///

void RCFController::changeLegImpedanceDuringStancePhase(void) {

	double aux_gain;
	double aux_HAA_Kp;
	double aux_HFE_Kp;
	double aux_KFE_Kp;
	double aux_HAA_Kd;
	double aux_HFE_Kd;
	double aux_KFE_Kd;


	newline::getDouble("HAA_Kp_stance",
			PIDManager.jointsStancePIDGains(PIDManager.LF_HAA,PIDManager.Kp), aux_gain);
	if (aux_gain < 0.0 || aux_gain > 600) {
		printf("unsafe input\n");
		aux_HAA_Kp = PIDManager.jointsStancePIDGains(PIDManager.LF_HAA,PIDManager.Kp);
	} else {
		aux_HAA_Kp = aux_gain;
	}

	newline::getDouble("HFE_Kp_stance",
			PIDManager.jointsStancePIDGains(PIDManager.LF_HFE,PIDManager.Kp), aux_gain);
	if (aux_gain < 0.0 || aux_gain > 600) {
		printf("unsafe input\n");
		aux_HFE_Kp = PIDManager.jointsStancePIDGains(PIDManager.LF_HFE,PIDManager.Kp);
	} else {
		aux_HFE_Kp = aux_gain;
	}

	newline::getDouble("KFE_Kp_stance",
			PIDManager.jointsStancePIDGains(PIDManager.LF_KFE,PIDManager.Kp), aux_gain);
	if (aux_gain < 0.0 || aux_gain > 600) {
		printf("unsafe input\n");
		aux_KFE_Kp = PIDManager.jointsStancePIDGains(PIDManager.LF_KFE,PIDManager.Kp);
	} else {
		aux_KFE_Kp = aux_gain;
	}

	newline::getDouble("HAA_Kd_stance",
			PIDManager.jointsStancePIDGains(PIDManager.LF_HAA,PIDManager.Kd), aux_gain);
	if (aux_gain < 0.0 || aux_gain > 50) {
		printf("unsafe input\n");
		aux_HAA_Kd = PIDManager.jointsStancePIDGains(PIDManager.LF_HAA,PIDManager.Kd);
	} else {
		aux_HAA_Kd = aux_gain;
	}

	newline::getDouble("HFE_Kd_stance",
			PIDManager.jointsStancePIDGains(PIDManager.LF_HFE,PIDManager.Kd), aux_gain);
	if (aux_gain < 0.0 || aux_gain > 50) {
		printf("unsafe input\n");
		aux_HFE_Kd = PIDManager.jointsStancePIDGains(PIDManager.LF_HFE,PIDManager.Kd);
	} else {
		aux_HFE_Kd = aux_gain;
	}

	newline::getDouble("KFE_Kd_stance",
			PIDManager.jointsStancePIDGains(PIDManager.LF_KFE,PIDManager.Kd), aux_gain);
	if (aux_gain < 0.0 || aux_gain > 50) {
		printf("unsafe input\n");
		aux_KFE_Kd = PIDManager.jointsStancePIDGains(PIDManager.LF_KFE,PIDManager.Kd);
	} else {
		aux_KFE_Kd = aux_gain;
	}

	PIDManager.setStanceJointPIDGains(PIDManager.LF_HAA, aux_HAA_Kp, 0.0, aux_HAA_Kd);
	PIDManager.setStanceJointPIDGains(PIDManager.LF_HFE, aux_HFE_Kp, 0.0, aux_HFE_Kd);
	PIDManager.setStanceJointPIDGains(PIDManager.LF_KFE, aux_KFE_Kp, 0.0, aux_KFE_Kd);
	PIDManager.setStanceJointPIDGains(PIDManager.RF_HAA, aux_HAA_Kp, 0.0, aux_HAA_Kd);
	PIDManager.setStanceJointPIDGains(PIDManager.RF_HFE, aux_HFE_Kp, 0.0, aux_HFE_Kd);
	PIDManager.setStanceJointPIDGains(PIDManager.RF_KFE, aux_KFE_Kp, 0.0, aux_KFE_Kd);
	PIDManager.setStanceJointPIDGains(PIDManager.LH_HAA, aux_HAA_Kp, 0.0, aux_HAA_Kd);
	PIDManager.setStanceJointPIDGains(PIDManager.LH_HFE, aux_HFE_Kp, 0.0, aux_HFE_Kd);
	PIDManager.setStanceJointPIDGains(PIDManager.LH_KFE, aux_KFE_Kp, 0.0, aux_KFE_Kd);
	PIDManager.setStanceJointPIDGains(PIDManager.RH_HAA, aux_HAA_Kp, 0.0, aux_HAA_Kd);
	PIDManager.setStanceJointPIDGains(PIDManager.RH_HFE, aux_HFE_Kp, 0.0, aux_HFE_Kd);
	PIDManager.setStanceJointPIDGains(PIDManager.RH_KFE, aux_KFE_Kp, 0.0, aux_KFE_Kd);


}

///*********************************************************************************************///

void RCFController::changePID_Manager(void) {

	double aux_gain;
	int aux_int;

	newline::getDouble("Smoothing Gain", PIDManager.getSmoothingGain(), aux_gain);
	PIDManager.setSmoothingGain(aux_gain);

	newline::getDouble("Step Height Threshold", PIDManager.getStepHeightThreshold(), aux_gain);
	PIDManager.setStepHeightThreshold(aux_gain);

	newline::getInt("Use PID Manager (0 or 1)", PIDManager.usePIDManager, aux_int);
	if ((aux_int == 0) || (aux_int == 1)) {
		if(aux_int == 0) {
			PIDManager.usePIDManager = false;
		}
		else{
			PIDManager.usePIDManager = true;
		}
	}

	newline::getDouble("HAA Inertia Comp Gain [0,1]", PIDManager.inertialCompJointGain(PIDManager.LF_HAA), aux_gain);
	if ((aux_int >= 0) && (aux_int <= 1)) {
		PIDManager.inertialCompJointGain(PIDManager.LF_HAA) = aux_gain;
		PIDManager.inertialCompJointGain(PIDManager.RF_HAA) = aux_gain;
		PIDManager.inertialCompJointGain(PIDManager.LH_HAA) = aux_gain;
		PIDManager.inertialCompJointGain(PIDManager.RH_HAA) = aux_gain;
	}


	newline::getDouble("HAA Inertia Comp Gain [0,1]", PIDManager.inertialCompJointGain(PIDManager.LF_HFE), aux_gain);
	if ((aux_int >= 0) && (aux_int <= 1)) {
		PIDManager.inertialCompJointGain(PIDManager.LF_HFE) = aux_gain;
		PIDManager.inertialCompJointGain(PIDManager.RF_HFE) = aux_gain;
		PIDManager.inertialCompJointGain(PIDManager.LH_HFE) = aux_gain;
		PIDManager.inertialCompJointGain(PIDManager.RH_HFE) = aux_gain;
	}


	newline::getDouble("HAA Inertia Comp Gain [0,1]", PIDManager.inertialCompJointGain(PIDManager.LF_KFE), aux_gain);
	if ((aux_int >= 0) && (aux_int <= 1)) {
		PIDManager.inertialCompJointGain(PIDManager.LF_KFE) = aux_gain;
		PIDManager.inertialCompJointGain(PIDManager.RF_KFE) = aux_gain;
		PIDManager.inertialCompJointGain(PIDManager.LH_KFE) = aux_gain;
		PIDManager.inertialCompJointGain(PIDManager.RH_KFE) = aux_gain;
	}

    newline::getInt("Set manager mode [1, 2 or 3]:", PIDManager.getMode(), aux_int);
    PIDManager.setMode(aux_int);

}

///*********************************************************************************************///

void RCFController::changeRCF_Parameters() {

	double auxd;
	int aux_int;

	newline::getDouble("Stance Hs (0-0.2]", CPG[dog::LF].stanceHs, auxd);
	if ((auxd < 0.0) || (auxd > 0.2)) {
		printf("unsafe value\n");
	}
	else {
		for (int leg = dog::LF; leg <= dog::RH; leg++) {
			CPG[leg].stanceHs = auxd;
		}
	}

	newline::getDouble("Comp. touch-down errors [0-1]", tdErrorComp, auxd);
	if ((auxd < 0.0) || (auxd > 1.0))
		printf("unsafe value\n");
	else
		tdErrorComp = auxd;

	//change reference parameters
	newline::getDouble("Step Height [0-0.3]", stepHeightNew, auxd);
	if ((auxd <= 0.0) || (auxd > 0.3))
		printf("unsafe height\n");
	else
		stepHeightNew = auxd;

	newline::getDouble("Forward Velocity [0-5]", VfX, auxd);
	if ((auxd < -5.0) || (auxd > 5)) {
		printf("unsafe freq\n");
	} else {
		VfX_New = auxd;
		forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
	}

	stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;

	newline::getDouble("Step Frequency [1.0, 3.0]", stepFrequency, auxd);
	if ((auxd < 1.0) || (auxd > 3.0))
		printf("unsafe value\n");
	else {
		stepFrequencyNew = auxd;

		stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
	}

	newline::getDouble("dutyF [0.3, 0.9]", dutyF, auxd);
	if ((auxd < 0.3) || (auxd > 0.9))
		printf("unsafe value\n");
	else {
		dutyF_New = auxd;

		stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
	}

	newline::getDouble("Step Length [0-0.6]", stepLengthNew, auxd);
	if ((auxd <= 0.0) || (auxd > 0.6))
		printf("unsafe length\n");
	else
		stepLengthNew = auxd;

	newline::getDouble("Roll angle (about x direction) [-90, 90]",
			eulerAngles(0) * 180 / M_PI, auxd);
	if ((auxd < -90) || (auxd > 90))
		printf("unsafe angle\n");
	else
		eulerAnglesNew(0) = auxd / 180 * M_PI;

	newline::getDouble("Pitch angle (about y direction) [-90, 90]",	eulerAngles(1) * 180 / M_PI, auxd);
	if ((auxd < -90) || (auxd > 90))
		printf("unsafe angle\n");
	else
		eulerAnglesNew(1) = auxd / 180 * M_PI;

	newline::getDouble("Kcoup [0, 10]", Kcoup, auxd);
	if ((auxd < 0) || (auxd > 10))
		printf("unsafe value\n");
	else
		KcoupNew = auxd;

	newline::getInt("Select Loc. Pattern: 0 for trot \n 1 for walk \n 2 for bound:",
			(int)pattern, aux_int);
	if (aux_int == 0)
		pattern = Pattern::Trot;
	if (aux_int == 1)
		pattern = Pattern::Walk;
	if (aux_int == 2)
		pattern = Pattern::Bound;

	newline::getDouble("Jump Force [0.0, 10000]",userJumpForce,auxd);
	if ((auxd < 0) || (auxd > 10000))
		printf("unsafe value\n");
	else
		userJumpForce = auxd;

	newline::getDouble("Squat Height [0.3, 0.6]",squatHeight,auxd);
	if ((auxd < 0.3) || (auxd > 0.6))
		printf("unsafe value\n");
	else
		squatHeight = auxd;

}

///*********************************************************************************************///

void RCFController::imuPrint(void) {
    std::cout << "Lin Pos = " << bs->getPosition_W()[0] << " " << bs->getPosition_W()[1] << " " << bs->getPosition_W()[2] << std::endl;
    std::cout << "Lin Vel = " << bs->getVelocity_W()[0] << " " << bs->getVelocity_W()[1] << " " << bs->getVelocity_W()[2] << std::endl;
    std::cout << "Lin Acc = " << bs->getAcceleration_B()[0] << " " << bs->getAcceleration_B()[1] << " " << bs->getAcceleration_B()[2] << std::endl;
    std::cout << "Ang Pos = " << bs->getRoll_W() << " " << bs->getPitch_W() << " " << bs->getYaw_W() << std::endl;
    std::cout << "Ang Rate = " << bs->getRotationRate_B()[0] << " " << bs->getRotationRate_B()[1] << " " << bs->getRotationRate_B()[2] << std::endl;
    std::cout << "Ang Acc = " << bs->getRotAcceleration_B()[0] << " " << bs->getRotAcceleration_B()[1] << " " << bs->getRotAcceleration_B()[2] << std::endl;
}


void RCFController::footSensorPrint(void) {
	std::cout << "footSensor LF " << footSensor->force[0][0] << " " << footSensor->force[0][1] << " " << footSensor->force[0][2] << std::endl;
	std::cout << "footSensor RF " << footSensor->force[1][0] << " " << footSensor->force[1][1] << " " << footSensor->force[1][2] << std::endl;
	std::cout << "footSensor LH " << footSensor->force[2][0] << " " << footSensor->force[2][1] << " " << footSensor->force[2][2] << std::endl;
	std::cout << "footSensor RH " << footSensor->force[3][0] << " " << footSensor->force[3][1] << " " << footSensor->force[3][2] << std::endl;
}

void RCFController::desJointStatePrint(void) {
	std::cout << "desired tau LF " << des_tau[0] << " " << des_tau[1] << " " << des_tau[2] << std::endl;
	std::cout << "desired tau RF " << des_tau[3] << " " << des_tau[4] << " " << des_tau[5] << std::endl;
	std::cout << "desired tau LH " << des_tau[6] << " " << des_tau[7] << " " << des_tau[8] << std::endl;
	std::cout << "desired tau RH " << des_tau[9] << " " << des_tau[10] << " " << des_tau[11] << std::endl;
	std::cout << "des_q LF " << des_q[0] << " " << des_q[1] << " " << des_q[2] << std::endl;
	std::cout << "des_q RF " << des_q[3] << " " << des_q[4] << " " << des_q[5] << std::endl;
	std::cout << "des_q LH " << des_q[6] << " " << des_q[7] << " " << des_q[8] << std::endl;
	std::cout << "des_q RH " << des_q[9] << " " << des_q[10] << " " << des_q[11] << std::endl;
}

void RCFController::jointErrorsPrint(void) {
	std::cout << "Pos Error: HAA  /  HFE  /  KFE "<< std::endl;
	std::cout << "  LF: " << des_q[0] - q[0] << "  " << des_q[1] - q[1] << "  " << des_q[2] - q[2] << std::endl;
	std::cout << "  RF: " << des_q[3] - q[3] << "  " << des_q[4] - q[4] << "  " << des_q[5] - q[5] << std::endl;
	std::cout << "  LH: " << des_q[6] - q[6] << "  " << des_q[7] - q[7] << "  " << des_q[8] - q[8] << std::endl;
	std::cout << "  RH: " << des_q[9] - q[9] << "  " << des_q[10] - q[10] << "  " << des_q[11] - q[11] << std::endl << std::endl;
}

void RCFController::jointStatePrint(void) {
	std::cout << "tau LF " << tau[0] << " " << tau[1] << " " << tau[2] << std::endl;
	std::cout << "tau RF " << tau[3] << " " << tau[4] << " " << tau[5] << std::endl;
	std::cout << "tau LH " << tau[6] << " " << tau[7] << " " << tau[8] << std::endl;
	std::cout << "tau RH " << tau[9] << " " << tau[10] << " " << tau[11] << std::endl;
	std::cout << "q LF " << q[0] << " " << q[1] << " " << q[2] << std::endl;
	std::cout << "q RF " << q[3] << " " << q[4] << " " << q[5] << std::endl;
	std::cout << "q LH " << q[6] << " " << q[7] << " " << q[8] << std::endl;
	std::cout << "q RH " << q[9] << " " << q[10] << " " << q[11] << std::endl;
	std::cout << "qd LF " << qd[0] << " " << qd[1] << " " << qd[2] << std::endl;
	std::cout << "qd RF " << qd[3] << " " << qd[4] << " " << qd[5] << std::endl;
	std::cout << "qd LH " << qd[6] << " " << qd[7] << " " << qd[8] << std::endl;
	std::cout << "qd RH " << qd[9] << " " << qd[10] << " " << qd[11] << std::endl;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************///
//*********************************************************************************************///
void RCFController::toggleJump()
{
	if(attitudeFlag) {
		jumpFlag = true;
		jumpOff = false;
		initJumpFlag = true;
		endJumpFlag = true;
		jumpForceNew = userJumpForce;
		setPID(300.0, 0.0, 0.0, 25.0, 0.0, 0.0);
		PIDManager.usePIDManager = false;
		//std::cout << "Init Pos: " << misc_sensor[B_Z] + 1 << std::endl;
		std::cout << "Init Pos: " << "Not Defined" << std::endl;
		std::cout << "trunkHeight: " << trunkHeight << std::endl;
		//scd2();
	}
	else {
		std::cout << "No Jump!!! Trunk Controller is off! : " << std::endl;
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************///
//*********************************************************************************************///
void RCFController::goSquat() {

	P0_Local_HF_New[dog::LF](rbd::Z) = - squatHeight;
	P0_Local_HF_New[dog::RF](rbd::Z) = - squatHeight;
	P0_Local_HF_New[dog::LH](rbd::Z) = - squatHeight;
	P0_Local_HF_New[dog::RH](rbd::Z) = - squatHeight;
	originChangingRate =  filterResponse::slowC;
	//Kd_trunk_x_new = 100.0;
	//Kd_trunk_y_new = 100.0;
	//Kd_trunk_z_new = 0.0;
	//Kd_trunk_yaw_new = 100.0;
	//body_weight_new = 80.0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************///
//*********************************************************************************************///
void RCFController::goBelly() {

    P0_Local_HF_New[dog::LF](rbd::X) = + 0.47;
    P0_Local_HF_New[dog::RF](rbd::X) = + 0.47;
    P0_Local_HF_New[dog::LH](rbd::X) = - 0.47;
    P0_Local_HF_New[dog::RH](rbd::X) = - 0.47;

    P0_Local_HF_New[dog::LF](rbd::Z) = - 0.10;
    P0_Local_HF_New[dog::RF](rbd::Z) = - 0.10;
    P0_Local_HF_New[dog::LH](rbd::Z) = - 0.10;
    P0_Local_HF_New[dog::RH](rbd::Z) = - 0.10;
    originChangingRate =  filterResponse::slowC;
    //Kd_trunk_x_new = 100.0;
    //Kd_trunk_y_new = 100.0;
    //Kd_trunk_z_new = 0.0;
    //Kd_trunk_yaw_new = 100.0;
    //body_weight_new = 80.0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************///
//*********************************************************************************************///
void RCFController::startSelfRighting() {

    //if((bs->getRoll_W() < 1.22) && (bs->getRoll_W() > -1.22) &&
      //      (bs->getPitch_W() < 1.22) && (bs->getPitch_W() > -1.22)) {
	  //  std::cout << "No self-righting!!! Robot is good!" << std::endl;
	//}
	//else {
		//if(!executeSelfRighting && fallDetectionFlag){
		if(!executeSelfRightingFlag) {
			executeSelfRightingFlag = true;
			std::cout << "Starting self-righting!!!" << std::endl;
		}
	//}
}

void RCFController::startLayDown() {
	//if(!executeSelfRighting && fallDetectionFlag){
	if(!executeLayDownFlag) {
		executeLayDownFlag = true;
		fallDetectionFlag = true;
		std::cout << "Starting SitDown!!!" << std::endl;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************///
//*********************************************************************************************///
void RCFController::resetOriginsForTrotting() {

	P0_Local_HF_New = P0_Local_HF_Default; //From config file

	originChangingRate =  filterResponse::slowC;
}

void RCFController::toggleForcedStance(){

	forcedStanceFlag = !forcedStanceFlag;

	if(forcedStanceFlag)
	{
		std::cout << "Forced Stance Flag is on!" << std::endl;
	}else{
		std::cout << "Forced Stance Flag is off!"<< std::endl;
	}

}

void RCFController::setPID() {

	double aux_gain;
	int aux_int;
	static double HAA_gain;
	static double HFE_gain;
	static double KFE_gain;
    static double HAA_gain_d;
	static double HFE_gain_d;
	static double KFE_gain_d;
	static double res_freq = 0;

	newline::getDouble("HAA_gain [0, 1000]", zeroGainTh[dog::LF_HAA], aux_gain);
	if ((aux_gain < 0) || (aux_gain > 1000)){
		printf("unsafe value\n");
	}else{
		HAA_gain = aux_gain;
	}

	newline::getDouble("HFE_gain [0, 1000]", zeroGainTh[dog::LF_HFE], aux_gain);
	if ((aux_gain < 0) || (aux_gain > 1000)){
		printf("unsafe value\n");
	}else{
		HFE_gain = aux_gain;
	}

	newline::getDouble("KFE_gain [0, 1500]", zeroGainTh[dog::LF_KFE], aux_gain);
	if ((aux_gain < 0) || (aux_gain > 1500)){
		printf("unsafe value\n");
	}else{
		KFE_gain = aux_gain;
	}

	newline::getDouble("HAA_gain_d [0, 500]", zeroGainThd[dog::LF_HAA], aux_gain);
	if ((aux_gain < 0) || (aux_gain > 500)){
		printf("unsafe value\n");
	}else{
		HAA_gain_d = aux_gain;
	}

	newline::getDouble("HFE_gain_d [0, 1000]", zeroGainThd[dog::LF_HFE], aux_gain);
	if ((aux_gain < 0) || (aux_gain > 1000)){
		printf("unsafe value\n");
	}else{
		HFE_gain_d = aux_gain;
	}

	newline::getDouble("KFE_gain_d [0, 1000]", zeroGainThd[dog::LF_KFE], aux_gain);
	if ((aux_gain < 0) || (aux_gain > 1000)){
		printf("unsafe value\n");
	}else{
		KFE_gain_d = aux_gain;
	}


	newline::getDouble("KFE_gain_int [0, 1]", zeroGainInt[dog::LF_KFE], aux_gain);
	if ((aux_gain < 0) || (aux_gain > 1)){
		printf("unsafe value\n");
	}else{
		zeroGainInt[dog::LF_KFE] = aux_gain;
		zeroGainInt[dog::RF_KFE] = aux_gain;
		zeroGainInt[dog::LH_KFE] = aux_gain;
		zeroGainInt[dog::RH_KFE] = aux_gain;
	}


	newline::getDouble("Inertial Comp Gain", inertialCompGain, aux_gain);
	if ((aux_gain < 0.0) || (aux_gain > 1.0)){
		printf("unsafe value\n");
	}else{
		inertialCompGain = aux_gain;
	}

	zeroGainTh[dog::LF_HAA] = HAA_gain;
	zeroGainTh[dog::LF_HFE] = HFE_gain;
	zeroGainTh[dog::LF_KFE] = KFE_gain;
	zeroGainThd[dog::LF_HAA] = HAA_gain_d;
	zeroGainThd[dog::LF_HFE] = HFE_gain_d;
	zeroGainThd[dog::LF_KFE] = KFE_gain_d;

	zeroGainTh[dog::RF_HAA] = HAA_gain;
	zeroGainTh[dog::RF_HFE] = HFE_gain;
	zeroGainTh[dog::RF_KFE] = KFE_gain;
	zeroGainThd[dog::RF_HAA] = HAA_gain_d;
	zeroGainThd[dog::RF_HFE] = HFE_gain_d;
	zeroGainThd[dog::RF_KFE] = KFE_gain_d;


	zeroGainTh[dog::LH_HAA] = HAA_gain;
	zeroGainTh[dog::LH_HFE] = HFE_gain;
	zeroGainTh[dog::LH_KFE] = KFE_gain;
    zeroGainThd[dog::LH_HAA] = HAA_gain_d;
	zeroGainThd[dog::LH_HFE] = HFE_gain_d;
	zeroGainThd[dog::LH_KFE] = KFE_gain_d;

	zeroGainTh[dog::RH_HAA] = HAA_gain;
	zeroGainTh[dog::RH_HFE] = HFE_gain;
	zeroGainTh[dog::RH_KFE] = KFE_gain;
	zeroGainThd[dog::RH_HAA] = HAA_gain_d;
	zeroGainThd[dog::RH_HFE] = HFE_gain_d;
	zeroGainThd[dog::RH_KFE] = KFE_gain_d;


    PIDManager.setDefaultJointPIDGains(PIDManager.LF_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LF_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LF_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RF_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RF_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RF_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LH_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LH_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LH_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RH_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RH_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RH_KFE, KFE_gain, 0.0, KFE_gain_d);



	std::cout << "Resonance freq.: ";
	res_freq = 1.0 / 2.0 / 3.1415 * sqrt(20 * KFE_gain / 30)
			* sqrt(
					1
							- 20 * zeroGainThd[dog::LF_KFE] / 2
									/ sqrt(30 * 20 * KFE_gain));
	//<< std::endl;

	std::cout << res_freq << std::endl;

	std::cout << "Resonance stance period: " << 0.5 / res_freq << std::endl;

	std::cout << "Resonance freq 2.: ";
	res_freq = 1.0 / 2.0 / 3.1415 * sqrt(20 * KFE_gain / 30)
			* sqrt(
					1
							- 20 * zeroGainThd[dog::LF_KFE] * 20
									* zeroGainThd[dog::LF_KFE] / 4 / 30 / 20
									/ KFE_gain);
	//<< std::endl;

	std::cout << res_freq << std::endl;
	std::cout << "Resonance stance period 2: " << 0.5 / res_freq << std::endl;

	std::cout << "CPG stance period: " << stepLength / forwVel << std::endl;

	setPID_Flag = true;
}

///*********************************************************************************************///

void RCFController::RCF_Setup() {

    int aux_int;

    newline::getInt("Use constrained-ID [0 or 1]", constrainedInvDynFlag, aux_int);
    if ((aux_int == 0 || aux_int == 1)) {
        if (aux_int == 0 && (aux_int != constrainedInvDynFlag)) {
            constrainedInvDynFlag = false;
            printf("Constrained inverse dynamics turned off!!! \n");
        }
        if (aux_int == 1 && (aux_int != constrainedInvDynFlag)) {
            constrainedInvDynFlag = true;
            printf("Using constrained inverse dynamics!!! \n");
        }
    }

    newline::getInt("Only gravity comp. from inv. dyn. [0 or 1]", invDynOnlyForGravCompFlag, aux_int);
    if ((aux_int == 0 || aux_int == 1) && (aux_int != invDynOnlyForGravCompFlag)) {
        if (aux_int == 0 && (aux_int != invDynOnlyForGravCompFlag)) {
            invDynOnlyForGravCompFlag = false;
            printf("Only gravity compensation turned off!!! \n");
        }
        if (aux_int == 1 && (aux_int != invDynOnlyForGravCompFlag)) {
            invDynOnlyForGravCompFlag = true;
            printf("Only gravity compensation from inverse dynamics!!! \n");
        }
    }

    newline::getInt("Forced leg stance status [0 or 1]", forcedStanceFlag, aux_int);
    if ((aux_int == 0 || aux_int == 1) && (aux_int != forcedStanceFlag)) {
        if (aux_int == 0 && (aux_int != forcedStanceFlag)) {
            forcedStanceFlag = false;
            printf("Force leg stance status turned off!!! \n");
        }
        if (aux_int == 1 && (aux_int != forcedStanceFlag)) {
            forcedStanceFlag = true;
            printf("Using forced leg stance status!!! \n");
        }
    }

    newline::getInt("Use leg stance holder [0 or 1]", useStanceHolderFlag, aux_int);
    if ((aux_int == 0 || aux_int == 1) && (aux_int != useStanceHolderFlag)) {
        if (aux_int == 0 && (aux_int != useStanceHolderFlag)) {
            useStanceHolderFlag = false;
            printf("Leg stance holder turned OFF!!! \n");
        }
        if (aux_int == 1 && (aux_int != useStanceHolderFlag)) {
            useStanceHolderFlag = true;
            printf("Leg stance holder turned ON!!! \n");
        }
    }

    newline::getInt("RCF State Estimator for Trunk Vel. [0 or 1]", useRCF_StateEstimator, aux_int);
    if ((aux_int == 0 || aux_int == 1) && (aux_int != useRCF_StateEstimator)) {
        if (aux_int == 0 && (aux_int != useRCF_StateEstimator)) {
            useRCF_StateEstimator = false;
            printf("Using DLS framework state estimator!!! \n");
        }
        if (aux_int == 1 && (aux_int != useRCF_StateEstimator)) {
            useRCF_StateEstimator = true;
            printf("Using RCF Kin+IMU state estimator!!! \n");
        }
    }

    newline::getInt("Use collision detection [0 or 1]", useCollisionDetectionFlag, aux_int);
    if ((aux_int == 0 || aux_int == 1) && (aux_int != useCollisionDetectionFlag)) {
        if (aux_int == 0 && (aux_int != useCollisionDetectionFlag)) {
            useCollisionDetectionFlag = false;
            printf("Detection of foot frontal collision OFF!!! \n");
        }
        if (aux_int == 1 && (aux_int != useCollisionDetectionFlag)) {
            useCollisionDetectionFlag = true;
            printf("Detection of foot frontal collision ON!!! \n");
        }
    }

    newline::getInt("Use step reaction [0 or 1]", useStepReactionFlag, aux_int);
    if ((aux_int == 0 || aux_int == 1) && (aux_int != useStepReactionFlag)) {
        if (aux_int == 0 && (aux_int != useStepReactionFlag)) {
            useStepReactionFlag = false;
            printf("Step reaction turned OFF!!! \n");
        }
        if (aux_int == 1 && (aux_int != useStepReactionFlag)) {
            useStepReactionFlag = true;
            printf("Step reaction turned ON!!! \n");
        }
    }




}

///*********************************************************************************************///

void RCFController::charICTPCallback(const std_msgs::CharConstPtr& msg){
	std::string user_text(1, msg->data);
	std::cout << "ICTP Read " << user_text << std::endl;
	ICTPLogic(user_text);
}


///*********************************************************************************************///
void RCFController::resetRobotDesiredPosition(){

    desRobotPositionWF << bs->getPosition_W()[rbd::X], bs->getPosition_W()[rbd::Y], 0.0;
    desRobotYaw = bs->getYaw_W();

    std::cout << "Initial robot position is: " << desRobotPositionWF.transpose() << std::endl;
    std::cout << "Initial robot yaw is: " << desRobotYaw << std::endl;
}

///*********************************************************************************************///
void RCFController::togglePositionControl(){

    if(positionControlFlag){
        positionControlFlag = false;
        KpTrunkYawNew = 0;
        KpTrunkX_New = 0;
        KpTrunkY_New = 0;
        std::cout << "Robot position control turned OFF!!!" << std::endl << std::endl;
    } else {
        if(attitudeFlag){

            positionControlFlag = true;
            KpTrunkYawNew = KpTrunkYawDefault;
            KpTrunkX_New = KpTrunkX_Default;
            KpTrunkY_New = KpTrunkY_Default;
            std::cout << "Robot position control turned ON!!!" << std::endl << std::endl;

            //Reset desired robot position
            resetRobotDesiredPosition();

        } else{
            std::cout << "Position control not allowed! Trunk controller must be ON!!!" << std::endl << std::endl;
        }
    }

}

///*********************************************************************************************///
void RCFController::toggleJoystick(){

	useJoystickFlag = !useJoystickFlag;
	if(useJoystickFlag){
		std::cout << "Using Joystick!!!" << std::endl;
	}
	else {
		std::cout << "Joystick turned OFF!!!" << std::endl;	
	}
}


///*********************************************************************************************///
void RCFController::showTrunkGains(){

    std::cout << std::endl;
    std::cout << "Kp Roll: " << KpTrunkRollDefault << "  Kd Roll: " << KdTrunkRollDefault << std::endl;
    std::cout << "Kp Pitch: " << KpTrunkPitchDefault << "  Kd Pitch: " << KdTrunkPitchDefault << std::endl;
    std::cout << "Kp Yaw: " << KpTrunkYawDefault << "  Kd Yaw: " << KdTrunkYawDefault << std::endl << std::endl;

    std::cout << "Kp X: " << KpTrunkX_Default << "  Kd X: " << KdTrunkX_Default << std::endl;
    std::cout << "Kp Y: " << KpTrunkY_Default << "  Kd Y: " << KdTrunkY_Default << std::endl;
    std::cout << "Kp Z: " << KpTrunkZ_Default << "  Kd Z: " << KdTrunkZ_Default << std::endl << std::endl;

    std::cout << "Robot weight: " << bodyWeightDefault << std::endl << std::endl;

}


///*********************************************************************************************///
void RCFController::setImpedanceRoughTerrain() {

    double HAA_gain, HFE_gain, KFE_gain;
    double HAA_gain_d, HFE_gain_d, KFE_gain_d;

    //Setting joint impedance related to the leg swing phase
    HAA_gain = 300;
    HFE_gain = 300;
    KFE_gain = 300;
    HAA_gain_d = 12;
    HFE_gain_d = 8;
    KFE_gain_d = 6;

    zeroGainTh[dog::LF_HAA] = HAA_gain;
    zeroGainTh[dog::LF_HFE] = HFE_gain;
    zeroGainTh[dog::LF_KFE] = KFE_gain;
    zeroGainThd[dog::LF_HAA] = HAA_gain_d;
    zeroGainThd[dog::LF_HFE] = HFE_gain_d;
    zeroGainThd[dog::LF_KFE] = KFE_gain_d;

    zeroGainTh[dog::RF_HAA] = HAA_gain;
    zeroGainTh[dog::RF_HFE] = HFE_gain;
    zeroGainTh[dog::RF_KFE] = KFE_gain;
    zeroGainThd[dog::RF_HAA] = HAA_gain_d;
    zeroGainThd[dog::RF_HFE] = HFE_gain_d;
    zeroGainThd[dog::RF_KFE] = KFE_gain_d;


    zeroGainTh[dog::LH_HAA] = HAA_gain;
    zeroGainTh[dog::LH_HFE] = HFE_gain;
    zeroGainTh[dog::LH_KFE] = KFE_gain;
    zeroGainThd[dog::LH_HAA] = HAA_gain_d;
    zeroGainThd[dog::LH_HFE] = HFE_gain_d;
    zeroGainThd[dog::LH_KFE] = KFE_gain_d;

    zeroGainTh[dog::RH_HAA] = HAA_gain;
    zeroGainTh[dog::RH_HFE] = HFE_gain;
    zeroGainTh[dog::RH_KFE] = KFE_gain;
    zeroGainThd[dog::RH_HAA] = HAA_gain_d;
    zeroGainThd[dog::RH_HFE] = HFE_gain_d;
    zeroGainThd[dog::RH_KFE] = KFE_gain_d;

    //Set flag to Inform that gains at motor servo must be updated
    setPID_Flag = true;


    //Updating PID Manager
    PIDManager.setDefaultJointPIDGains(PIDManager.LF_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LF_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LF_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RF_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RF_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RF_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LH_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LH_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LH_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RH_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RH_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RH_KFE, KFE_gain, 0.0, KFE_gain_d);

    PIDManager.setStanceJointPIDGains(PIDManager.LF_HAA, 100, 0.0, 10);
    PIDManager.setStanceJointPIDGains(PIDManager.LF_HFE, 25, 0.0, 8);
    PIDManager.setStanceJointPIDGains(PIDManager.LF_KFE, 25, 0.0, 6);
    PIDManager.setStanceJointPIDGains(PIDManager.RF_HAA, 100, 0.0, 10);
    PIDManager.setStanceJointPIDGains(PIDManager.RF_HFE, 25, 0.0, 8);
    PIDManager.setStanceJointPIDGains(PIDManager.RF_KFE, 25, 0.0, 6);
    PIDManager.setStanceJointPIDGains(PIDManager.LH_HAA, 100, 0.0, 10);
    PIDManager.setStanceJointPIDGains(PIDManager.LH_HFE, 25, 0.0, 8);
    PIDManager.setStanceJointPIDGains(PIDManager.LH_KFE, 25, 0.0, 6);
    PIDManager.setStanceJointPIDGains(PIDManager.RH_HAA, 100, 0.0, 10);
    PIDManager.setStanceJointPIDGains(PIDManager.RH_HFE, 25, 0.0, 8);
    PIDManager.setStanceJointPIDGains(PIDManager.RH_KFE, 25, 0.0, 6);

}


///*********************************************************************************************///
void RCFController::resetImpedance() {

    double HAA_gain, HFE_gain, KFE_gain;
    double HAA_gain_d, HFE_gain_d, KFE_gain_d;

    //Setting joint impedance related to the leg swing phase
    HAA_gain = 300;
    HFE_gain = 250;
    KFE_gain = 250;
    HAA_gain_d = 12;
    HFE_gain_d = 8;
    KFE_gain_d = 6;

    zeroGainTh[dog::LF_HAA] = HAA_gain;
    zeroGainTh[dog::LF_HFE] = HFE_gain;
    zeroGainTh[dog::LF_KFE] = KFE_gain;
    zeroGainThd[dog::LF_HAA] = HAA_gain_d;
    zeroGainThd[dog::LF_HFE] = HFE_gain_d;
    zeroGainThd[dog::LF_KFE] = KFE_gain_d;

    zeroGainTh[dog::RF_HAA] = HAA_gain;
    zeroGainTh[dog::RF_HFE] = HFE_gain;
    zeroGainTh[dog::RF_KFE] = KFE_gain;
    zeroGainThd[dog::RF_HAA] = HAA_gain_d;
    zeroGainThd[dog::RF_HFE] = HFE_gain_d;
    zeroGainThd[dog::RF_KFE] = KFE_gain_d;


    zeroGainTh[dog::LH_HAA] = HAA_gain;
    zeroGainTh[dog::LH_HFE] = HFE_gain;
    zeroGainTh[dog::LH_KFE] = KFE_gain;
    zeroGainThd[dog::LH_HAA] = HAA_gain_d;
    zeroGainThd[dog::LH_HFE] = HFE_gain_d;
    zeroGainThd[dog::LH_KFE] = KFE_gain_d;

    zeroGainTh[dog::RH_HAA] = HAA_gain;
    zeroGainTh[dog::RH_HFE] = HFE_gain;
    zeroGainTh[dog::RH_KFE] = KFE_gain;
    zeroGainThd[dog::RH_HAA] = HAA_gain_d;
    zeroGainThd[dog::RH_HFE] = HFE_gain_d;
    zeroGainThd[dog::RH_KFE] = KFE_gain_d;

    //Set flag to Inform that gains at motor servo must be updated
    setPID_Flag = true;


    //Updating PID Manager
    PIDManager.setDefaultJointPIDGains(PIDManager.LF_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LF_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LF_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RF_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RF_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RF_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LH_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LH_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LH_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RH_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RH_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RH_KFE, KFE_gain, 0.0, KFE_gain_d);

    //Setting PID Manager for impedances related to the leg stance phase
    HAA_gain = 300;     HFE_gain = 250;    KFE_gain = 250;
    HAA_gain_d = 12;    HFE_gain_d = 8;    KFE_gain_d = 6;

    PIDManager.setStanceJointPIDGains(PIDManager.LF_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setStanceJointPIDGains(PIDManager.LF_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setStanceJointPIDGains(PIDManager.LF_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setStanceJointPIDGains(PIDManager.RF_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setStanceJointPIDGains(PIDManager.RF_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setStanceJointPIDGains(PIDManager.RF_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setStanceJointPIDGains(PIDManager.LH_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setStanceJointPIDGains(PIDManager.LH_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setStanceJointPIDGains(PIDManager.LH_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setStanceJointPIDGains(PIDManager.RH_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setStanceJointPIDGains(PIDManager.RH_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setStanceJointPIDGains(PIDManager.RH_KFE, KFE_gain, 0.0, KFE_gain_d);

}

}//@namespace dls_controller
