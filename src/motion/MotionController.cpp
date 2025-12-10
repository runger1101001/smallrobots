#include "./MotionController.h"

namespace SmallRobots {

    //------------------------------------------------------------------------------------------------------------------
    //  MotionController
    //------------------------------------------------------------------------------------------------------------------

    MotionController::MotionController(DifferentialKinematics& drive, Odometry& _odometryCtrl) 
        : kinematics(drive), odometryCtrl(_odometryCtrl), point_and_shoot(drive), dubin_controller(drive)
    {
    };

    MotionController::~MotionController() {
    };

    void MotionController::setup(){

    };
    void MotionController::run(){

        curPose = odometryCtrl.getCurPose();

        if (currentMode == DUBINS_PATH) {
            dubin_controller.run(curPose);
        } else if (currentMode == POINT_AND_SHOOT) {
            point_and_shoot.run(curPose);
        }

    };


    // void MotionController::addPoseToPath(Pose p)
    // {
    //     path.push_back(p); //at the end of path, continue current path

    // };
    // void MotionController::addPoseToPathAndGoThereFirst(Pose p)
    // {
    //     path.push_back(p); //at the end of path, continue current path
    //     curPathIndex= path.size()-1;
    // };
    // void MotionController::setPoseToReplacePath(Pose p)
    // {
    //     path.clear(); //stop continueing currentpath 
    //     path.push_back(p);
    //     curPathIndex=0;

    // };

    // void MotionController::addPoseListToPath(std::vector<Pose> poses){
    //     path.insert(path.end(), poses.begin(), poses.end());
        
    // };
    // void MotionController::addPoseListToPathAndGoThereFirst(std::vector<Pose> poses){
    //     path.insert(path.end(), poses.begin(), poses.end());
    //     curPathIndex= path.size()-1;
    // };
    // void MotionController::setPoseListToReplacePath(std::vector<Pose> poses){
    //     path = poses;
    //     curPathIndex=0;
    // };

    // void MotionController::deletePath()
    // {
    //     path.clear();
    // };

    // void MotionController::setLoopPath(){
    //     pathBehaviour = LOOP ; //PATHBEHAVIOURS
    // };
    // void MotionController::setPausePath(){
    //     pathBehaviour = PAUSE ; //PATHBEHAVIOURS
    // };
    // void MotionController::setContinuePath(){
    //     pathBehaviour =  CONTINUE; //PATHBEHAVIOURS
    // };
    // void MotionController::setEndPath(){
    //     pathBehaviour = END ; //PATHBEHAVIOURS
    // };
    // void MotionController::setRestartPath(){
    //     pathBehaviour = RESTART; //PATHBEHAVIOURS
    // };
    


    void MotionController::stop(){
        // kinematics.setSpeed(0,0);
        kinematics.stop(); //not sure what is better, but when the motors are disabled, the robot moves further than intended, maybe wait a bit after stop, then disable
    };

    void MotionController::enableMotors(){
        kinematics.start();
    };

    void MotionController::setRobotVelocity( float _vRobot){ //in mm/s

        if (_vRobot >=0){ //-1 resets to the last set speed
            vRobot =_vRobot;
            
            if (currentMode == DUBINS_PATH){
                dubin_controller.setRobotVelocityAndActivate(_vRobot); //active
                point_and_shoot.setRobotVelocity(_vRobot); //passive
            } else if (currentMode == POINT_AND_SHOOT){
                point_and_shoot.setRobotVelocityAndActivate(_vRobot); //active
                dubin_controller.setRobotVelocity(_vRobot); //passive
            }
        }
    };

    void MotionController::setDubinsPathRadius(float _radius){ //this is only for the next move command from dubin path
        dubin_controller.setPathRadius(_radius);//in mm
    }; 

    // void MotionController::setPathBevahiourType(int type){
    //     pathBehaviour = type;
    // };
    void MotionController::setDesiredVelocitySmoothedPointAndShoot(float vx, float vy, float speed, float smoothing_factor, 
                                                                   float significant_heading_change_rad){
        enableMotors();
        point_and_shoot.setDesiredVelocitySmoothed(vx, vy, speed, smoothing_factor, significant_heading_change_rad);
        currentMode = POINT_AND_SHOOT;
    }

    void MotionController::setDesiredVelocityPointAndShoot(float vx, float vy, float speed) {
        enableMotors();
        point_and_shoot.setDesiredVelocity(vx, vy, speed);
        currentMode = POINT_AND_SHOOT;
    }
    void MotionController::setTargetPointAndShoot(const Pose& target, float speed, AngleUnit angleUnit) {
        enableMotors();
        
        // Convert target angle from degrees to radians
        Pose target_radians = target;
        if (angleUnit == AngleUnit::DEGREES) {
            target_radians.angle = target.angle * M_PI / 180.0f;
        }
        point_and_shoot.setTarget(target_radians, curPose, speed);
        currentMode = POINT_AND_SHOOT;
    }
    void MotionController::setCurvatureFactorPnS(float factor) {
        point_and_shoot.setCurvatureFactor(factor);
    }
    
    void MotionController::setTargetDubinsPath(const Pose& target, float speed, AngleUnit angleUnit){
        enableMotors();
        dubin_controller.setRobotVelocity(speed);
        dubin_controller.activateNewTarget();
        
        // Convert target angle from degrees to radians
        Pose target_radians = target;
        if (angleUnit == AngleUnit::DEGREES) {
            target_radians.angle = target.angle * M_PI / 180.0f;
        }

        dubin_controller.setPoseToReplacePath(target_radians); //addPoseToPathAndGoThereFirst(target);
        currentMode = DUBINS_PATH;
    };
    



}; // namespace SmallRobots