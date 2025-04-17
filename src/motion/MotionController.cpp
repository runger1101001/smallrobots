

#include "./MotionController.h"

namespace SmallRobots {

    //------------------------------------------------------------------------------------------------------------------
    //  MotionController
    //------------------------------------------------------------------------------------------------------------------

    MotionController::MotionController(DifferentialKinematics& drive, Odometry& _odometryCtrl) : kinematics(drive), odometryCtrl(_odometryCtrl) {
    };

    MotionController::~MotionController() {
    };

    void MotionController::setup(){

    };
    void MotionController::run(){

        curPose = odometryCtrl.getCurPose();

   
        if (subPathIndex == 0){ //RECEIVE NEW TARGET
            calculateDubinForNextPoseInPath();
            Serial.println ("START SEG 1");
            subPathIndex=1;
            setWheelVelocitiesSeg1();
            
        } else if (subPathIndex==-1){
            //NOTHING HAPPENS
        }
        else
        {
            //CHECK IF ARRIVED 
            if (checkIfArrived()){
                //Serial.println (subPathIndex);
                if (subPathIndex == 1){
                    Serial.println ("FINISHED SEG 1");
                    subPathIndex=2;
                    setWheelVelocitiesSeg2();
                    Serial.println ("START SEG 2");                
                }
                else if (subPathIndex == 2){
                    Serial.println ("FINISHED SEG 2");
                    subPathIndex=3;
                    setWheelVelocitiesSeg3();
                    Serial.println ("START SEG 3");                    
                }
                else if (subPathIndex == 3) {
                    Serial.println ("FINISHED SEG 3");
                    //What now?
                    if (pathBehaviour == LOOP)
                    {
                        loopPath(); //counts up the pathIndex, if reached end, restarts from beginning, returns false if only one Pose in path
                        subPathIndex=0;
                    }
                    else if (pathBehaviour ==END)
                    {
                        stop();
                        subPathIndex=-1;
                    }
                    
                }            
            } 
        }

    };

    void MotionController::activateNewTarget()
    {
        subPathIndex=0;
    };

    void MotionController::addPoseToPath(Pose p)
    {
        path.push_back(p); //at the end of path, continue current path

    };
    void MotionController::addPoseToPathAndGoThereFirst(Pose p)
    {
        path.push_back(p); //at the end of path, continue current path
        curPathIndex= path.size()-1;
    };
    void MotionController::setPoseToReplacePath(Pose p)
    {
        path.clear(); //stop continueing currentpath 
        path.push_back(p);
        curPathIndex=0;

    };

    void MotionController::addPoseListToPath(std::vector<Pose> poses){
        path.insert(path.end(), poses.begin(), poses.end());
        
    };
    void MotionController::addPoseListToPathAndGoThereFirst(std::vector<Pose> poses){
        path.insert(path.end(), poses.begin(), poses.end());
        curPathIndex= path.size()-1;
    };
    void MotionController::setPoseListToReplacePath(std::vector<Pose> poses){
        path = poses;
        curPathIndex=0;
    };

    void MotionController::deletePath()
    {
        path.clear();
    };

    void MotionController::setLoopPath(){
        pathBehaviour = LOOP ; //PATHBEHAVIOURS
    };
    void MotionController::setPausePath(){
        pathBehaviour = PAUSE ; //PATHBEHAVIOURS
    };
    void MotionController::setContinuePath(){
        pathBehaviour =  CONTINUE; //PATHBEHAVIOURS
    };
    void MotionController::setEndPath(){
        pathBehaviour = END ; //PATHBEHAVIOURS
    };
    void MotionController::setRestartPath(){
        pathBehaviour = RESTART; //PATHBEHAVIOURS
    };
    

    void MotionController::calculateDubinForNextPoseInPath() //get next pose in path, calculate dubin path from current pose and target pose
    {
        
        // Serial.println ("MotionController::setTarget()");
        // Serial.println("curPathIndex: " + (String) curPathIndex);
        // Serial.println("path queue length: " + (String) path.size());
        targetPose = path[curPathIndex];
        if (curPathIndex==0 && path.size()<=1){ //add cur pose to path so we can loop
            path.insert(path.begin(), curPose);
        }
        // Serial.println("Current Pose: " + (String)curPose.x + ", " + (String) curPose.y + ", " + (String)degrees( curPose.angle));
        // Serial.println("Target Pose: " + (String)targetPose.x + ", " + (String) targetPose.y + ", " + (String) degrees(targetPose.angle));
        pathPlanner.calculate(curPose, targetPose);
        // Serial.println ("Update path, shortest path: " + pathPlanner.getShortestPathName());

        // Serial.println ("1.) -"+ pathPlanner.arcDirName1+ "- with angle: "+ degrees (pathPlanner.arcAngle1)+ "°, around center: "+ pathPlanner.arcCenter1.x +"," + pathPlanner.arcCenter1.y);
        // if (pathPlanner.arcDirName12.equals ("S")) Serial.println ("2.) -"+ pathPlanner.arcDirName12+ "- with distance: "+ pathPlanner.lineLength+ ", from start: "+ pathPlanner.lineStart.x + ", " +pathPlanner.lineStart.y+ " to end: "+ pathPlanner.lineEnd.x + ", " + pathPlanner.lineEnd.y);
        // else Serial.println ("2.) -"+ pathPlanner.arcDirName12+ "- with angle: "+ degrees (pathPlanner.arcAngle12)+ "°, around center: "+ pathPlanner.arcCenter12.x +"," + pathPlanner.arcCenter12.y);
        // Serial.println ("3.) -"+ pathPlanner.arcDirName2+ "- with angle: "+ degrees (pathPlanner.arcAngle2)+ "°, around center: "+ pathPlanner.arcCenter2.x +"," + pathPlanner.arcCenter2.y);

        ICC = pathPlanner.arcCenter1;
        //Serial.println ("ICC: " + (String) ICC.x +  " , " + (String) ICC.y);
        curV = Vector (curPose.x, curPose.y);
        // Serial.println("curV: " +  (String) curV.x + " , " + (String) curV.y);

        R = distance(curV, ICC);
        // Serial.println("R: " + (String) R);

        //rest values for next straight move
        if (pathPlanner.arcDirName12.equals ("S")) {
            curDistance= 0;
            lastDistance = pathPlanner.lineLength *2;
            //Serial.println("SET LAST DISTANCE to "+ (String) lastDistance);
        }

    };

    void MotionController::setWheelVelocitiesSeg1() //ARC left or right with minRad
    {
        //Serial.println ("Set wheel velocities segment 1(3) of path.");

        curDirName = pathPlanner.arcDirName1;

        if (pathPlanner.arcDirName1.equals ("L")) {
          kinematics.turnLeftForward(vRobot,R); //kinematics.move(vRobot, R); 
          targetAngle = pathPlanner.arcAngle1 + curPose.angle ;
        } else if (pathPlanner.arcDirName1.equals ("R")){
          kinematics.turnRightForward(vRobot,R); //kinematics.move(vRobot, -R); 
          targetAngle = -pathPlanner.arcAngle1 + curPose.angle ;
        }
        //Serial.println ("targetAngle: "+ String (degrees(targetAngle))+ " °");
    };
    void MotionController::setWheelVelocitiesSeg2() //STRAIGHT or left or right ARC
    {

        //Serial.println ("Set wheel velocities segment 2(3) of path.");

        curDirName = pathPlanner.arcDirName12;

        if (pathPlanner.arcDirName12.equals ("S")) {

            // Serial.println ("2.) -"+ pathPlanner.arcDirName12+ "- with distance: "+ pathPlanner.lineLength+ "+ from start: "+ pathPlanner.lineStart.x + ", " +  pathPlanner.lineStart.y + " to end: "+ pathPlanner.lineEnd.x + ", " + pathPlanner.lineEnd.y);
            targetPose = Pose() ;
            targetPose.x = pathPlanner.lineEnd.x;
            targetPose.y = pathPlanner.lineEnd.y;
            targetPose.angle = curPose.angle;
            // GO Straight
            kinematics.move(vRobot);

            //rest values for next straight move
      
            curDistance= 0;
            lastDistance = pathPlanner.lineLength *2;
            // Serial.println("SET LAST DISTANCE to "+ (String) lastDistance);
    

          } else
          { //R or L

            // Serial.println ("2.) -"+ pathPlanner.arcDirName12+ "- with angle: "+ degrees (pathPlanner.arcAngle12)+ "°, around center: "+ pathPlanner.arcCenter12.x + ", " + pathPlanner.arcCenter12.y);

            //Serial.println("current robot angle:  "+ String(degrees (curPose.angle))+ " °");

            ICC = pathPlanner.arcCenter12; //OR ? in theory the same: Vector(curPose.x - R * sin(curPose.angle), curPose.y + R* cos(curPose.angle) );
            R = pathPlanner.turnRadius;//OR ? in theory the same:  distance(Vector (curPose.x, curPose.y), ICC);
            // curV = Vector (curPose.x, curPose.y);
            // R = distance(curV, ICC);

            if (pathPlanner.arcDirName12.equals ("L")) {
                kinematics.turnLeftForward(vRobot,R); //kinematics.move(vRobot, R); 
                targetAngle = pathPlanner.arcAngle12 + curPose.angle ;
            } else if (pathPlanner.arcDirName12.equals ("R")) {
                kinematics.turnRightForward(vRobot,R); //kinematics.move(vRobot, -R); 
                targetAngle = -pathPlanner.arcAngle12 + curPose.angle ;
            }
          }

    };
    void MotionController::setWheelVelocitiesSeg3() //ARC left or right
    {

        // Serial.println ("Set wheel velocities segment 3(3) of path.");

        // Serial.println ("3.) -"+ pathPlanner.arcDirName2+ "- with angle: "+ degrees (pathPlanner.arcAngle2)+ "°, around center: "+ pathPlanner.arcCenter2.x + ", " + pathPlanner.arcCenter2.y);
        // Serial.println("current robot angle:  "+ String(degrees (curPose.angle))+ " °");

        ICC = pathPlanner.arcCenter2;//OR ? in theory the same: Vector(curPose.x - R * sin(curPose.angle), curPose.y + R* cos(curPose.angle) );
        R = pathPlanner.turnRadius;//OR ? in theory the same:  distance(Vector (curPose.x, curPose.y), ICC);
        // curV = Vector (curPose.x, curPose.y);
        // R = distance(curV, ICC);


        curDirName = pathPlanner.arcDirName2;

        if (pathPlanner.arcDirName2.equals ("L")) {
            kinematics.turnLeftForward(vRobot,R); //kinematics.move(vRobot, R); 
            targetAngle = pathPlanner.arcAngle2 + curPose.angle ;
        } else if (pathPlanner.arcDirName2.equals ("R")){
            kinematics.turnRightForward(vRobot,R); //kinematics.move(vRobot, -R); 
            targetAngle = -pathPlanner.arcAngle2 + curPose.angle ;
        }

        //Serial.println ("targetAngle: " + String (degrees(targetAngle)) + " °");

    };

    void MotionController::stop(){
        // kinematics.setSpeed(0,0);
        kinematics.stop(); //not sure what is better, but when the motors are disabled, the robot moves further than intended, maybe wait a bit after stop, then disable
    };

    void MotionController::enableMotors(){
        kinematics.start();
    };

    void MotionController::setRobotVelocity( float _vRobot){ //in mm/s
        //make sure that the sign of new velocity is the same as in current movement
        float speed = kinematics.getCurRobotSpeed();
        if (speed >= 0)vRobot = abs( _vRobot );
        else vRobot = - abs(_vRobot);
        vRobot =_vRobot;
        float radius = kinematics.getCurRobotRadius();
        if (subPathIndex !=-1 )kinematics.move(vRobot, radius);
    };

    void MotionController::setPathRadius(float _radius){ //this is only for the next move command from dubin path
        pathPlanner.setPathRadius(_radius);//in mm
    }; 

    void MotionController::setPathBevahiourType(int type){
        pathBehaviour = type;
    };


    bool MotionController::loopPath()
    {
        bool loop= true;
        curPathIndex++;
        if (curPathIndex >= path.size()) curPathIndex= 0; 
        if (path.size() ==1) loop = false;
        return loop;
    };

    bool MotionController::checkIfArrived(){
        bool arrived = false;
        
        if (curDirName.equals("S")){
            Vector curV = Vector(curPose.x, curPose.y);
            Vector tarV = Vector (targetPose.x, targetPose.y);
            //Serial.println("curV:" + String(curV.x) + ", " + (String) curV.y);
            //Serial.println("tarV:" + String(tarV.x) + ", " + (String) tarV.y);
            curDistance = distance( curV, tarV );
            //Serial.println("distance: " + (String) curDistance + "< lastDistance: " + (String) lastDistance);
        } else
        {
            //Serial.println ("targetAngle - curPose.angle :"
            // + String (degrees(targetAngle))+ " ° - "
            // + String (degrees(curPose.angle ))+ " ° = "
            // + String ( degrees (targetAngle - curPose.angle)) + " °"
            // );
            
        }
        if (
          //(curDirName.equals ("L") && targetAngle - curPose.angle  <= arriveAngleDistance )
          (curDirName.equals ("L") && targetAngle - curPose.angle  <= arriveAngleDistance )
          ||
          (curDirName.equals ("R")  && targetAngle -  curPose.angle >= arriveAngleDistance) 
          ||
          //(curDirName.equals ("S")  && curDistance <= arriveDistance) ) //TODO OR CHECK lastDistance > distance, if that changes, then overshoot!, or slowdown when close to target?
          (curDirName.equals ("S")  && curDistance > lastDistance) 
        )
        {
            arrived = true;
            // Serial.println("ARRIVED");
           //reset distances when next pose path is calculated
        }
    
        if (curDirName.equals ("S"))  lastDistance = curDistance;
         
        return arrived;
    };

}; // namespace SmallRobots