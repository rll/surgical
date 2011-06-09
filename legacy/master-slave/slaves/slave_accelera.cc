#include "slave_accelera.h"


Slave_accelera::Slave_accelera(int id, boost::shared_ptr<Controller> controller, boost::shared_ptr<System> sys) : Slave_mark1(id, controller, sys) {

    // No longer [Mot0;Mot1;Mot2;Mot3] = M*[pitch;roll;tension;gross].
    //
    // Now [A;B;C;D] = M*[pitch;tilt;grip;gross].
    // A = yaw 1
    // B = pitch
    // C = gross
    // D = yaw 2
#define BASEHOME true
#define WRISTHOME false
/* ///////////////////////////////////// 
//OPEN_FACTOR used to be +- 0.35
//M(1,1) = M(2,1) = 0.821 previously
if(id==1){
#define M(m, n) (*_wrist_to_motor_transform)[m][n]
    M(0,0) =  -1.005; M(0,1) =  0.000; M(0,2) =  0.000;                 M(0,3) = 0.000;
    M(1,0) =  -0.680; M(1,1) =  1.345; M(1,2) =  SLAVE_1_OPEN_FACTOR;   M(1,3) = 0.000;
    M(2,0) =  -0.680; M(2,1) =  1.345; M(2,2) = -1*SLAVE_1_OPEN_FACTOR; M(2,3) = 0.000;
    M(3,0) =  -0.000; M(3,1) =  0.000; M(3,2) =  0.000;                 M(3,3) = -0.673;
#undef M
}
else{
//NOTE: Was set at +- 0.47 for big end affector
//M(1,1) = M(2,1) = 0.821 previously
#define M(m, n) (*_wrist_to_motor_transform)[m][n]
    M(0,0) =  -1.005; M(0,1) =  0.000; M(0,2) =  0.000;                 M(0,3) = 0.000;
    M(1,0) =  -0.680; M(1,1) =  1.345; M(1,2) =  SLAVE_2_OPEN_FACTOR;   M(1,3) = 0.000;
    M(2,0) =  -0.680; M(2,1) =  1.345; M(2,2) = -1*SLAVE_2_OPEN_FACTOR; M(2,3) = 0.000;
    M(3,0) =  -0.000; M(3,1) =  0.000; M(3,2) =  0.000;                 M(3,3) = -0.673;
#undef M
}
    // limits
    pitch_min           = 0;
    pitch_max           = M_PI;
    roll_min            = -M_PI/2;
    roll_max            = 3*M_PI/2;
    /* FIX FOR BROKEN YAW */
/* ///////////////////////////////////// 
    if(id==2){
        roll_min = M_PI/2 - 2;
        roll_max = M_PI/2 + 2;
    }
    gross_min           = 0;
    gross_max           = 2*M_PI;
    arm_min             = 82.55; //STEPHEN CHANGED TO 77.55
    
    cone_sphere_bound   = 233.26;
    arm_max             = 285.75;
    radius_height_ratio = 129.30/233.26;
    ///* Adjusted to make fit...probably not good */
    //if(id == 1)
    //    radius_height_ratio += 0.65;
    //else
    //    radius_height_ratio += 0.46;
    /* End adjustment */
/* ///////////////////////////////////// 
    default_tension     = 0.0;

    // kinematic constants
    stick_length        = 349.25;
    if(id==1) stick_length        += 33.5; //FIXME ADDED TO INCLUDE WRIST -- REMOVE 
    else if(id==2) stick_length += 35.5;
//////////////////////////////////////// */
    // abstraction stuff
    _geometry = new Accelera_geometry(id-1);
    _wrist_to_motor_transform = _geometry->_wrist_to_motor_transform;
    pitch_min = _geometry->pitch_min;
    pitch_max = _geometry->pitch_max;
    roll_min = _geometry->roll_min;
    roll_max = _geometry->roll_max;
    gross_min = _geometry->gross_min;
    gross_max = _geometry->gross_max;
    arm_min = _geometry->arm_min;
    cone_sphere_bound = _geometry->cone_sphere_bound;
    arm_max = _geometry->arm_max;
    radius_height_ratio = _geometry->radius_height_ratio;
    default_tension = _geometry->default_tension;
    stick_length = _geometry->stick_length;
    

    // homing points (slave's elbow frame)
    home_points[0][iPITCH]   = pitch_max;
    home_points[0][iROLL]    = roll_max+M_PI/8;
    home_points[0][iGROSS]   = 0;
    home_points[0][iX]       = -125.757759;
    home_points[0][iY]       = -193.135278;
    home_points[0][iZ]       =  308.940616; // - 135.86
    home_points[0][iGRIP]    = GRIP_ON;

    home_points[1][iPITCH]   = pitch_max;
    home_points[1][iROLL]    = roll_max+M_PI/8;
    home_points[1][iGROSS]   = 0;
    home_points[1][iX]       = -125.757759;
    home_points[1][iY]       = -193.135278;
    home_points[1][iZ]       =  308.940616;
    home_points[1][iGRIP]    = GRIP_OFF;

}

Slave_accelera::~Slave_accelera() { }

double* Slave_accelera::point_to_motors(double in[num_dof], double out[num_actuators], bool use_elbow_coor, bool use_kdl) { 
    return _geometry->point_to_motors(in, out, use_elbow_coor, use_kdl);
}
double* Slave_accelera::motors_to_point(double motor_pos[num_actuators], double out[num_dof], bool use_base_coor, bool use_kdl) {
    return _geometry->motors_to_point(motor_pos, out, use_base_coor, use_kdl);
}
double* Slave_accelera::elbow_to_base(double in[3], double out[3]) {
    return _geometry->elbow_to_base(in, out);
}
double* Slave_accelera::base_to_motor(double in[3], double out[3]) { 
    return _geometry->base_to_motor(in, out);
}
double* Slave_accelera::elbow_tip_swap(double loc[3], double out[3]) { 
    return _geometry->elbow_tip_swap(loc, out);
}
double* Slave_accelera::base_to_elbow(double in[3], double out[3]) { 
    return _geometry->base_to_elbow(in, out);
}
double* Slave_accelera::motor_to_base(double in[3], double out[3]) { 
    return _geometry->motor_to_base(in, out);
}
double* Slave_accelera::wrist_to_tip(double in[num_dof], double out[num_actuators]) { 
    return _geometry->wrist_to_tip(in, out);
}
double* Slave_accelera::point_to_tip(double in[num_dof], double out[num_dof], double* guess, bool called) { 
    return _geometry->point_to_tip(in,out,guess,called);
}
double* Slave_accelera::tip_to_point(double in[num_dof], double out[num_dof]) {
    return _geometry->tip_to_point(in,out);
}
double* Slave_accelera::CheckerboardToRobot(double in[num_dof], double out[num_dof]) {
    return _geometry->CheckerboardToRobot(in,out);
}
/* ////////////////////////////////////// 
double* Slave_accelera::point_to_motors(double in[num_dof], double out[num_actuators], bool use_elbow_coor) {

    // WRIST KINEMATICS
    // Apply the model to get the motor positions
    CwMtx::CWTMatrix<> wrist_goal(4, 1);
    wrist_goal[0][0] = in[iPITCH];   // pitch
    wrist_goal[1][0] = in[iTILT];    // tilt
    wrist_goal[2][0] = in[iGRIP];    // grip
    wrist_goal[3][0] = in[iGROSS];   // gross

    wrist_goal = (*_wrist_to_motor_transform) * wrist_goal;
    for(int i=0; i<4; i++){
        out[i] = wrist_goal[i][0];
    }
    out[7] = 0; // no more pneumatics, 'H' is unused

    // BASE KINEMATICS
    double v[3];
    v[0] = in[iX];
    v[1] = in[iY];
    v[2] = in[iZ];
    if(!use_elbow_coor) {
        // Move from tip to elbow
        elbow_tip_swap(v, v);
    }

    elbow_to_base(v, v);
    base_to_motor(v, v);
    for(int i=0;i<3;i++) {
        out[i+4] = v[i];
    }
    
    if(PRINT_WHEN_USING_ELBOW_COOR){
        if(use_elbow_coor)
            debugprintf("Using Elbow Coordinates!\n");
    }
    return out;
}

double* Slave_accelera::motors_to_point(double motor_pos[num_actuators], double out[num_dof], bool use_elbow_coor) {
    int i;

    // WRIST KINEMATICS
    CwMtx::CWTSquareMatrix<> M(*_wrist_to_motor_transform);
    CwMtx::CWTMatrix<> wrist_motors(4, 1);
    
    // Obtain wrist setpoints
    M.makeInverse();
    for(i=0;i<4;i++)
        wrist_motors[i][0] = motor_pos[i];
    CwMtx::CWTVector<> wrist_pos = M * wrist_motors;

    // BASE KINEMATICS
    double base[3] = {motor_pos[4], motor_pos[5], motor_pos[6] };
    motor_to_base(base, base);
    base_to_elbow(base, base);

    // Move from elbow to tip?
    if (!use_elbow_coor) {
        elbow_tip_swap(base, base);
    }

    // pack result & return
    out[iPITCH]  = wrist_pos[0];      // pitch
    out[iTILT]   = wrist_pos[1];      // tilt
    out[iGROSS]  = wrist_pos[3];      // gross
    out[iX]      = base[0];           // x
    out[iY]      = base[1];           // y
    out[iZ]      = base[2];           // z
    out[iGRIP]   = wrist_pos[2];      // grip
    
    if(PRINT_WHEN_USING_ELBOW_COOR){
        if(use_elbow_coor)
            debugprintf("Using Elbow Coordinates!\n");
    }
    
    return out;
}
///////////////////////////////// */
void Slave_accelera::home() {

    int i;
    double straightpoints[num_slaves][num_dof];
    straightpoints[0][iPITCH] = M_PI/2;
    straightpoints[0][iTILT] = M_PI/2;
    straightpoints[0][iGROSS] = M_PI;
    straightpoints[0][iGRIP] = GRIP_ON;
       
    straightpoints[1][iPITCH] = M_PI/2;
    straightpoints[1][iTILT] = M_PI/2;
    straightpoints[1][iGROSS] = M_PI;
    straightpoints[1][iGRIP] = GRIP_ON;
    
    double* home_pos = home_points[_id-1];
    std::cout << "Homing process for Slave " << _id << std::endl;
    
    //////////////////////////////////Base Homing/////////////////////////////////////////////////////
    if(BASEHOME){
        std::cout << "Base homing..." << std::endl;
        
        home_base();
        init_setpoints_base(home_pos[iX], home_pos[iY], home_pos[iZ], true); //initially true
    
        std::cout << "Congratulations, Slave " << _id << " is homed." << std::endl;
        std::cout << "Place the slave in the trocar and press enter to begin." << std::endl;
        getchar();
    }
    
//////////////////////////////////Wrist Homing////////////////////////////////////////////////////
    if(WRISTHOME){
        std::cout << "Wrist Homing..." << std::endl;
    
        double voltage = 0.0;
        while (voltage < MAX_VOLTAGE_WRIST) {
            voltage += 0.02;
            for(i=0;i<4;i++) {
                _sys->queue_voltage(i, voltage);
            }
            _sys->apply_voltages();
            usleep(500000);
        }
        _sys->reset_all();
    
        init_setpoints_arm(home_pos[iPITCH], home_pos[iTILT], home_pos[iGROSS], home_pos[iGRIP]);
        init_setpoints_base(0,0,-100); // Random point so we can use move_to
        
        usleep(1000000);
        return;
        std::cout << "Wrist homing complete.  Straightening wrist...";
        
        double time = 4000000;
        double setpoints[num_dof];
        
        current_pose(setpoints);
        current_pose(straightpoints[_id-1]);
        
        double pitch_step = -1*(setpoints[iPITCH]- straightpoints[_id-1][iPITCH])/(time/_interval);
        double tilt_step  = -1*(setpoints[iTILT] - straightpoints[_id-1][iTILT])/(time/_interval);
        double gross_step = -1*(setpoints[iGROSS]- straightpoints[_id-1][iGROSS])/(time/_interval);
        double grip_step  = -1*(setpoints[iGRIP] - straightpoints[_id-1][iGRIP])/(time/_interval);
        while(time > 0) {
            setpoints[iPITCH] += pitch_step;
            setpoints[iTILT]  += tilt_step;
            setpoints[iGROSS] += gross_step;
            setpoints[iGRIP]  += grip_step;
            move_to(setpoints);
            usleep(_interval);
            time -= _interval;
        }
        time = 1000000;
        while(time > 0) {
            move_to(straightpoints[_id-1]);
            usleep(_interval);
            time -= _interval;
        }
    
        std::cout << "Is the wrist straight? (CTRL+C if not, else enter)" << std::endl;
        getchar();
    }
    else{
        init_setpoints_arm(straightpoints[_id-1][iPITCH], straightpoints[_id-1][iTILT],straightpoints[_id-1][iGROSS],straightpoints[_id-1][iGRIP]);
    } 

}

void Slave_accelera :: reset_wrists(){
    double straightpoints[num_slaves][num_dof];
    straightpoints[0][iPITCH] = M_PI/2;
    straightpoints[0][iTILT] = M_PI/2;
    straightpoints[0][iGROSS] = M_PI;
    straightpoints[0][iGRIP] = GRIP_ON;
    straightpoints[1][iPITCH] = M_PI/2;
    straightpoints[1][iTILT] = M_PI/2;
    straightpoints[1][iGROSS] = M_PI;
    straightpoints[1][iGRIP] = GRIP_ON;
    init_setpoints_arm(straightpoints[_id-1][iPITCH], straightpoints[_id-1][iTILT],straightpoints[_id-1][iGROSS],straightpoints[_id-1][iGRIP]);
}
