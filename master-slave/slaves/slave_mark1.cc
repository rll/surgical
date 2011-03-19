#include "slave_mark1.h"

using std::cout;
using std::endl;

// Actual definitions
Slave_mark1::Slave_mark1(int id, boost::shared_ptr<Controller> controller, boost::shared_ptr<System> s) : Slave(id, controller, s) {
    _wrist_to_motor_transform= new CwMtx::CWTSquareMatrix<>(4);
    _elbow_to_base_transform = new CwMtx::CWTSquareMatrix<>(4);
    _base_to_motor_transform = new CwMtx::CWTSquareMatrix<>(4);

    /* An experimentally found matrix which, when multiplied with a vector of pitch,
     * roll, tension, gross, x, y, z, and grip, returns the motor positions for all
     * actuators 0 through 7,  CCW roll is positive.  */
#define M(m, n) (*_wrist_to_motor_transform)[m][n]
    M(0,0) =  0.800; M(0,1) =  0.000; M(0,2) = 1.0; M(0,3) = 0.0;
    M(1,0) = -0.548; M(1,1) =  0.429; M(1,2) = 1.0; M(1,3) = 0.0;
    M(2,0) = -0.404; M(2,1) = -0.296; M(2,2) = 1.0; M(2,3) = 0.0;
    M(3,0) =  0.000; M(3,1) =  0.000; M(3,2) = 0.0; M(3,3) = 1.0;
#undef M

    /* Homogeneous transformation from elbow frame to base frame */
#define M(m, n) (*_elbow_to_base_transform)[m][n]
    (*_elbow_to_base_transform).fill(0);
    M(0,0) =  0.000; M(0,1) =  1.000; M(0,2) = 0.000; M(0,3) = 550.86;
    M(1,0) = -1.000; M(1,1) =  0.000; M(1,2) = 0.000; M(1,3) =   0.00;
    M(2,0) =  0.000; M(2,1) =  0.000; M(2,2) = 1.000; M(2,3) = 219.00;
    M(3,0) =  0.000; M(3,1) =  0.000; M(3,2) = 0.000; M(3,3) =   1.00;
#undef M

    /* Homogeneous transformation from base frame to base motor frame */
#define M(m, n) (*_base_to_motor_transform)[m][n]
    (*_base_to_motor_transform).fill(0);
    M(0,0) =  0.000; M(0,1) =  1.000; M(0,2) = 0.000; M(0,3) = 212.725;
    M(1,0) = -1.000; M(1,1) =  0.000; M(1,2) = 0.000; M(1,3) = -50.800;
    M(2,0) =  0.000; M(2,1) =  0.000; M(2,2) = 1.000; M(2,3) =  15.875;
    M(3,0) =  0.000; M(3,1) =  0.000; M(3,2) = 0.000; M(3,3) =   1.000;
#undef M

    // limits
    pitch_min           = 0;
    pitch_max           = M_PI;
    roll_min            = -3*M_PI/2;
    roll_max            = 0;
    gross_min           = -23*M_PI/6;
    gross_max           = 0;
    arm_min             = 90;
    cone_sphere_bound   = 184.252779;
    arm_max             = 205;
    radius_height_ratio = 89.8660853/184.252779;
    default_tension     = 0.0;

    // For base kinematics
    cross_length        = 431.8;
    stick_length        = 363.2;
    boom_length         = 548.4;
    boom_altitude       = 520.7;
    link_length_with_pinion = 182.9;
    link_altitude       = 179.1;

    // Homing positions
    home_points[0][iPITCH]   = 0.0;
    home_points[0][iROLL]    = 0.0;
    home_points[0][iGROSS]   = 0.0;
    home_points[0][iX]       = -113.369364;
    home_points[0][iY]       = -197.282175;
    home_points[0][iZ]       =  292.764160;
    home_points[0][iGRIP]    = 0.0;

    home_points[1][iPITCH]   = 0.0;
    home_points[1][iROLL]    = 0.0;
    home_points[1][iGROSS]   = 0.0;
    home_points[1][iX]       = -113.369364;
    home_points[1][iY]       = -197.282175;
    home_points[1][iZ]       =  292.764160;
    home_points[1][iGRIP]    = 0.0;
}

Slave_mark1::~Slave_mark1() {
    delete _wrist_to_motor_transform;
    delete _elbow_to_base_transform;
    delete _base_to_motor_transform;
}


void Slave_mark1::init_setpoints_arm(double pitch, double roll, double gross, double grip){
    // Calculate where the motors are right now based on given tip position
    double args[num_dof];
    double encoder_offsets[num_actuators];
    args[iPITCH] = pitch; args[iROLL] = roll; args[iGROSS] = gross; args[iGRIP] = grip;
    point_to_motors(args, encoder_offsets, false, false);

    // Set the encoders to believe where they are at
    for(int i=0; i < 4; i++) {
        double tension_offset;
        if (i < 3)      // Gross rotation has no tension offset
            tension_offset = default_tension;
        else
            tension_offset = 0.0;
    	init_setpoint(i,encoder_offsets[i] - tension_offset);
    }
}

void Slave_mark1::init_setpoints_base(double x, double y, double z, bool use_elbow_coor){
    // Calculate where the motors are right now based on the current tip position
    double args[num_dof]; double encoder_offsets[num_actuators];
    args[iX] = x; args[iY] = y; args[iZ] = z;
    point_to_motors(args, encoder_offsets, use_elbow_coor, false);

    // Set the encoders to believe where they are at
    for(int i=4; i < 7; i++) {
    	init_setpoint(i, encoder_offsets[i]);
    }
}

void Slave_mark1::init_setpoints(std::map<int,double> point_value, bool use_base_coor){
    bool no_wrist = false, no_base = false;
    std::vector<int> pts;
    std::vector<int>::iterator index;
    
    // Fucking C++ maps don't have key membership testing, so I have to do it manually
    pts.push_back(iPITCH); pts.push_back(iROLL); pts.push_back(iGROSS); pts.push_back(iGRIP);
    for(index = pts.begin(); index != pts.end(); index++) {
        if (point_value.count(*index) == 0) {
            no_wrist = true;
            break;
        }
    }
    
    if(!no_wrist) {
        init_setpoints_arm(point_value[iPITCH], point_value[iROLL], point_value[iGROSS], point_value[iGRIP]);
    }
    
    pts.clear();
    pts.push_back(iX); pts.push_back(iY); pts.push_back(iZ);
    for(index = pts.begin(); index != pts.end(); index++) {
        if (point_value.count(*index) == 0) {
            no_base = true;
            break;
        }
    }
    if(!no_base) {
        init_setpoints_base(point_value[iX], point_value[iY], point_value[iZ], use_base_coor);
    }
}

void Slave_mark1::home() {
    std::cout << "Homing base..." << std::endl;

    // Straighten tip first to avoid cable breaking
    //double outV = 1.2, curV = 0.0, weight = 1.0;
    //double waitTime = 500000;
    //printf("\tStraightening wrist first...\n");
    //while(curV < outV) {
    //	curV += outV / 10.0;
    //    _sys->queue_voltage(0,curV);
    //    _sys->apply_voltages();
    //	usleep(waitTime * weight);
    //	weight *= 1.2;
    //}
    //_sys->reset_all();

    home_base();

    double* home_pos = home_points[_id-1];
    init_setpoints_base(home_pos[iX], home_pos[iY], home_pos[iZ], true);

    std::cout << "Base homing complete.  Move arm into trocar for arm homing." << std::endl;
    getchar();

    std::cout << "Homing arm..." << std::endl;
    std::cout << "Press enter when in homed wrist position." << std::endl;
    getchar();
    //home_wrist();
    init_setpoints_arm(home_pos[iPITCH], home_pos[iROLL], home_pos[iGROSS], home_pos[iGRIP]);

    std::cout << "Slave is homed.  Please move the slave into working position, then press any key." << std::endl;
    getchar();
}

void Slave_mark1::home_wrist() {
    double actuator_ratios[4] = { 1,1,1,1.3 }; 
    std::vector<int> toHome;
    std::vector<int>::iterator current_index;
    double outV = 1.0, curV, weight;
    unsigned long waitTime = 500000;
    unsigned short i;

    // specify homing order
    toHome.push_back(3);	//gross rotation
    toHome.push_back(0);	//pitch
    toHome.push_back(2);	//roll 2 (ccw)
    toHome.push_back(1);	//roll 1 (cw)

    // home each motor with ramping velocity control.  It is
    // assumed that a positive velocity will tighten the wire
    // of each motor.
    current_index = toHome.begin();
    while(current_index != toHome.end()) {
    	std::cout << "\thoming channel " << *current_index << std::endl;
    	curV = 0.0;
    	weight = 1.0;
    	while(curV < outV) {
    		curV += outV / 10.0;
            _sys->queue_voltage(*current_index, curV*actuator_ratios[*current_index]);
            _sys->apply_voltages();
    		usleep(waitTime * weight);
    		weight *= 1.2;
    	}
        _sys->reset_all();
        
    	++current_index;
    }
}

void Slave_mark1::home_base() {
    printf("\tActual base homing...\n");
    // Actual base homing
    double voltage = 0;
    time_t diff = 0;
    
    while (voltage < 3.0) {
        voltage += 0.125;
        _sys->queue_voltage(4, voltage);
        _sys->queue_voltage(5,-voltage);
        _sys->queue_voltage(6,-voltage);
        _sys->apply_voltages();
        usleep(500000);
    }
    /*
    while (true){
        voltage = -2;
        _sys->queue_voltage(6,voltage);
        _sys->apply_voltages();
        usleep(500000);
    }
    */
    _sys->reset_all();
}

double* Slave_mark1::nearest_legal_point(double in[num_dof], double result[num_dof]) {
    double pitch    = in[iPITCH];
    double roll     = in[iROLL];
    double gross    = in[iGROSS];
    double x        = in[iX];
    double y        = in[iY];
    double z        = in[iZ];
    double grip     = in[iGRIP];
    
    if(MAKE_ALL_POINTS_LEGAL){
        for(int i = 0; i < num_dof; i++){
            result[i] = in[i];
        }
        return result;
    }

    if (! (pitch >= pitch_min && pitch <= pitch_max)) {
        //printf("Pitch out of bounds.  Reverting to safe limits\n");
       if (pitch < pitch_min)
            pitch = pitch_min;
        else
            pitch = pitch_max;
    }
    
    if (! ( roll  >= roll_min && roll  <= roll_max ) ) {
        //printf("Roll out of bounds.  Reverting to safe limits\n");
        if (roll < roll_min)
            roll  = roll_min;
        else
            roll  = roll_max;
    }
    /*
    if (! ( gross >= gross_min && gross <= gross_max )) {
        printf("Gross out of bounds.  Reverting to safe limits\n");
        if(gross < gross_min)
            gross = gross_min;
        else
            gross = gross_max;
    }
    */
    /*
    
    if (grip > 1.5 || grip < -0.5) {
        if(grip < 0)
            grip = -0.5;
        else
            grip = 1.5;
    }
    */

    CwMtx::CWTVector<> P(3);                         // The goal point
    P[0] = x; P[1] = y; P[2] = -1*fabs(z);
    //printf("Requested point: (%f, %f, %f)\n", P(0), P(1), P(2));

    // Too short?
    if (P.norm() < arm_min) {
        if(PRINT_SETPOINT_VS_NEAREST_LEGAL)
            debugprintf("SETPOINT < ARM_MIN by %f\n",fabs(P.norm() - arm_min));
        P.makeUnit();
        P = P*arm_min;
    }
    
    // Cone bounding?
    CwMtx::CWTVector<> C(3);
    C.fill(0.0); C[2] = P[2];           // The center of the cone at P's height
    double radius = radius_height_ratio*fabs(P[2]);  
                                        // radius at point's height
    CwMtx::CWTVector<> D = P - C;       // Flatten such that the point and the circle portion
                                        // of the cone are @ z=0
    CwMtx::CWTVector<> zero(3); zero.fill(0.0);
    if (D.norm() > radius) {
        if(PRINT_SETPOINT_VS_NEAREST_LEGAL)
            debugprintf("D.norm()) > RADIUS by %f\n",fabs(D.norm() - radius));
        D.makeUnit();  // Identify the closest point in x,y at this height
        D = D*radius;         
        //printf("Requested point: (%f, %f, %f)\n", P(0), P(1), P(2));
        //printf("Nearest point at this height: (%f, %f)\n", D(0), D(1));
        D[2] = P[2];    // D is now a vector on the side of the cone
        D.makeUnit();
        double t = -1*( ((zero-P)*(D-zero)) * (1/D.norm()));
        P = D*t;
    }
    
    // Too long?
    if (P.norm() > arm_max) {
        if(PRINT_SETPOINT_VS_NEAREST_LEGAL)
            debugprintf("SETPOINT > ARM_MAX by %f\n",fabs(P.norm() - arm_max));
        P.makeUnit();
        P = P*arm_max;
    }
    //printf("Nearest point period: (%f, %f, %f)\n", P(0), P(1), P(2));
    result[iPITCH] = pitch; result[iROLL] = roll; result[iGROSS] = gross;
    result[iX] =  P[0]; result[iY] = P[1]; result[iZ] =  P[2];
    result[iGRIP] = grip;

    if(PRINT_SETPOINT_VS_NEAREST_LEGAL){
        if(in[0] != result[0] || in[1] != result[1] || in[2] != result[2] || in[3] != result[3] || in[4] != result[4] || in[5] != result[5] || in[6] != result[6]){
            debugprintf("Tried to move to: [0] %f, [1] %f, [2] %f, [3] %f, [4] %f, [5] %f, [6] %f\n",in[0],in[1],in[2],in[3],in[4],in[5],in[6]);
            debugprintf("Nearest legal is: [0] %f, [1] %f, [2] %f, [3] %f, [4] %f, [5] %f, [6] %f\n", result[0],result[1],result[2],result[3],result[4],result[5],result[6]);
            double motor_legal[num_actuators];
            point_to_motors(result,motor_legal);
            printf("Nearest motors :");
            for (int channel = 0; channel < num_actuators; channel++) {
                printf("[%d] %f", channel, motor_legal[channel]);
            }
            printf("\n");
        }
    }
    return result;
}

double* Slave_mark1::point_to_motors(double in[num_dof], double out[num_actuators], bool use_elbow_coor,
bool use_kdl){
    printf("in slave_mark1\n\n\n");
    // WRIST KINEMATICS
    // Apply the model to get the motor positions
    CwMtx::CWTMatrix<> wrist_goal(4, 1);
    wrist_goal[0][0] = in[iPITCH];
    wrist_goal[1][0] = in[iROLL];
    wrist_goal[2][0] = default_tension;
    wrist_goal[3][0] = in[iGROSS];

    wrist_goal = (*_wrist_to_motor_transform) * wrist_goal;
    for(int i=0; i<4; i++){
        out[i] = wrist_goal[i][0];
    }
    out[7] = in[iGRIP]; // grip is unchanged

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

double* Slave_mark1::elbow_to_base(double in[3], double out[3]) {

    CwMtx::CWTMatrix<>          in_vect(4,1);
    for(int i=0;i<3;i++) {
        in_vect[i][0] = in[i];
    }
    in_vect[3][0] = 1;
    in_vect = (*_elbow_to_base_transform)*in_vect;

    //printf("x:%f, y:%f, z:%f\n", in_vect[0][0], in_vect[1][0], in_vect[2][0]);

    double line_fx = in_vect[0][0];
    double line_fy = in_vect[1][0];
    double line_fz = in_vect[2][0];
    double line_af = sqrt( line_fx*line_fx + line_fy*line_fy + line_fz*line_fz );

    double angle_afz = acos(line_fz/line_af);
    double angle_afb = angle_law_of_cos(line_af, boom_altitude, boom_length);
    double angle_zfb = angle_afb - angle_afz;
    double angle_abf = angle_law_of_cos(boom_length, boom_altitude, line_af);
    double angle_fde = M_PI-angle_abf;

    double line_fe = side_law_of_cos(link_altitude, link_length_with_pinion, angle_fde);
        //Length from origin to E

    double angle_dfe = angle_law_of_cos(link_altitude, line_fe, link_length_with_pinion);
    double angle_vert = angle_dfe + angle_zfb;
        //Angle from z down to point E

    double angle_rotation = atan2(line_fy, line_fx) + M_PI;
        //Angle from x to E

    //printf("angle vert:%f, angle_rot:%f, line_fe:%f\n", angle_vert*180/M_PI, angle_rotation*180/M_PI, line_fe);
    out[0] = line_fe*sin(angle_vert)*cos(angle_rotation);
    out[1] = line_fe*sin(angle_vert)*sin(angle_rotation);
    out[2] = line_fe*cos(angle_vert);
    return out;
}

double* Slave_mark1::base_to_motor(double in[3], double out[3]) {
    double line_4to6 = 425.45;
    double line_5to6 = 450.85;
    
    CwMtx::CWTMatrix<>          in_vect(4,1);

    for(int i=0;i<3;i++) {
        in_vect[i][0] = in[i];
    }
    in_vect[3][0] = 1;

    in_vect = (*_base_to_motor_transform)*in_vect;

    double x = in_vect[0][0];
    double y = in_vect[1][0];
    double z = in_vect[2][0];

    // screw lengths for motors 4,5,6 respectively
    //printf("x:%f, y:%f, z:%f\n", x, y, z);
    out[2] = sqrt(sq(x) + sq(y) + sq(z));
    out[1] = sqrt(sq(x-line_4to6/2) + sq(y-sqrt(sq(line_5to6)-sq(line_4to6/2))) + sq(z));
    out[0] = sqrt(sq(x-line_4to6) + sq(y) + sq(z));
    return out;
}

double* Slave_mark1::motors_to_point(double motor_pos[num_actuators], double out[num_dof], bool use_elbow_coor, bool use_kdl) {
    int i;
    printf("in slave_mark1\n\n\n");
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
    out[iROLL]   = wrist_pos[1];      // roll
    out[iGROSS]  = wrist_pos[3];      // gross
    out[iX]      = base[0];           // x
    out[iY]      = base[1];           // y
    out[iZ]      = base[2];           // z
    out[iGRIP]   = motor_pos[7];      // grip
    
    if(PRINT_WHEN_USING_ELBOW_COOR){
        if(use_elbow_coor)
            debugprintf("Using Elbow Coordinates!\n");
    }
    
    return out;
}

double* Slave_mark1::base_to_elbow(double in[3], double out[3]) {
    double x = in[0],y = in[1],z = in[2];

    double line_ef      = sqrt( sq(x) + sq(y) + sq(z));
    double angle_edf    = angle_law_of_cos(link_altitude, link_length_with_pinion, line_ef);
    double angle_abf    = M_PI - angle_edf;
    double line_af      = side_law_of_cos(boom_length, boom_altitude, angle_abf);
    double angle_dfe    = angle_law_of_cos(link_altitude, line_ef, link_length_with_pinion);
    double angle_efminusx = atan2(z, fabs(sqrt(sq(x) + sq(y))));      // assumed +z
    double angle_zfb    = M_PI/2 - (angle_dfe+angle_efminusx);

    double angle_afb    = angle_law_of_cos(boom_altitude, line_af, boom_length);
    double angle_vert   = angle_afb - angle_zfb;    // aka angle_afz
    double angle_rotation = atan2(-y,-x);

    double x_out        = line_af*sin(angle_vert)*cos(angle_rotation);
    double y_out        = line_af*sin(angle_vert)*sin(angle_rotation);
    double z_out        = line_af*cos(angle_vert);

    CwMtx::CWTSquareMatrix<> base_to_elbow_transform((*_elbow_to_base_transform));
    CwMtx::CWTMatrix<>          in_vect(4,1);
    base_to_elbow_transform.makeInverse();

    in_vect[0][0] = x_out;
    in_vect[1][0] = y_out;
    in_vect[2][0] = z_out;
    in_vect[3][0] = 1;

    in_vect = base_to_elbow_transform*in_vect;
    for(int i=0;i<3;i++) {
        out[i] = in_vect[i][0];
    }
    return out;
}

double* Slave_mark1::motor_to_base(double in[3], double out[3]) {
    // See Pierra della Francesca's Tetrahedron Formula
    // http://www.mathpages.com/HOME/kmath424.htm

    double a = 425.45, b = 450.85, c = 450.85, d = in[2], e = in[0], f = in[1];
    double A = sq(a), B = sq(b), C = sq(c), D = sq(d), E = sq(e), F = sq(f);

    double volume = sqrt( (A*F*(-A+B+C+D+E-F)+B*E*(A-B+C+D-E+F)+C*D*(A+B-C-D+E+F)-(A+F)*(B+E)*(C+D)/2-(A-F)*(B-E)*(C-D)/2) / 144);
    double angle_ab = angle_law_of_cos(a, b, c);
    double floor_area = 0.5*a*b*sin(angle_ab);  // (1/2)bh
    double z = (volume / floor_area)*3;         // our z coordinate

    double angle_da = angle_law_of_cos(a, d, e);
    double xaxis_to_peak_length = sin(angle_da)*d;
    double x = sqrt(D - sq(xaxis_to_peak_length));
    //printf("D: %f, xaxis_to_peak_length: %f\n", D, sq(xaxis_to_peak_length));

    double y = sqrt(D - sq(z) - sq(x));

    //printf("x:%f, y:%f, z:%f\n", x, y, z);

    // we assume we are always in +x,+y,+z

    CwMtx::CWTSquareMatrix<>    motor_to_base_transform((*_base_to_motor_transform));
    CwMtx::CWTMatrix<>          in_vect(4,1);
    
    motor_to_base_transform.makeInverse();
    in_vect[0][0] = x;
    in_vect[1][0] = y;
    in_vect[2][0] = z;
    in_vect[3][0] = 1;

    in_vect = motor_to_base_transform*in_vect;
    for(int i=0;i<3;i++) {
        out[i] = in_vect[i][0];
    }
    return out;
}

double* Slave_mark1::elbow_tip_swap(double loc[3], double out[3]) {
    CwMtx::CWTVector<> v= CwMtx::CWTVector<>(3);
    v[0] = loc[0]; v[1] = loc[1]; v[2] = loc[2];

    // Note top_stick_length doesn't mean from elbow to trocar
    // necessarily.  Could also mean from trocar to tip.
    double top_stick_length = v.norm();
    double bottom_stick_length = stick_length - top_stick_length;
    CwMtx::CWTMatrix<> u = v * -1 * ( bottom_stick_length / top_stick_length);
    if (top_stick_length == 0) {
        u.fill(0.0);
        u[2][0] = STICK_LENGTH;
    }
    
    for(int i=0;i<3;i++)
        out[i] = u[i][0];
    return out;
}

