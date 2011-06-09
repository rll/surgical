#include "accelera_geometry.h"
#define STABLE_MODE false

Accelera_geometry::Accelera_geometry(int id){
    this->id = id;
    _wrist_to_motor_transform = new CwMtx::CWTSquareMatrix<>(4);
    _elbow_to_base_transform = new CwMtx::CWTSquareMatrix<>(4);
    _base_to_motor_transform  = new CwMtx::CWTSquareMatrix<>(4);
    _gross_transform = new CwMtx::CWTSquareMatrix<>(4);
    _pitch_transform = new CwMtx::CWTSquareMatrix<>(4);
    _tilt_transform  = new CwMtx::CWTSquareMatrix<>(4);
    _tip_pitch_transform = new CwMtx::CWTSquareMatrix<>(4);
    _tip_gross_transform = new CwMtx::CWTSquareMatrix<>(4);

    #define M(m, n) (*_wrist_to_motor_transform)[m][n]
    if (id == 0) {
        M(0,0) =  -1.005; M(0,1) =  0.000; M(0,2) =  0.000;                 M(0,3) = 0.000;
        M(1,0) =  -0.680; M(1,1) =  1.345; M(1,2) =  SLAVE_1_OPEN_FACTOR;   M(1,3) = 0.000;
        M(2,0) =  -0.680; M(2,1) =  1.345; M(2,2) = -1*SLAVE_1_OPEN_FACTOR; M(2,3) = 0.000;
        M(3,0) =  -0.000; M(3,1) =  0.000; M(3,2) =  0.000;                 M(3,3) = -0.673;
    } else {
        M(0,0) =  -1.005; M(0,1) =  0.000; M(0,2) =  0.000;                 M(0,3) = 0.000;
        M(1,0) =  -0.680; M(1,1) =  1.345; M(1,2) =  SLAVE_2_OPEN_FACTOR;   M(1,3) = 0.000;
        M(2,0) =  -0.680; M(2,1) =  1.345; M(2,2) = -1*SLAVE_2_OPEN_FACTOR; M(2,3) = 0.000;
        M(3,0) =  -0.000; M(3,1) =  0.000; M(3,2) =  0.000;                 M(3,3) = -0.673;
    }
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
    
    /* Homogeneous transformation from wrist frame to gross rotated wrist frame */
    #define M(m, n) (*_gross_transform)[m][n]
    //along y
    (*_gross_transform).fill(0);
    M(0,0) =  0.000; M(0,1) =  0.000; M(0,2) = 0.000; M(0,3) = 0.000;
    M(1,0) =  0.000; M(1,1) =  1.000; M(1,2) = 0.000; M(1,3) = 0.000;
    M(2,0) =  0.000; M(2,1) =  0.000; M(2,2) = 0.000; M(2,3) = 0.000; // 0.878
    M(3,0) =  0.000; M(3,1) =  0.000; M(3,2) = 0.000; M(3,3) = 1.000;
    #undef M
    
    /* Homogeneous transformation from wrist frame to pitch transformed wrist frame */
    #define M(m, n) (*_pitch_transform)[m][n]
    //along x
    (*_pitch_transform).fill(0);
    M(0,0) =  1.000; M(0,1) =  0.000; M(0,2) = 0.000; M(0,3) = 0.000;
    M(1,0) =  0.000; M(1,1) =  0.000; M(1,2) = 0.000; M(1,3) = 0.000; // 1.082
    M(2,0) =  0.000; M(2,1) =  0.000; M(2,2) = 0.000; M(2,3) = 0.000;
    M(3,0) =  0.000; M(3,1) =  0.000; M(3,2) = 0.000; M(3,3) = 1.000;
    #undef M
    
    /* Homogeneous transformation from wrist frame to roll transformed wrist frame */
    #define M(m, n) (*_tilt_transform)[m][n]
    // along y
    (*_tilt_transform).fill(0);
    M(0,0) =  0.000; M(0,1) =  0.000; M(0,2) = 0.000; M(0,3) = 0.000; // 1.302
    M(1,0) =  0.000; M(1,1) =  1.000; M(1,2) = 0.000; M(1,3) = 0.000;
    M(2,0) =  0.000; M(2,1) =  0.000; M(2,2) = 0.000; M(2,3) = 0.000;
    M(3,0) =  0.000; M(3,1) =  0.000; M(3,2) = 0.000; M(3,3) = 1.000;
    #undef M
    
    /* Homogeneous transform of wrist frame to stick frame (pitch) */
    #define M(m, n) (*_tip_pitch_transform)[m][n]
    //along x
    (*_tip_pitch_transform).fill(0);
    M(0,0) =  1.000; M(0,1) =  0.000; M(0,2) = 0.000; M(0,3) = 0.000;
    M(1,0) =  0.000; M(1,1) =  0.000; M(1,2) = 0.000; M(1,3) = 0.000;
    M(2,0) =  0.000; M(2,1) =  0.000; M(2,2) = 0.000; M(2,3) = 0.000;
    M(3,0) =  0.000; M(3,1) =  0.000; M(3,2) = 0.000; M(3,3) = 1.000;
    #undef M
    
    /* Homogeneous tranform of wrist frame to stick frame (gross) */
     #define M(m, n) (*_tip_gross_transform)[m][n]
    // along y
    (*_tip_gross_transform).fill(0);
    M(0,0) =  0.000; M(0,1) =  0.000; M(0,2) = 0.000; M(0,3) = 0.000; 
    M(1,0) =  0.000; M(1,1) =  1.000; M(1,2) = 0.000; M(1,3) = 0.000;
    M(2,0) =  0.000; M(2,1) =  0.000; M(2,2) = 0.000; M(2,3) = 0.000;
    M(3,0) =  0.000; M(3,1) =  0.000; M(3,2) = 0.000; M(3,3) = 1.000;
    #undef M
    
    pitch_min              = 0;
    pitch_max              = M_PI;
    roll_min               = -M_PI/2;
    roll_max               = 3*M_PI/2;
    /* FIX FOR BROKEN YAW 
    if(id==1){
        roll_min = M_PI/2 - 2;
        roll_max = M_PI/2 + 2;
    } */
    gross_min               = 0;
    gross_max               = 2*M_PI;
    arm_min                 = 82.55;
    cone_sphere_bound       = 233.26;
    arm_max                 = 285.75;
    radius_height_ratio     = 129.30/233.26;
    default_tension         = 0.0;
    stick_length            = 349.25;
    wrist_gross_length      = 10.72;
    wrist_pitch_length      = 11.71;
    wrist_tilt_length       = 11.53; 
    //stick_length        += 33.5; //FIXME ADDED TO INCLUDE WRIST -- REMOVE 
    //else if(id==2) stick_length += 35.5;
    cross_length            = 431.8;
    boom_length             = 548.4;
    boom_altitude           = 520.7;
    link_length_with_pinion = 182.9;
    link_altitude           = 179.1;
    
    //angles pertaining to tip
    angle_vertical_tip      = 0;
    angle_rotation_tip      = 0;
    
    // setup KDL
    _tip_chain =  new KDL::Chain();
    (*_tip_chain).addSegment(Segment(Joint(Joint::RotY),   Frame(Vector(0.0,0.0,0.0))));
    (*_tip_chain).addSegment(Segment(Joint(Joint::RotX),   Frame(Vector(0.0,0.0,0.0))));
    (*_tip_chain).addSegment(Segment(Joint(Joint::TransY), Frame(Vector(0.0,0.0,0.0)))); // is it tx or ty at 0?
    (*_tip_chain).addSegment(Segment(Joint(Joint::RotY),   Frame(Vector(0.0,wrist_gross_length,0.0))));
    (*_tip_chain).addSegment(Segment(Joint(Joint::RotX),   Frame(Vector(0.0,0.0,-1*wrist_pitch_length))));
    (*_tip_chain).addSegment(Segment(Joint(Joint::RotY),   Frame(Vector(-1*wrist_tilt_length,0.0,0.0))));
    num_joints = (*_tip_chain).getNrOfJoints();
    
    _jnt_min = new KDL::JntArray(num_joints);
    _jnt_max = new KDL::JntArray(num_joints);
    _fksolver = new KDL::ChainFkSolverPos_recursive((*_tip_chain));
    _iksolver = new KDL::ChainIkSolverVel_pinv((*_tip_chain));
    
    (*_jnt_min)(0) = -1*M_PI/2.0;  (*_jnt_max)(0) =     M_PI/2.0;
    (*_jnt_min)(1) =   -1.0*M_PI;  (*_jnt_max)(1) =         M_PI;  
    (*_jnt_min)(2) =           0;  (*_jnt_max)(2) =        350.0;
    (*_jnt_min)(3) =        -2.0;  (*_jnt_max)(3) = 2.0*M_PI+2.0;
    (*_jnt_min)(4) =           0;  (*_jnt_max)(4) =         M_PI;
    (*_jnt_min)(5) = -1*M_PI-1.0;  (*_jnt_max)(5) =          1.0;

    if (id == 0) { 
        kdl_guess[0] = 0.00167;
        kdl_guess[1] = -0.7014;
        kdl_guess[2] = 82.87;
        kdl_guess[3] = M_PI;
        kdl_guess[4] = M_PI/2;
        kdl_guess[5] = -1*M_PI/2;
    }
    else if (id == 1) { 
        kdl_guess[0] = -0.056;
        kdl_guess[1] = -0.794;
        kdl_guess[2] = 78.808;
        kdl_guess[3] = M_PI;
        kdl_guess[4] = M_PI/2;
        kdl_guess[5] = -1*M_PI/2;
    }
}

Accelera_geometry::~Accelera_geometry(){
    //Do nothing;
}

double* Accelera_geometry::CheckerboardToRobot(const double in[num_dof], double out[num_dof]) { 
    
    out[iX] = in[iX];
    out[iY] = in[iY];
    out[iZ] = in[iZ];
 
    Rotation r_dest;
    printf("\nOld Euler ZYX: [%f] [%f] [%f]\n", in[iEulerZ],in[iEulerY],in[iEulerX]);
    r_dest = r_dest.EulerZYX(in[iEulerZ], in[iEulerY], in[iEulerX]);    

    Rotation r_rotate(0.000,  0.000,  1.000,
                      0.000,  1.000,  0.000,
                      -1.000,  0.000, 0.000 );

    /*printf("Rotation Matrix:\n");
    for(int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%f ", r_dest(i,j));
        }
        printf("\n");
    }*/
    
    r_dest =  r_rotate * r_dest ;
    
    /*printf("Rotation Matrix:\n");
    for(int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%f ", r_dest(i,j));
        }
        printf("\n");
    }*/
    
    double z,y,x;
    r_dest.GetEulerZYX(z,y,x);
    out[iEulerZ] = z;
    out[iEulerY] = y;
    out[iEulerX] = x; 
    out[iGRIP] = in[iGRIP];
    
    printf("New Euler ZYX: [%f] [%f] [%f]\n", z,y,x);
    
    return out;
}

int Accelera_geometry::FrameToJnt (const JntArray& q_init, const Frame& p_in, 
                                    JntArray& q_out, int maxiter, double eps) {
    Frame f;
    Twist delta_twist;
    JntArray delta_q(num_joints);                                
    q_out = q_init;
    JntArray best_seen(num_joints); 
    double best_twist_eps = 99999999;

    for (unsigned int i = 0; i <maxiter; i++) { 
        _fksolver->JntToCart(q_out, f);
        delta_twist = diff(f,p_in);

        Vector vel(delta_twist.vel);
        Vector rot(delta_twist.rot);
        for(int x = 0; x < 3; x++) {
            if(vel(x) < 0) { 
                vel(x) = -1*vel(x);
            }
            if(rot(x) < 0) {
                rot(x) = -1*rot(x);
            }
        }
        if(vel(0) < best_twist_eps && vel(1) < best_twist_eps && vel(2) < best_twist_eps &&
           rot(0) < best_twist_eps && rot(1) < best_twist_eps && rot(2) < best_twist_eps) {
            best_seen = q_out; 
            best_twist_eps = 0;
            for(int x = 0; x < 3; x++) {
                if(vel(x) > best_twist_eps) {
                    best_twist_eps = vel(x);    
                }
                if(rot(x) > best_twist_eps) { 
                    best_twist_eps = rot(x);
                }
            }
        }
        if(Equal(delta_twist,Twist::Zero(),eps))
            return 1;
            //return 1;
        _iksolver->CartToJnt(q_out,delta_twist,delta_q);
        /*for (int j = 0; j < num_joints; j++) {
            delta_q(j) = delta_q(j) / 100;
        }*/
        Add(q_out,delta_q,q_out);
        /*printf("\ndelta_q: [%f %f %f %f %f %f]", delta_q(0), delta_q(1), delta_q(2),
                                                 delta_q(3), delta_q(4), delta_q(5));*/
        for (unsigned int j= 0; j < num_joints; j++) { 
            if (j == 3) { 
                int k = (int)(q_out(j)/(2*M_PI));
                q_out(j) = q_out(j) - 2*M_PI*k;
                continue;
            }
            if (q_out(j) < (*_jnt_min)(j)) {
                q_out(j) = (*_jnt_min)(j);
            }
            else if (q_out(j) > (*_jnt_max)(j)) {
                q_out(j) = (*_jnt_max)(j);
            }
        }
    }
    // enforce joint constraints on joints 3,4,5
    q_out = best_seen;
    printf("No converge joints:");
    for(int j = 3; j < num_joints; j++) {
      printf(" [%d] %f", j, q_out(j));
    }
    printf("\n");

    return -1;
}

double* Accelera_geometry::tip_to_point(double in[num_dof], double out[num_dof]) {


    /*double v[3];
    v[0] = in[iX];
    v[1] = in[iY];
    v[2] = in[iZ];
    //printf("\n%f %f %f\n", v[0], v[1], v[2]);
    // Move from tip to elbow to base
    elbow_tip_swap(v,v);
    elbow_to_base(v,v);
    // Calculate angles
    double tip_rot[3] = {v[0], v[1], v[2]};
    double tip_vert[3] = {v[0], v[1], v[2]}; 
    base_to_elbow(tip_vert, tip_vert, true); // sets up vertical angle calculation
    elbow_tip_swap(tip_vert, tip_vert, true); // calculates vertical angle
    base_to_elbow(tip_rot, tip_rot, false, true); // sets up rot angle calculation
    elbow_tip_swap(tip_rot, tip_rot, false, true); // calculates rot angle
    */
    
    // should be passed in wrist_angles, stick_gross, stick_pitch, and stick_length
    
    JntArray joints(num_joints);
    joints(0) = in[iSTICK_GROSS];
    joints(1) = in[iSTICK_PITCH];
    joints(2) = in[iSTICK_LENGTH];
    joints(3) = in[iGROSS];
    joints(4) = in[iPITCH];
    joints(5) = -1 * in[iTILT];

    /*kdl_guess[0] = joints(0);
    kdl_guess[1] = joints(1);
    kdl_guess[2] = joints(2);
    kdl_guess[3] = joints(3);
    kdl_guess[4] = joints(4);
    kdl_guess[5] = joints(5);
    */
    //printf("\njointsforw: %f %f %f %f %f %f", joints(0), joints(1), joints(2), joints(3), joints(4), joints(5));
    /*printf("\n");
    for (int i = 0; i < num_joints; i++) { 
        printf("thinks joint %d = %f ", i, joints(i));
        printf("limits [%f - %f] \n", (*_jnt_min)(i), (*_jnt_max)(i));
    }
    printf("tip_chain joints: %d", (*_tip_chain).getNrOfJoints());
    */
    ChainFkSolverPos_recursive fksolver((*_tip_chain));
    Frame frame; 
    fksolver.JntToCart(joints, frame); 
    out[iX] = frame.p[0];
    out[iY] = frame.p[1]; 
    out[iZ] = frame.p[2];
    /*for (int i = 0; i < 3; i++) { 
       printf("thinks: [%f]\n", frame.p[i]);
    }*/
    printf("Rotation Matrix:\n");
    for(int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%f ", frame.M(i,j));
        }
        printf("\n");
    }
    double z,y,x;
    frame.M.GetEulerZYX(z,y,x);
    //printf("r_frame = [%f %f %f]\n",z,y,x); 
    out[iEulerZ] = z;
    out[iEulerY] = y;
    out[iEulerX] = x; 
    out[iGRIP] = in[iGRIP]; 
    
    return out; 
}

double* Accelera_geometry::point_to_tip(double in[num_dof], double out[num_dof], double* guess, bool called) { 

    ChainFkSolverPos_recursive fksolver((*_tip_chain));
    ChainIkSolverVel_pinv iksolverv((*_tip_chain));
    ChainIkSolverPos_NR_JL iksolverpos((*_tip_chain),(*_jnt_min),(*_jnt_max),fksolver,iksolverv,100000,1e0);
    //ChainIkSolverPos_NR iksolverpos((*_tip_chain),fksolver,iksolverv,100000,5e-1);
    JntArray q(num_joints);
    JntArray q_init(num_joints);
    for (int i = 0; i < num_joints; i++) {
        if (guess) { 
            q_init(i) = guess[i];
        } else { 
            q_init(i) = kdl_guess[i];
        }
    }
    Vector v_dest(in[iX], in[iY], in[iZ]);
    Rotation r_dest;
    r_dest = r_dest.EulerZYX(in[iEulerZ], in[iEulerY], in[iEulerX]);    
    Frame f_dest(r_dest, v_dest);
    //printf("\nv_dest = [%f %f %f]", v_dest[0], v_dest[1], v_dest[2]);
    double z, y, x;
    r_dest.GetEulerZYX(z,y,x);
    //printf("\nr_dest = [%f %f %f]",z,y,x);
    
    //int ret = iksolverpos.CartToJnt(q_init, f_dest, q);
    int ret = FrameToJnt(q_init, f_dest, q, 100000, 5e-3);
    if (ret < 0) { 
        printf("\nDID NOT CONVERGE");
        return NULL;
    }
    //int ret = FrameToJnt(q_init, f_dest, q, 100000, 1e-3);
    //printf("\nret = %d", ret);
    //printf("\njointsolve: ");
    for (int i = 0; i < num_joints; i++) { 
        //printf("%f ", q(i));
        if(ret >= 0) {
            kdl_guess[i] = q(i);
        }
    }
    //ChainFkSolverPos_recursive fksolver2((*_tip_chain));
    //Frame frame; 
    //fksolver2.JntToCart(q, frame); 
    //printf("\nv_solve: [%f %f %f]", frame.p[0], frame.p[1], frame.p[2]);
    //frame.M.GetEulerZYX(z,y,x);
    //printf("\nr_solve: [%f %f %f]", z,y,x);
    x = sin(q(0))*sin(q(1))*q(2);
    y = cos(q(1))*q(2);
    z = cos(q(0))*sin(q(1))*q(2);
    //if (called) { 
    //    printf("\n[x,y,z] = [%f %f %f]", x,y,z); 
    //}
    out[iX] = x; // x = sin(stick_pitch)*sin(stick_gross)*stick_length
    out[iY] = y;           // y = cos(stick_pitch)*stick_length
    out[iZ] = z;  // z = sin(stick_pitch)*cos(stick_gross)*stick_length
    out[iGROSS] = q(3);
    out[iPITCH] = q(4);
    out[iTILT] = -1 * q(5);
    out[iGRIP] = in[iGRIP];
    return out; 
    
}

/*void Accelera_geometry::test_kdl_forward( JntArray joints ) { 
 
    ChainFkSolverPos_recursive fksolver(chain);//Forward position solver
    Frame F_forward; 
    fksolver.JntToCart(joints, F_forward); 
    for (int i = 0; i < 3; i++) { 
       printf("thinks: [%f]\n", F_forward.p[i]);
    }
    for (int i = 0; i < 3; i++) { 
       for (int j = 0; j < 3; j++) { 
          printf("%f ", F_forward.M(i,j));
       }
          printf("\n");
    }
}*/
/*
void Accelera_geometry::test_kdl(Frame F_dest) {
    //approach: use kdl to ensure backwards from pt to elbow make sense. 
    KDL::Chain chain;
    chain.addSegment(Segment(Joint(Joint::RotY),   Frame(Vector(0.0,0.0,0.0))));
    chain.addSegment(Segment(Joint(Joint::RotX),   Frame(Vector(0.0,0.0,0.0))));
    chain.addSegment(Segment(Joint(Joint::TransY), Frame(Vector(0.0,0.0,0.0)))); // is it tx or ty at 0?
    chain.addSegment(Segment(Joint(Joint::RotY),   Frame(Vector(0.0,wrist_gross_length,0.0))));
    chain.addSegment(Segment(Joint(Joint::RotX),   Frame(Vector(0.0,0.0,-1*wrist_pitch_length))));
    chain.addSegment(Segment(Joint(Joint::RotY),   Frame(Vector(-1*wrist_tilt_length,0.0,0.0))));

    JntArray jnt_min(chain.getNrOfJoints());
    JntArray jnt_max(chain.getNrOfJoints());
    printf("Number of joints: %d", chain.getNrOfJoints());
    jnt_min(0) =     -1*M_PI;  jnt_max(0) =     M_PI;
    jnt_min(1) =     -1*M_PI;  jnt_max(1) =     M_PI;  
    jnt_min(2) =           0;  jnt_max(2) =      350;
    jnt_min(3) =          -2;  jnt_max(3) = 2*M_PI+2;
    jnt_min(4) =           0;  jnt_max(4) =     M_PI;
    jnt_min(5) =           1;  jnt_max(5) =-1*M_PI-1;
    
    
    ChainFkSolverPos_recursive fksolver(chain);//Forward position solver
    //ChainIkSolverVel_wdls iksolverv(chain);//Inverse velocity solver
    ChainIkSolverVel_pinv iksolverv(chain);
    ChainIkSolverPos_NR_JL iksolverpos(chain,jnt_min,jnt_max,fksolver,iksolverv,100000,5e-1);//Maximum 100 iterations, stop at accuracy 1e-6
    //ChainIkSolverPos_NR iksolverpos(chain,fksolver,iksolverv,100000,2e0);

    JntArray q(chain.getNrOfJoints());
    JntArray q_init(chain.getNrOfJoints());
    for (int i = 0; i < chain.getNrOfJoints(); i++) {
        q_init(i) = test_kdl_arr[i];
        //q_init(i) = M_PI/2;
    }


    //Rotation R_dest;
    /*
    Rotation R_dest( 1.000,       0.000,        0.000,
                     0.000, cos(M_PI/4),-1*sin(M_PI/4),
                     0.000, sin(M_PI/4),   cos(M_PI/4) );                 
     //////////
    /*
    Rotation R_dest(  0.000,  0.000, -1.000,
                      1.000,  0.000,  0.000,
                      0.000, -1.000,  0.000 );
     /////////                  
    /*
    double alpha, beta, gamma;
    R_dest.GetEulerZYX(alpha, beta, gamma);
    printf("\nEuler ZYX: [%f, %f, %f]\n", alpha, beta, gamma);
    R_dest.GetEulerZYZ(alpha, beta, gamma);
    printf("Euler ZYZ: [%f, %f, %f]\n", alpha, beta, gamma);
    R_dest.GetRPY(alpha, beta, gamma);
    printf("RPY: [%f, %f, %f]\n", alpha, beta, gamma);

    Rotation R_test;
    R_test.DoRotY(M_PI);
    R_test.DoRotX(M_PI/2);
    R_test.DoRotY(M_PI/2);
    
    for (int i = 0; i < 3; i++) { 
        for (int j = 0; j < 3; j++) { 
            printf("%f ", R_test(i,j));
        }
        printf("\n");
    }
     //////////

    printf("\np_dest = (%f, %f, %f)", F_dest.p[0], F_dest.p[1], F_dest.p[2]);
    printf("\nr_dest: \n"); 
    for (int i = 0; i < 3; i++) { 
        for (int j = 0; j < 3; j++) { 
            printf("%f ", F_dest.M(i,j));
        }
        printf("\n");
    }                
    //Vector P_dest ( in[0], in[1], in[2] );
    
    //Frame F_dest (R_dest, P_dest);
    JntArray q_forward(chain.getNrOfJoints());
    int ret = iksolverpos.CartToJnt(q_init, F_dest, q);
    printf("return value = %d\n", ret);
    for (int i= 0; i<chain.getNrOfJoints(); i++) {
        double value = q(i);
        if (ret >= 0 ) { 
            test_kdl_arr[i] = q(i);
        }
        q_forward(i) = q(i);
        //if (i != 2) { 
        value = value - 2*M_PI*((int)(value / (2*M_PI)));
        printf("joint pos %d: %f (%f)\n", i, value * 180 / M_PI, value);
        //} else { 
        //    printf("joint pos %d: %f\n", i, value);
        //}
    } 
    Vector p_thinks;
    Rotation R_thinks;
    Frame F_thinks(R_thinks, p_thinks);
    ChainFkSolverPos_recursive fksolver_reset(chain);
    /*
    q_forward(0) = 1*M_PI/2;
    q_forward(1) = 0;
    q_forward(2) = -1*M_PI/2;

    
    for (int i = 0; i < chain.getNrOfJoints(); i++) {
        printf("thinks joint %d: [%f]\n", i, q_forward(i));
    } /////////
    fksolver_reset.JntToCart(q_forward, F_thinks);
    for (int i = 0; i < 3; i++) { 
        printf("thinks: [%f]\n", F_thinks.p[i]);
    }
    for (int i = 0; i < 3; i++) { 
        for (int j = 0; j < 3; j++) { 
            printf("%f ", F_thinks.M(i,j));
        }
        printf("\n");
    }
}
*/
double* Accelera_geometry:: point_to_motors(const double in[num_dof], double out[num_actuators], bool use_elbow_coor, bool use_kdl) {

    double input[num_dof];
    for (int i = 0; i<num_dof; i++) {
        input[i] = in[i];
    }
    if (!STABLE_MODE && !use_elbow_coor && use_kdl) { 
            if(point_to_tip(input,input,kdl_guess,true) == NULL) {
                return NULL; 
            }
    }
    // WRIST KINEMATICS
    // Apply the model to get the motor positions
    CwMtx::CWTMatrix<> wrist_goal(4, 1);
    wrist_goal[0][0] = input[iPITCH];   // pitch
    wrist_goal[1][0] = input[iTILT];    // tilt
    wrist_goal[2][0] = input[iGRIP];    // grip
    wrist_goal[3][0] = input[iGROSS];   // gross

    wrist_goal = (*_wrist_to_motor_transform) * wrist_goal;
    for(int i=0; i<4; i++){
        out[i] = wrist_goal[i][0];
    }
    out[7] = 0; // no more pneumatics, 'H' is unused

    // BASE KINEMATICS
    double v[3];
    v[0] = input[iX];
    v[1] = input[iY];
    v[2] = input[iZ];
    if(!use_elbow_coor) {
        // Move from tip to elbow
        elbow_tip_swap(v, v);
    }

    elbow_to_base(v, v);  
    base_to_motor(v, v);
    for(int i=0;i<3;i++) {
        out[i+4] = v[i];
    }
    return out;
}

double* Accelera_geometry::motors_to_point(const double motor_pos[num_actuators], double out[num_dof], bool use_elbow_coor, bool use_kdl) {
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
    if (!use_elbow_coor && !STABLE_MODE && use_kdl) {
        elbow_tip_swap(base, base, true, true);
        out[iPITCH]  = wrist_pos[0];      // pitch
        out[iTILT]   = wrist_pos[1];      // tilt
        out[iGROSS]  = wrist_pos[3];      // gross
        out[iSTICK_GROSS] = base[0];
        out[iSTICK_PITCH] = base[1];
        out[iSTICK_LENGTH] =base[2];
        out[iGRIP]   = wrist_pos[2];      // grip
    } else {
        if (!use_elbow_coor) {
            elbow_tip_swap(base,base); 
        }
        // pack result & return
        out[iPITCH]  = wrist_pos[0];      // pitch
        out[iTILT]   = wrist_pos[1];      // tilt
        out[iGROSS]  = wrist_pos[3];      // gross
        out[iX]      = base[0];           // x
        out[iY]      = base[1];           // y
        out[iZ]      = base[2];           // z
        out[iGRIP]   = wrist_pos[2];      // grip
    }
    if (!STABLE_MODE) { 
        if (!use_elbow_coor && use_kdl) { 
            //wrist_to_tip(out,out);
            tip_to_point(out,out);
            //point_to_tip(out,out,kdl_guess);
        }
        //out[iPITCH] = out[iPITCH];      // pitch
        //out[iTILT]  = -1 * out[iTILT];      // tilt
        //out[iGROSS] = out[iGROSS];      // gross
        //test_kdl_arr[0] = out[iGROSS];
        //test_kdl_arr[1] = out[iPITCH];
        //test_kdl_arr[2] = out[iTILT];
        //JntArray q(3);
        //q(0) = out[iGROSS];
        //q(1) = out[iPITCH];
        //q(2) = out[iTILT];
        
        //test_kdl_forward(q);
        
        //Vector p_dest( out[iX], out[iY], out[iZ] );
        //Rotation r_destChainFkSolverPos_recursive fksolver((*_tip_chain));
        //Frame frame; 
        //fksolver.JntToCart(joints, frame); ; 
        //r_dest.DoRotY(out[iGROSS]);
        //r_dest.DoRotX(out[iPITCH]);
        //r_dest.DoRotY(out[iTILT]);
        //Frame f_dest(r_dest, p_dest);
        //test_kdl(f_dest);
        
        /*double test[num_actuators];
        point_to_motors(out, test, use_elbow_coor, use_kdl);
        printf("\n");
        for (int i = 0; i < num_actuators; i++) { 
            printf("%f %f\n", motor_pos[i], test[i]);
        }*/
        
    }
    return out;
}

double* Accelera_geometry::convert_point_to_kdl(double in[num_dof], double out[num_dof]) {
    double motor_pos[num_actuators];
    point_to_motors(in,motor_pos);
    motors_to_point(motor_pos,out,false,true);
}

double* Accelera_geometry::elbow_to_base(double in[3], double out[3]) {

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

double* Accelera_geometry::base_to_motor(double in[3], double out[3]) {
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

double* Accelera_geometry::elbow_tip_swap(double loc[3], double out[3], bool compute_vert, bool compute_rot) {
    CwMtx::CWTVector<> v= CwMtx::CWTVector<>(3);
    v[0] = loc[0]; v[1] = loc[1]; v[2] = loc[2];

    // Note top_stick_length doesn't mean from elbow to trocar
    // necessarily.  Could also mean from trocar to tip.
    double top_stick_length = v.norm();
    double bottom_stick_length = stick_length - top_stick_length;
    CwMtx::CWTVector<> u = v * -1 * ( bottom_stick_length / top_stick_length);
    /*if (compute_vert && !compute_rot) { 
        angle_vertical_tip = -1 * asin(v[2] / top_stick_length);
        if (v[1] > 0 ) {
            angle_vertical_tip = -1*M_PI - angle_vertical_tip; 
        }
        printf("\nangle_vert_tip: %f (%f), angle_rot_tip: %f (%f)\n", angle_vertical_tip * 180 / M_PI,
        angle_vertical_tip, angle_rotation_tip * 180 / M_PI, angle_rotation_tip); 

    } else if (compute_rot && !compute_vert) {
        angle_rotation_tip = -1 * (acos(v[0] / top_stick_length) - M_PI/2);
    }*/
    if (compute_rot && compute_vert) {
        //printf("\ntop_stick_length = %f, bottom_stick_length = %f", top_stick_length, bottom_stick_length);
        angle_vertical_tip = -1 * acos( u[1] / bottom_stick_length );
        angle_rotation_tip = asin( u[0] / (bottom_stick_length * sin(angle_vertical_tip))); 
        //printf("\nangle_vert_tip: %f (%f), angle_rot_tip: %f (%f)", angle_vertical_tip * 180 / M_PI,
        //angle_vertical_tip, angle_rotation_tip * 180 / M_PI, angle_rotation_tip); 
        out[0] = angle_rotation_tip;
        out[1] = angle_vertical_tip;
        out[2] = bottom_stick_length;
        return out;
    }
    if (top_stick_length == 0) {
        u.fill(0.0);
        u[2] = STICK_LENGTH;
    }
    
    for(int i=0;i<3;i++)
        out[i] = u[i];
    return out;
}

double* Accelera_geometry::base_to_elbow(double in[3], double out[3], bool compute_vert, bool compute_rot) {
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
    
   // printf("\nangle_vert: %f (%f), angle_rot: %f (%f)\n", angle_vert * 180 / M_PI,
   // angle_vert, angle_rotation * 180 / M_PI, angle_rotation);
    
    if (compute_vert) { 
        angle_rotation = 0;
    }
    
    if (compute_rot) { 
        angle_vert = M_PI / 2;
    }

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

double* Accelera_geometry::motor_to_base(double in[3], double out[3]) {
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

double* Accelera_geometry::wrist_to_tip(double in[num_dof], double out[num_dof]) { 
    double pitch = in[iPITCH];
    double tilt = in[iTILT];
    double gross = in[iGROSS];
    double x = in[iX];
    double y = in[iY];
    double z = in[iZ];
    double grip = in[iGRIP];
    
    
    tip_as_point(in,in);
    
    out[iPITCH] = pitch;
    out[iTILT] = tilt;
    out[iGROSS] = gross;
    out[iX] = in[iX]; 
    out[iY] = in[iY];  
    out[iZ] = in[iZ];  
    out[iGRIP] = grip;
    
    return out; 
}


double* Accelera_geometry::tip_to_wrist(double in[num_dof], double out[num_dof]) { 
    double pitch = in[iPITCH];
    double tilt = in[iTILT];
    double gross = in[iGROSS];
    double x = in[iX];
    double y = in[iY];
    double z = in[iZ];
    double grip = in[iGRIP];
    
    tip_as_point(in,in);
    
    out[iPITCH] = pitch;
    out[iTILT] = tilt;
    out[iGROSS] = gross;
    out[iX] = x - in[iX]; 
    out[iY] = y - in[iY];  
    out[iZ] = z - in[iZ];  
    out[iGRIP] = grip;
    
    return out; 
}

double* Accelera_geometry::tip_as_point(double in[num_dof], double out[num_dof]) {
    double v[3];
    v[0] = in[iX];
    v[1] = in[iY];
    v[2] = in[iZ];
    
    //printf("\n%f %f %f\n", v[0], v[1], v[2]);
    // Move from tip to elbow to base
    elbow_tip_swap(v,v);
    elbow_to_base(v,v);
    // Calculate angles
    double tip_rot[3] = {v[0], v[1], v[2]};
    double tip_vert[3] = {v[0], v[1], v[2]}; 
    base_to_elbow(tip_vert, tip_vert, true); // sets up vertical angle calculation
    base_to_elbow(tip_rot, tip_rot, false, true); // sets up rot angle calculation
    elbow_tip_swap(tip_vert, tip_vert, true); // calculates vertical angle
    elbow_tip_swap(tip_rot, tip_rot, false, true); // calculates rot angle
 

    double pitch = in[iPITCH];
    double tilt = in[iTILT];
    double gross = in[iGROSS];
  
    /* for faster computation */ 
    double cos_gross = cos(gross);
    double cos_pitch = cos(pitch);
    double cos_tilt = cos(tilt);
    double sin_gross = sin(gross); 
    double sin_pitch = sin(pitch);
    double sin_tilt = sin(tilt); 
    
    /* to rotate from tip frame to stick frame */ 
    //FIXME
    angle_vertical_tip = 0;
    angle_rotation_tip = 0;
    double cos_pitch_tip = cos(angle_vertical_tip);
    double sin_pitch_tip = sin(angle_vertical_tip);
    double cos_gross_tip = cos(angle_rotation_tip);
    double sin_gross_tip = sin(angle_rotation_tip); 
    
    #define M(m, n) (*_gross_transform)[m][n] 
    //along y
    M(0,0) =  cos_gross;    M(0,2) = sin_gross;      
    M(2,0) =  -1*sin_gross; M(2,2) = cos_gross; 
    #undef M
    
    #define M(m, n) (*_pitch_transform)[m][n]
    //along x
    M(1,1) =  cos_pitch;    M(1,2) = -1*sin_pitch;
    M(2,1) =  sin_pitch;    M(2,2) = cos_pitch;
    #undef M
    
    #define M(m, n) (*_tilt_transform)[m][n]
    //along y
    M(0,0) =  cos_tilt;     M(0,2) =  sin_tilt;
    M(2,0) =  -1*sin_tilt;  M(2,2) =  cos_tilt;
    #undef M
    
    #define M(m, n) (*_tip_pitch_transform)[m][n]
    //along x
    M(1,1) =  cos_pitch_tip;    M(1,2) = -1*sin_pitch_tip;
    M(2,1) =  sin_pitch_tip;    M(2,2) = cos_pitch_tip;
    #undef M
    
    #define M(m, n) (*_tip_gross_transform)[m][n] 
    //along y
    M(0,0) =  cos_gross_tip;    M(0,2) = sin_gross_tip;      
    M(2,0) =  -1*sin_gross_tip; M(2,2) = cos_gross_tip; 
    #undef M
     
    CwMtx::CWTMatrix<> wrist_1(4,1);
    CwMtx::CWTMatrix<> wrist_2(4,1);
    CwMtx::CWTMatrix<> wrist_3(4,1);

    wrist_1[0][0] = 0.000;                   
    wrist_1[1][0] = wrist_gross_length;     
    wrist_1[2][0] = 0.000;                
    wrist_1[3][0] = 1.000;                   
    
    wrist_2[0][0] = 0.000;
    wrist_2[1][0] = 0.000;
    wrist_2[2][0] = wrist_pitch_length;
    wrist_2[3][0] = 1.000;
    
    wrist_3[0][0] = wrist_tilt_length;
    wrist_3[1][0] = 0.000;
    wrist_3[2][0] = 0.000;
    wrist_3[3][0] = 1.000;
    
    wrist_2 = (*_gross_transform)*((*_pitch_transform)*wrist_2);
    wrist_3 = (*_gross_transform)*((*_pitch_transform)*((*_tilt_transform)*wrist_3));
    
    wrist_2[0][0] = -1*wrist_2[0][0]; // correcting for sign errors in transformation here
    wrist_2[1][0] = -1*wrist_2[1][0]; // and here
    wrist_2[2][0] = -1*wrist_2[2][0]; // and here
    wrist_3[0][0] = -1*wrist_3[0][0]; // and here
    
    /*printf("\n%f %f %f\n%f %f %f\n%f %f %f",
           wrist_1[0][0], wrist_1[1][0], wrist_1[2][0],
           wrist_2[0][0], wrist_2[1][0], wrist_2[2][0],
           wrist_3[0][0], wrist_3[1][0], wrist_3[2][0]); */ 
           
    wrist_1 = (*_tip_gross_transform)*((*_tip_pitch_transform)*wrist_1);
    wrist_2 = (*_tip_gross_transform)*((*_tip_pitch_transform)*wrist_2);
    wrist_3 = (*_tip_gross_transform)*((*_tip_pitch_transform)*wrist_3);
        
    /*printf("\n%f %f %f\n%f %f %f\n%f %f %f",
           wrist_1[0][0], wrist_1[1][0], wrist_1[2][0],
           wrist_2[0][0], wrist_2[1][0], wrist_2[2][0],
           wrist_3[0][0], wrist_3[1][0], wrist_3[2][0]);
    */
    out[iPITCH] = in[iPITCH];
    out[iTILT]  = in[iTILT];
    out[iGROSS] = in[iGROSS]; 
    out[iX] = wrist_1[0][0] + wrist_2[0][0] + wrist_3[0][0];
    out[iY] = wrist_1[1][0] + wrist_2[1][0] + wrist_3[1][0];
    out[iZ] = wrist_1[2][0] + wrist_2[2][0] + wrist_3[2][0];
    out[iGRIP] = in[iGRIP];
    
    return out; 
}
