#include "controller_lqr.h"

Controller_lqr::Controller_lqr(int slave, int statesize, int outputsize, const char* matrixfilepath, const char* targetfilepath) : Controller(slave, statesize, outputsize) {
    
    matrix_data                 = new Time_file(matrixfilepath, TRUE_STATE_SIZE * outputsize);
    target_data                 = new Time_file(targetfilepath, TRUE_STATE_SIZE);
    last_control.tv_sec         = 0;
    last_control.tv_usec        = 0;
    last_state                  = new double[TRUE_STATE_SIZE];
    for(int i=0;i<TRUE_STATE_SIZE;i++) {
        last_state[i]           = 0;
    }
    target_state                = new CwMtx::CWTMatrix<>(TRUE_STATE_SIZE, 1);
    for(int i =0;i<TRUE_STATE_SIZE;i++) {
        (*target_state)[i][0]   = 0;
    }

    K                           = new CwMtx::CWTMatrix<>(output_size, TRUE_STATE_SIZE);
}

Controller_lqr::~Controller_lqr() {
    matrix_data->close();
    target_data->close();
    delete K;
    delete target_state;
    delete[] last_state;
}

// state = [position; velocity; total volt]
// edit by Stephen: sets "goal" to be the target
int Controller_lqr::control(const double* state, double* goal, double* out) {
    // Grab new response log_data (if required)
    if(last_control.tv_sec == 0 && last_control.tv_usec == 0){
        gettimeofday(&last_control, NULL);
    }

    timeval         calltime;
    gettimeofday(&calltime, NULL);
    double interval = timediff(calltime, last_control);
    last_control.tv_sec = calltime.tv_sec;
    last_control.tv_usec = calltime.tv_usec;
    load_next_datum();

    // Position
    CwMtx::CWTMatrix<>  true_state(TRUE_STATE_SIZE, 1);
    for(int i=0;i<state_size;i++) {
        true_state[i][0] = state[i];
    }

    // Input
    for (int i = 0; i < output_size; i++){
         true_state[state_size + i][0] = last_state[state_size + i];
    }
    true_state[TRUE_STATE_SIZE-1][0] = 1;

    // Calculate distance to target
    CwMtx::CWTMatrix<>      output(output_size, 1);
    CwMtx::CWTMatrix<>      distance_to_target(TRUE_STATE_SIZE, 1);
    distance_to_target      = -(*target_state) + true_state;    
    // Ensure bias term is 1
    distance_to_target[TRUE_STATE_SIZE - 1][0] = 1;

    CwMtx::CWTMatrix<> delta_volt = (*K)*distance_to_target;

    // pack up delta_volt
    for(int i=0;i<output_size;i++){ 
        out[i] = delta_volt[i][0]+last_state[state_size+i];
    }

    // Save all necessary info
    last_output.clear();
    for(int i=0;i<state_size;i++) {
        last_state[i] = state[i];
        last_state[state_size + i] += delta_volt[i][0];
        last_output.push_back(out[i]);
    }
    //last_state[TRUE_STATE_SIZE-1] = 1;
    //added by Stephen
    for(int i=0;i<state_size;i++){
        goal[i] = (*target_state)[i][0];
    }
    return output_size;
}

//void Controller_lqr::assert_endofline() {
//
//    char buf[256];
//    log_data->getline(buf,256);
//    for(int i=0;i<strlen(buf);i++){
//        assert(isspace(buf[i]));
//    }
//}

void Controller_lqr::load_next_datum() {
    if(!(matrix_data->eof() || target_data->eof())){
        load_next_matrix();
        load_next_target();
    }
}

//void Controller_lqr::load_next_clock() {
//
//    assert(log_data->is_open());
//    double prev_clock = file_clock;
//    (*log_data) >> file_clock;
//    remainder += file_clock - prev_clock;
//    //printf("clock loaded\n");
//
//    assert_endofline();
//}

void Controller_lqr::load_next_matrix() {
    assert(matrix_data->is_open());
    std::vector<double> data = matrix_data->data();
    //printf("data size: %d, expected size: %d\n", data.size(), TRUE_STATE_SIZE*output_size);
    //printf("this matrix: \n");
    for(int row=0;row<output_size;row++) {
        for(int col=0;col<TRUE_STATE_SIZE;col++) {
            (*K)[row][col] = data[TRUE_STATE_SIZE*row + col];
            //printf("%f ", data[TRUE_STATE_SIZE*row + col]);
        }
        //printf("\n");
    }
    //assert_endofline();
}

void Controller_lqr::load_next_target() {
    assert(target_data->is_open());
    //printf("this target: ");
    std::vector<double> data = target_data->data();
    for(int i=0; i<TRUE_STATE_SIZE; i++){
        (*target_state)[i][0] = data[i];
        //printf("[%d] %f, ", i, data[i]);
    }
    //printf("\n");
    //assert_endofline();
}

void Controller_lqr:: reset_time(){
    matrix_data->reset_time();
    target_data->reset_time();
}

bool Controller_lqr:: is_complete(){
    return matrix_data->eof() || target_data->eof();
    
}
