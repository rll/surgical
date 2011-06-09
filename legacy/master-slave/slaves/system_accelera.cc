#include "system_accelera.h"

using std::cout;
using std::endl;

char real_channel(int channel) {
    //return ((char) (channel + 'A'));
    switch(channel){
    
    case 0:
        return 'E'; //PITCH -- used to be A, now E
        break;
    case 1:
        return 'F'; //YAW1 -- used to be B, now F
        break;
    case 2:
        return 'G'; //YAW2 -- used to be C, now G
        break;
    case 3:
        return 'H'; // GROSS -- used to be D, now H
        break;
    case 4:
        return 'A'; // LEFT BASE -- used to be E, now A
        break;
    case 5:
        return 'B'; // REAR BASE -- used to be F, now B
        break;
    case 6:
        return 'C'; // RIGHT BASE -- used to be G, now C
        break;
    default:
        return 'D'; // DEFAULT -- nothing is plugged into D
        break;
    }
}

void convert_to_real_channels(double* unsortedchannels, double* sortedchannels){
    for(int i = 0; i < num_actuators; i++){
        sortedchannels[(int) real_channel(i)-'A'] = unsortedchannels[i];
    }
}

System_accelera::System_accelera(int slave, boost::shared_ptr<Galil> galil) : System(slave) {
    g = galil;
    assert(g);
    slave_id = slave;
    for(int i = 0; i < num_actuators; i++)
        maxVoltage[i] = 0;

    cout << "Resetting Slave " << slave << "  Controller...";
    send_command("RS");           // Reset so we know where we are
    sleep(3);                   // Wait for it to reset...
    cout << "done!" << endl;
    //send_command("TM 187.5");     // Servo Update loop in usec
    send_command("MO ABCDEFGH");  // Turn off motors
    send_command("AG*=0");        // Turn amplifier gain down
    //SHOULD I DO THIS?

    //send_command("AG A=1");
    //send_command("AG B=1");
    //send_command("AG C=1");
    //send_command("AGH=1");        // Higher amplifier gain for pneumatics
    if (USE_LOW_ACCELERATION) { 
        send_command("AC*=300");      // Turn acceleration rate down to 1024 counts / second^2 
        printf("Using low accerlation\n");
    }
    send_command("SH ABCDEFGH");  // Turn motors back down
    //send_command("DR 5.33");      // status update every millisec 
    send_command("DR 2.00"); // status update every 2 ms
    
    for(int i=0;i<num_actuators;i++) {
        actuator_mins[i] = 0.0;
        actuator_maxs[i] = 0.0;
        queued_voltages[i] = 0.0;
        pid_constants[i][iKP] = 0;
        pid_constants[i][iKI] = 0;
        pid_constants[i][iKD] = 0;
    }
    if (slave == 1) { 
            pid_constants[0][iKP] = 138.00; pid_constants[0][iKI] = 2.00; pid_constants[0][iKD] = 2858.00;
            pid_constants[1][iKP] = 142.00; pid_constants[1][iKI] = 2.00; pid_constants[1][iKD] = 2707.00;
            pid_constants[2][iKP] = 135.00; pid_constants[2][iKI] = 2.00; pid_constants[2][iKD] = 2917.00;
            pid_constants[3][iKP] = 156.00; pid_constants[3][iKI] = 2.00; pid_constants[3][iKD] = 2854.00;
            pid_constants[4][iKP] = 153.00; pid_constants[4][iKI] = 3.00; pid_constants[4][iKD] = 2488.00;
            pid_constants[5][iKP] = 272.00; pid_constants[5][iKI] = 6.00; pid_constants[5][iKD] = 2561.00;
            pid_constants[6][iKP] = 175.00; pid_constants[6][iKI] = 2.00; pid_constants[6][iKD] = 2603.00;
    } else if (slave == 2) { 
            pid_constants[0][iKP] = 135.00; pid_constants[0][iKI] = 2.00; pid_constants[0][iKD] = 2702.00;
            pid_constants[1][iKP] = 146.00; pid_constants[1][iKI] = 2.00; pid_constants[1][iKD] = 3138.00;
            pid_constants[2][iKP] = 160.00; pid_constants[2][iKI] = 3.00; pid_constants[2][iKD] = 3149.00;
            pid_constants[3][iKP] = 137.00; pid_constants[3][iKI] = 2.00; pid_constants[3][iKD] = 2623.00;
            pid_constants[4][iKP] = 154.00; pid_constants[4][iKI] = 3.00; pid_constants[4][iKD] = 2323.00;
            pid_constants[5][iKP] = 135.00; pid_constants[5][iKI] = 2.00; pid_constants[5][iKD] = 2518.00;
            pid_constants[6][iKP] = 130.00; pid_constants[6][iKI] = 3.00; pid_constants[6][iKD] = 2513.00;
    }
}

System_accelera::~System_accelera() { 
    send_command("DR 0"); // Turn off status updates
    send_command("MO");   // turn off all motors
}

void System_accelera::init_encoder(int channel, int sign, double ratio, double offset) {
    assert( channel < num_actuators && channel >= 0);

    encoder_signs[channel] = sign;
    encoder_ratios[channel] = ratio;
    encoder_offsets[channel] = offset;
    
    if(PRINT_ENCODER_OFFSET_CHANGES){
        debugprintf("Encoder sign   [%d] set to %d\n", channel, encoder_signs[channel]);
        debugprintf("Encoder ratio  [%d] set to %f\n", channel, encoder_ratios[channel]);
        debugprintf("Encoder offset [%d] set to %f\n", channel, encoder_offsets[channel]);
    }

    std::string command = build_cmd("DP", real_channel(channel), 'c', 0, 'i');
    send_command(command);
}

double System_accelera::read_encoder(int channel, bool no_cache) {
    assert( channel < num_actuators && channel >= 0);
    fetch_record();

    // TP = Tell Position

    double val;
    if (no_cache) {
        flush_galil_recv_buffer();
        std::string command = build_cmd("TP", real_channel(channel));
        val = g->commandValue(command);
    } else { 
        std::string command = build_cmd("_TP", real_channel(channel));
        val = g->sourceValue(latest_record, command);
    } 
    // Convert to our units
	double currentInput =  val / encoder_ratios[channel];
	currentInput *= encoder_signs[channel];
	currentInput += encoder_offsets[channel];
    //cout << "Channel " << real_channel(channel) << " read" << endl;
    //if (channel == 5) {
        //printf("raw position: %f\n", currentInput);
    //}
    return currentInput;
}

double System_accelera::read_velocity(int channel, bool no_cache) { 
     assert( channel < num_actuators && channel >= 0);
    fetch_record();

    // TV = Tell Velocity

    double val;
    if (no_cache) {
        flush_galil_recv_buffer();
        std::string command = build_cmd("TV", real_channel(channel));
        val = g->commandValue(command);
    } else { 
        std::string command = build_cmd("_TV", real_channel(channel));
        val = g->sourceValue(latest_record, command);
    } 
    // Convert to our units
	double currentInput =  val / encoder_ratios[channel];
	currentInput *= encoder_signs[channel];
    return currentInput;

}

void System_accelera::flush_galil_recv_buffer() { 
    std::string buf = g->read();
    while(buf.size() != 0) {
        buf = g->read();
    }
}

void System_accelera::set_position(int channel, double value) {
    init_encoder(channel, encoder_signs[channel], encoder_ratios[channel], value);
}

void System_accelera::fetch_record() {
    std::vector<char> new_record;
    int old_timeout = g->timeout_ms;
    g->timeout_ms = 0;
    while (true) {
        try {
            new_record = g->record("DR");
            latest_record = new_record;
        } catch (...) {
            g->timeout_ms = old_timeout;
            break;
        }
    }
    //cout << "record fetched" << endl;
}

void System_accelera::init_actuator(int channel, int sign, double offset) {
    assert( channel < num_actuators && channel >= 0);
    actuator_signs[channel] = sign;
    actuator_offsets[channel] = offset;

    std::string kp = build_cmd("KP", real_channel(channel), 'c', 0.0, 'd');   // Turn off PID
    std::string ki = build_cmd("KI", real_channel(channel), 'c', 0.0, 'd');
    std::string kd = build_cmd("KD", real_channel(channel), 'c', 0.0, 'd');
    std::string mt = build_cmd("MT", real_channel(channel), 'c',   1, 'i');   // Set servo motor
    std::string br = build_cmd("BR", real_channel(channel), 'c',   1, 'i');   // Set brushed motor
    std::string au = build_cmd("AU", real_channel(channel), 'c', 0.5, 'd');   // Set low inductance
    send_command(kp);
    send_command(ki);
    send_command(kd);
    send_command(mt);
    send_command(br);
    send_command(au);
    //cout << "Actuator " << real_channel(channel) << " initialized" << endl;
}

void System_accelera::queue_voltage(int channel, double volt) {
    assert( channel < num_actuators && channel >= 0);
    if(volt > actuator_maxs[channel]) 
        volt = actuator_maxs[channel];
    else if(volt < actuator_mins[channel]) 
        volt = actuator_mins[channel];
    // Sign + offset
    volt *= actuator_signs[channel];
    volt += actuator_offsets[channel];

    // Controller limits
    if (volt > 9.9) {
        volt = 9.9;
    } else if (volt < -1 * 9.9) {
        volt = -1 * 9.9;
    }
    volt *= 0.5;
    if(NO_VOLTAGES)
        volt = 0;
    queued_voltages[channel] = volt;
    

    //cout << "Channel " << real_channel(channel) << " voltage " << volt << " added" << endl;
}

void System_accelera::init_pid_control() { 
    init_reference_position();
    for (int i = 0; i < num_actuators; i++) {
        printf("\nInitializing channel %d...", i);  
        init_pid(i);
    }
}

void System_accelera::init_reference_position() { 
    for (int channel = 0; channel < num_actuators; channel++) { 
        double encoder_value = (read_encoder(channel) - encoder_offsets[channel]) * encoder_signs[channel] * encoder_ratios[channel];
        std::string pa = build_cmd("PA", real_channel(channel), 'c', encoder_value, 'd');
        send_command(pa);
    }
    std::string bg = "BG";
    send_command(bg);
    sleep(1);
    
    send_command("PT*=1"); //enable position tracking
    
}

void System_accelera::init_pid(int channel) {
    //set PID to 0 for safety 
    std::string kp = build_cmd("KP", real_channel(channel), 'c', 0.0, 'd');   // Turn off PID
    std::string ki = build_cmd("KI", real_channel(channel), 'c', 0.0, 'd');
    std::string kd = build_cmd("KD", real_channel(channel), 'c', 0.0, 'd');
    //#define send_command(x) printf("\n%s", x.c_str())
    send_command(kp);
    send_command(ki);
    send_command(kd);
    sleep(1);
    // setup PID constants
    kp = build_cmd("KP", real_channel(channel), 'c', pid_constants[channel][iKP], 'd');   // Turn on PID
    ki = build_cmd("KI", real_channel(channel), 'c', pid_constants[channel][iKI], 'd');
    kd = build_cmd("KD", real_channel(channel), 'c', pid_constants[channel][iKD], 'd');
    send_command(kp);
    send_command(ki);
    send_command(kd);
    sleep(1);
    
    
    //#undef send_command

}

void System_accelera::set_reference_position(double motor_pos[num_actuators]) { 
    //#define send_command(x) printf("\n%s", x.c_str())
    //printf("\nSlave Id = %d", slave_id);
    std::ostringstream command;
    //command << "AM;";
    /*printf("\nReference (Ignore 0-4): ");
    for (int i = 0; i < num_actuators; i++) {
        printf("[%d] %f ", i, motor_pos[i]);
    }*/
    for (int channel = 0; channel < num_actuators; channel++) {
        double encoder_value = (motor_pos[channel] - encoder_offsets[channel]) * encoder_signs[channel] * encoder_ratios[channel];
        std::string pa = build_cmd("PA", real_channel(channel), 'c', encoder_value, 'd');
        command << pa << ";";
    }
    command << "BG;";
    std::string s = command.str();
    int old_timeout = g->timeout_ms;
    g->timeout_ms = 0;
    try {
        send_command(s);
    } catch(...) {
        if(PRINT_COMMAND_FAILURES)
            printf("\nFAILURE: %s", s.c_str());
    }
    g->timeout_ms = old_timeout;
    //sleep(1);
    //std::string bg = "BG;"
    //send_command(bg);
    //std::string mc = "MC";
    //send_command(mc);
    //#undef send_command
}

void System_accelera::apply_voltages() {

    double voltage_by_channel[num_actuators];
    convert_to_real_channels(queued_voltages, voltage_by_channel);
    std::ostringstream command;
    command << "OF ";
    for(int i=0;i<num_actuators-1;i++) {
        command << floor(voltage_by_channel[i]*1000)/1000 << ",";
    }
    command << floor(voltage_by_channel[num_actuators-1]*1000)/1000;
    
    int old_timeout = g->timeout_ms;
    g->timeout_ms = 0;
    
    try {
        send_command(command.str().c_str());
    } catch (...) { 
        if(PRINT_COMMAND_FAILURES)
            printf("Command '%s' did not work.\n",command.str().c_str());
    }
    for(int i = 0; i < num_actuators;i++){
        if(voltage_by_channel[i] > maxVoltage[i])
            maxVoltage[i] = voltage_by_channel[i];
    }
    if(PRINT_MAX_VOLTAGE)
        printf("Max voltages so far: %f,%f,%f,%f,%f,%f,%f,%f\n", maxVoltage[0],maxVoltage[1],maxVoltage[2],maxVoltage[3],maxVoltage[4],maxVoltage[5],maxVoltage[6],maxVoltage[7]);
    g->timeout_ms = old_timeout;
}

void System_accelera::set_limit(int channel, double min, double max) {
    assert( channel < num_actuators && channel >= 0);
    actuator_mins[channel] = min;
    actuator_maxs[channel] = max;

    //cout << "Channel " << real_channel(channel) << "min/max set: " << min << " " << max << endl;
}

void System_accelera::set_torque_limit(int channel, double limit, double peak){
    assert(g);
    assert( channel < num_actuators && channel >= 0);
    real_channel(channel);
    std::string tl = build_cmd("TL", real_channel(channel), 'c', limit, 'd');   // Sets the Torque Limit
    std::string tk = build_cmd("TK", real_channel(channel), 'c', peak, 'd');   // Sets the Peak Torque
    send_command(tk);
    send_command(tl);
    //As a safety precaution, we keep the hard limit -- but at the peak
    actuator_mins[channel] = -1 *peak * 2;
    actuator_maxs[channel] = peak * 2;
}

void System_accelera::send_command(const char* command){
    if(PRINT_COMMANDS){
        printf("Send Command: %s\n",command);
    }
    g->command(command);
}

void System_accelera::send_command(std::string& command){
    System_accelera::send_command(command.c_str());
}

void System_accelera::reset_motors(){
    //send_command("MO ABCDEFGH");
    reset_all();
    usleep(1000);
    //send_command("SH ABCDEFGH");  // Turn motors back down
}
