#include "ravenMasterReceiver.h"

#define RAD2DEG *180.0/M_PI

struct robot_device rdev;
struct sender2receiver sender_to_receiver;
struct receiver2sender receiver_to_sender;
UDPSocket *UDP_from_slave;
UDPSocket *UDP_from_send;
UDPSocket *UDP_to_send;
unsigned short port_Robot;
char addr_to_send[16];
unsigned short port_to_send;
unsigned short port_from_send;

Log *logIn;
Log *logInBin;
Log *logInBinWhatIWant;
Log *logInReplay; 


void socketInit() {
	port_Robot = (unsigned short) RT_TO_RECEIVER_PORT;
	cout << "From slave port: " << port_Robot << endl << endl;

	UDP_from_slave = new UDPSocket(port_Robot);
	UDP_from_slave->setNonblocking();

	
	strcpy(addr_to_send, SURGICAL5);
	cout << "To sender IP: " << addr_to_send << endl;
	port_to_send = (unsigned short) RECEIVER_TO_SENDER_PORT;
	cout << "To sender port: " << port_to_send << endl;
	port_from_send = (unsigned short) SENDER_TO_RECEIVER_PORT;
	cout << "From sender port: " << port_from_send << endl << endl;

	UDP_to_send = new UDPSocket();
	UDP_to_send->setNonblocking();
	UDP_from_send = new UDPSocket(port_from_send);
	UDP_from_send->setNonblocking();
}

bool recv_UDP_from_slave() {
	string addr;
	unsigned short port;
	int ret;
	bool received = true;

	try {
		ret = UDP_from_slave->recvFrom(&rdev, sizeof(robot_device), addr, port);
	}
	catch (SocketException SE){
		cout << "failed to recv_UDP\n";
	}

	if (ret == sizeof(robot_device)) {
		if (rdev.Ox12345678 == 0x12345678) {
			//printf("fine\n");
		} else if (rdev.Ox12345678 == 0x78563412) {
			//printf("opposite endianness\n");
			unsigned int *ip_first, *ip_last, *ip;
			if ( ((char) (rdev.Ox12345678)) != 0x78) // Data received in other endianness
			{				
				ip_first = (unsigned int*) &(rdev.Ox12345678);
				ip_last  = (unsigned int*) &(rdev.ending_int);
				for (ip = ip_first; ip <= ip_last; ip++)
				{
					*ip = byteswap(*ip);
				}
			}
		} else {
			printf("ERROR: Unsupported endianness\n");
			received = false;
		}
	}	else {
		//printf("received %d of %d bytyes\n", ret, sizeof(robot_device));
		received = false;
	}
	return received;
}

unsigned int byteswap (unsigned int nLongNumber) {
	return (((nLongNumber&0x000000FF)<<24)+((nLongNumber&0x0000FF00)<<8)+
					((nLongNumber&0x00FF0000)>>8)+((nLongNumber&0xFF000000)>>24));
}

bool recv_UDP_from_send() {
	string addr;
	unsigned short port;
	int ret;
	try {
		ret = UDP_from_send->recvFrom(&sender_to_receiver, sizeof(sender2receiver), addr, port);
	}	catch (SocketException SE){
		cout << "failed to recv_UDP from cmd\n";
	}
	return (ret == sizeof(sender2receiver));
}

void send_UDP_to_send() {
	try {
		UDP_to_send->sendTo(&receiver_to_sender,sizeof(receiver2sender),addr_to_send,port_to_send);
	}	catch (SocketException SE) {
		cerr << "failed to send_UDP\n";
	}
}

int main(int argc, char* argv[]) {
	socketInit();
	bool dataStart = false;	
	int id, joint;
	char time_s[25];
	//bool toggle;
	bool received = false;
	
	Timer *timeAlex = new Timer(); 

	logIn = new Log("logs/in");
	logInBin = new Log("logs/in_bin", true, true); 
	logInBinWhatIWant = new Log("logs/in_bin_me", true, true); 
	logInReplay = new Log("logs/replay", true, true); 
	
	timestamp(time_s);
	logIn->Write("[%s] ", time_s);
	
	logIn->Write("rdev.Ox12345678 rdev.type rdev.filler1 rdev.timestamp rdev.runlevel rdev.sublevel rdev.filler2 rdev.surgeon_mode ");
	
	logIn->Write("rdev.mech[2].type rdev.mech[2].filler1 rdev.mech[2].pos.x rdev.mech[2].pos.y rdev.mech[2].pos.z rdev.mech[2].pos_d.x rdev.mech[2].pos_d.y rdev.mech[2].pos_d.z rdev.mech[2].base_pos.x rdev.mech[2].base_pos.y rdev.mech[2].base_pos.z rdev.mech[2].ori.R[0][0] rdev.mech[2].ori.R[1][0] rdev.mech[2].ori.R[2][0] rdev.mech[2].ori.R[0][1] rdev.mech[2].ori.R[1][1] rdev.mech[2].ori.R[2][1] rdev.mech[2].ori.R[0][2] rdev.mech[2].ori.R[1][2] rdev.mech[2].ori.R[2][2] rdev.mech[2].ori.yaw rdev.mech[2].ori.pitch rdev.mech[2].ori.roll rdev.mech[2].ori.grasp rdev.mech[2].ori_d.R[0][0] rdev.mech[2].ori_d.R[1][0] rdev.mech[2].ori_d.R[2][0] rdev.mech[2].ori_d.R[0][1] rdev.mech[2].ori_d.R[1][1] rdev.mech[2].ori_d.R[2][1] rdev.mech[2].ori_d.R[0][2] rdev.mech[2].ori_d.R[1][2] rdev.mech[2].ori_d.R[2][2] rdev.mech[2].ori_d.yaw rdev.mech[2].ori_d.pitch rdev.mech[2].ori_d.roll rdev.mech[2].ori_d.grasp rdev.mech[2].base_ori.R[0][0] rdev.mech[2].base_ori.R[1][0] rdev.mech[2].base_ori.R[2][0] rdev.mech[2].base_ori.R[0][1] rdev.mech[2].base_ori.R[1][1] rdev.mech[2].base_ori.R[2][1] rdev.mech[2].base_ori.R[0][2] rdev.mech[2].base_ori.R[1][2] rdev.mech[2].base_ori.R[2][2] rdev.mech[2].base_ori.yaw rdev.mech[2].base_ori.pitch rdev.mech[2].base_ori.roll rdev.mech[2].base_ori.grasp rdev.mech[2].move_to_pos.x rdev.mech[2].move_to_pos.y rdev.mech[2].move_to_pos.z ");
	logIn->Write("rdev.mech[2].joint[0].mech_type rdev.mech[2].joint[0].type rdev.mech[2].joint[0].state rdev.mech[2].joint[0].enc_val rdev.mech[2].joint[0].filler rdev.mech[2].joint[0].current_cmd rdev.mech[2].joint[0].jpos_off rdev.mech[2].joint[0].jpos rdev.mech[2].joint[0].mpos rdev.mech[2].joint[0].jvel rdev.mech[2].joint[0].mvel rdev.mech[2].joint[0].tau rdev.mech[2].joint[0].tau_d rdev.mech[2].joint[0].tau_g rdev.mech[2].joint[0].jpos_d rdev.mech[2].joint[0].mpos_d rdev.mech[2].joint[0].jcon_jpos_d rdev.mech[2].joint[0].jcon_jpos_t rdev.mech[2].joint[0].jpos_d_old rdev.mech[2].joint[0].mpos_d_old rdev.mech[2].joint[0].jvel_d rdev.mech[2].joint[0].mvel_d rdev.mech[2].joint[0].enc_offset rdev.mech[2].joint[0].perror_int rdev.mech[2].joint[0].merr rdev.mech[2].joint[0].merr_p rdev.mech[2].joint[0].merr_d ");
	logIn->Write("rdev.mech[2].joint[1].mech_type rdev.mech[2].joint[1].type rdev.mech[2].joint[1].state rdev.mech[2].joint[1].enc_val rdev.mech[2].joint[1].filler rdev.mech[2].joint[1].current_cmd rdev.mech[2].joint[1].jpos_off rdev.mech[2].joint[1].jpos rdev.mech[2].joint[1].mpos rdev.mech[2].joint[1].jvel rdev.mech[2].joint[1].mvel rdev.mech[2].joint[1].tau rdev.mech[2].joint[1].tau_d rdev.mech[2].joint[1].tau_g rdev.mech[2].joint[1].jpos_d rdev.mech[2].joint[1].mpos_d rdev.mech[2].joint[1].jcon_jpos_d rdev.mech[2].joint[1].jcon_jpos_t rdev.mech[2].joint[1].jpos_d_old rdev.mech[2].joint[1].mpos_d_old rdev.mech[2].joint[1].jvel_d rdev.mech[2].joint[1].mvel_d rdev.mech[2].joint[1].enc_offset rdev.mech[2].joint[1].perror_int rdev.mech[2].joint[1].merr rdev.mech[2].joint[1].merr_p rdev.mech[2].joint[1].merr_d ");
	logIn->Write("rdev.mech[2].joint[2].mech_type rdev.mech[2].joint[2].type rdev.mech[2].joint[2].state rdev.mech[2].joint[2].enc_val rdev.mech[2].joint[2].filler rdev.mech[2].joint[2].current_cmd rdev.mech[2].joint[2].jpos_off rdev.mech[2].joint[2].jpos rdev.mech[2].joint[2].mpos rdev.mech[2].joint[2].jvel rdev.mech[2].joint[2].mvel rdev.mech[2].joint[2].tau rdev.mech[2].joint[2].tau_d rdev.mech[2].joint[2].tau_g rdev.mech[2].joint[2].jpos_d rdev.mech[2].joint[2].mpos_d rdev.mech[2].joint[2].jcon_jpos_d rdev.mech[2].joint[2].jcon_jpos_t rdev.mech[2].joint[2].jpos_d_old rdev.mech[2].joint[2].mpos_d_old rdev.mech[2].joint[2].jvel_d rdev.mech[2].joint[2].mvel_d rdev.mech[2].joint[2].enc_offset rdev.mech[2].joint[2].perror_int rdev.mech[2].joint[2].merr rdev.mech[2].joint[2].merr_p rdev.mech[2].joint[2].merr_d ");
	logIn->Write("rdev.mech[2].joint[3].mech_type rdev.mech[2].joint[3].type rdev.mech[2].joint[3].state rdev.mech[2].joint[3].enc_val rdev.mech[2].joint[3].filler rdev.mech[2].joint[3].current_cmd rdev.mech[2].joint[3].jpos_off rdev.mech[2].joint[3].jpos rdev.mech[2].joint[3].mpos rdev.mech[2].joint[3].jvel rdev.mech[2].joint[3].mvel rdev.mech[2].joint[3].tau rdev.mech[2].joint[3].tau_d rdev.mech[2].joint[3].tau_g rdev.mech[2].joint[3].jpos_d rdev.mech[2].joint[3].mpos_d rdev.mech[2].joint[3].jcon_jpos_d rdev.mech[2].joint[3].jcon_jpos_t rdev.mech[2].joint[3].jpos_d_old rdev.mech[2].joint[3].mpos_d_old rdev.mech[2].joint[3].jvel_d rdev.mech[2].joint[3].mvel_d rdev.mech[2].joint[3].enc_offset rdev.mech[2].joint[3].perror_int rdev.mech[2].joint[3].merr rdev.mech[2].joint[3].merr_p rdev.mech[2].joint[3].merr_d ");
	logIn->Write("rdev.mech[2].joint[4].mech_type rdev.mech[2].joint[4].type rdev.mech[2].joint[4].state rdev.mech[2].joint[4].enc_val rdev.mech[2].joint[4].filler rdev.mech[2].joint[4].current_cmd rdev.mech[2].joint[4].jpos_off rdev.mech[2].joint[4].jpos rdev.mech[2].joint[4].mpos rdev.mech[2].joint[4].jvel rdev.mech[2].joint[4].mvel rdev.mech[2].joint[4].tau rdev.mech[2].joint[4].tau_d rdev.mech[2].joint[4].tau_g rdev.mech[2].joint[4].jpos_d rdev.mech[2].joint[4].mpos_d rdev.mech[2].joint[4].jcon_jpos_d rdev.mech[2].joint[4].jcon_jpos_t rdev.mech[2].joint[4].jpos_d_old rdev.mech[2].joint[4].mpos_d_old rdev.mech[2].joint[4].jvel_d rdev.mech[2].joint[4].mvel_d rdev.mech[2].joint[4].enc_offset rdev.mech[2].joint[4].perror_int rdev.mech[2].joint[4].merr rdev.mech[2].joint[4].merr_p rdev.mech[2].joint[4].merr_d ");
	logIn->Write("rdev.mech[2].joint[5].mech_type rdev.mech[2].joint[5].type rdev.mech[2].joint[5].state rdev.mech[2].joint[5].enc_val rdev.mech[2].joint[5].filler rdev.mech[2].joint[5].current_cmd rdev.mech[2].joint[5].jpos_off rdev.mech[2].joint[5].jpos rdev.mech[2].joint[5].mpos rdev.mech[2].joint[5].jvel rdev.mech[2].joint[5].mvel rdev.mech[2].joint[5].tau rdev.mech[2].joint[5].tau_d rdev.mech[2].joint[5].tau_g rdev.mech[2].joint[5].jpos_d rdev.mech[2].joint[5].mpos_d rdev.mech[2].joint[5].jcon_jpos_d rdev.mech[2].joint[5].jcon_jpos_t rdev.mech[2].joint[5].jpos_d_old rdev.mech[2].joint[5].mpos_d_old rdev.mech[2].joint[5].jvel_d rdev.mech[2].joint[5].mvel_d rdev.mech[2].joint[5].enc_offset rdev.mech[2].joint[5].perror_int rdev.mech[2].joint[5].merr rdev.mech[2].joint[5].merr_p rdev.mech[2].joint[5].merr_d ");
	logIn->Write("rdev.mech[2].joint[6].mech_type rdev.mech[2].joint[6].type rdev.mech[2].joint[6].state rdev.mech[2].joint[6].enc_val rdev.mech[2].joint[6].filler rdev.mech[2].joint[6].current_cmd rdev.mech[2].joint[6].jpos_off rdev.mech[2].joint[6].jpos rdev.mech[2].joint[6].mpos rdev.mech[2].joint[6].jvel rdev.mech[2].joint[6].mvel rdev.mech[2].joint[6].tau rdev.mech[2].joint[6].tau_d rdev.mech[2].joint[6].tau_g rdev.mech[2].joint[6].jpos_d rdev.mech[2].joint[6].mpos_d rdev.mech[2].joint[6].jcon_jpos_d rdev.mech[2].joint[6].jcon_jpos_t rdev.mech[2].joint[6].jpos_d_old rdev.mech[2].joint[6].mpos_d_old rdev.mech[2].joint[6].jvel_d rdev.mech[2].joint[6].mvel_d rdev.mech[2].joint[6].enc_offset rdev.mech[2].joint[6].perror_int rdev.mech[2].joint[6].merr rdev.mech[2].joint[6].merr_p rdev.mech[2].joint[6].merr_d ");
	logIn->Write("rdev.mech[2].inputs rdev.mech[2].outputs rdev.mech[2].filler2 rdev.mech[2].enabled rdev.mech[2].homing_mode rdev.mech[2].arm_homing_procedure_state rdev.mech[2].tool_plate_placement_procedure_state rdev.mech[2].tool_placement_procedure_state rdev.mech[2].tool_homing_procedure_state rdev.mech[2].processed_packets_from_master ");

	logIn->Write("rdev.mech[3].type rdev.mech[3].filler1 rdev.mech[3].pos.x rdev.mech[3].pos.y rdev.mech[3].pos.z rdev.mech[3].pos_d.x rdev.mech[3].pos_d.y rdev.mech[3].pos_d.z rdev.mech[3].base_pos.x rdev.mech[3].base_pos.y rdev.mech[3].base_pos.z rdev.mech[3].ori.R[0][0] rdev.mech[3].ori.R[1][0] rdev.mech[3].ori.R[2][0] rdev.mech[3].ori.R[0][1] rdev.mech[3].ori.R[1][1] rdev.mech[3].ori.R[2][1] rdev.mech[3].ori.R[0][2] rdev.mech[3].ori.R[1][2] rdev.mech[3].ori.R[2][2] rdev.mech[3].ori.yaw rdev.mech[3].ori.pitch rdev.mech[3].ori.roll rdev.mech[3].ori.grasp rdev.mech[3].ori_d.R[0][0] rdev.mech[3].ori_d.R[1][0] rdev.mech[3].ori_d.R[2][0] rdev.mech[3].ori_d.R[0][1] rdev.mech[3].ori_d.R[1][1] rdev.mech[3].ori_d.R[2][1] rdev.mech[3].ori_d.R[0][2] rdev.mech[3].ori_d.R[1][2] rdev.mech[3].ori_d.R[2][2] rdev.mech[3].ori_d.yaw rdev.mech[3].ori_d.pitch rdev.mech[3].ori_d.roll rdev.mech[3].ori_d.grasp rdev.mech[3].base_ori.R[0][0] rdev.mech[3].base_ori.R[1][0] rdev.mech[3].base_ori.R[2][0] rdev.mech[3].base_ori.R[0][1] rdev.mech[3].base_ori.R[1][1] rdev.mech[3].base_ori.R[2][1] rdev.mech[3].base_ori.R[0][2] rdev.mech[3].base_ori.R[1][2] rdev.mech[3].base_ori.R[2][2] rdev.mech[3].base_ori.yaw rdev.mech[3].base_ori.pitch rdev.mech[3].base_ori.roll rdev.mech[3].base_ori.grasp rdev.mech[3].move_to_pos.x rdev.mech[3].move_to_pos.y rdev.mech[3].move_to_pos.z ");
	logIn->Write("rdev.mech[3].joint[0].mech_type rdev.mech[3].joint[0].type rdev.mech[3].joint[0].state rdev.mech[3].joint[0].enc_val rdev.mech[3].joint[0].filler rdev.mech[3].joint[0].current_cmd rdev.mech[3].joint[0].jpos_off rdev.mech[3].joint[0].jpos rdev.mech[3].joint[0].mpos rdev.mech[3].joint[0].jvel rdev.mech[3].joint[0].mvel rdev.mech[3].joint[0].tau rdev.mech[3].joint[0].tau_d rdev.mech[3].joint[0].tau_g rdev.mech[3].joint[0].jpos_d rdev.mech[3].joint[0].mpos_d rdev.mech[3].joint[0].jcon_jpos_d rdev.mech[3].joint[0].jcon_jpos_t rdev.mech[3].joint[0].jpos_d_old rdev.mech[3].joint[0].mpos_d_old rdev.mech[3].joint[0].jvel_d rdev.mech[3].joint[0].mvel_d rdev.mech[3].joint[0].enc_offset rdev.mech[3].joint[0].perror_int rdev.mech[3].joint[0].merr rdev.mech[3].joint[0].merr_p rdev.mech[3].joint[0].merr_d ");
	logIn->Write("rdev.mech[3].joint[1].mech_type rdev.mech[3].joint[1].type rdev.mech[3].joint[1].state rdev.mech[3].joint[1].enc_val rdev.mech[3].joint[1].filler rdev.mech[3].joint[1].current_cmd rdev.mech[3].joint[1].jpos_off rdev.mech[3].joint[1].jpos rdev.mech[3].joint[1].mpos rdev.mech[3].joint[1].jvel rdev.mech[3].joint[1].mvel rdev.mech[3].joint[1].tau rdev.mech[3].joint[1].tau_d rdev.mech[3].joint[1].tau_g rdev.mech[3].joint[1].jpos_d rdev.mech[3].joint[1].mpos_d rdev.mech[3].joint[1].jcon_jpos_d rdev.mech[3].joint[1].jcon_jpos_t rdev.mech[3].joint[1].jpos_d_old rdev.mech[3].joint[1].mpos_d_old rdev.mech[3].joint[1].jvel_d rdev.mech[3].joint[1].mvel_d rdev.mech[3].joint[1].enc_offset rdev.mech[3].joint[1].perror_int rdev.mech[3].joint[1].merr rdev.mech[3].joint[1].merr_p rdev.mech[3].joint[1].merr_d ");
	logIn->Write("rdev.mech[3].joint[2].mech_type rdev.mech[3].joint[2].type rdev.mech[3].joint[2].state rdev.mech[3].joint[2].enc_val rdev.mech[3].joint[2].filler rdev.mech[3].joint[2].current_cmd rdev.mech[3].joint[2].jpos_off rdev.mech[3].joint[2].jpos rdev.mech[3].joint[2].mpos rdev.mech[3].joint[2].jvel rdev.mech[3].joint[2].mvel rdev.mech[3].joint[2].tau rdev.mech[3].joint[2].tau_d rdev.mech[3].joint[2].tau_g rdev.mech[3].joint[2].jpos_d rdev.mech[3].joint[2].mpos_d rdev.mech[3].joint[2].jcon_jpos_d rdev.mech[3].joint[2].jcon_jpos_t rdev.mech[3].joint[2].jpos_d_old rdev.mech[3].joint[2].mpos_d_old rdev.mech[3].joint[2].jvel_d rdev.mech[3].joint[2].mvel_d rdev.mech[3].joint[2].enc_offset rdev.mech[3].joint[2].perror_int rdev.mech[3].joint[2].merr rdev.mech[3].joint[2].merr_p rdev.mech[3].joint[2].merr_d ");
	logIn->Write("rdev.mech[3].joint[3].mech_type rdev.mech[3].joint[3].type rdev.mech[3].joint[3].state rdev.mech[3].joint[3].enc_val rdev.mech[3].joint[3].filler rdev.mech[3].joint[3].current_cmd rdev.mech[3].joint[3].jpos_off rdev.mech[3].joint[3].jpos rdev.mech[3].joint[3].mpos rdev.mech[3].joint[3].jvel rdev.mech[3].joint[3].mvel rdev.mech[3].joint[3].tau rdev.mech[3].joint[3].tau_d rdev.mech[3].joint[3].tau_g rdev.mech[3].joint[3].jpos_d rdev.mech[3].joint[3].mpos_d rdev.mech[3].joint[3].jcon_jpos_d rdev.mech[3].joint[3].jcon_jpos_t rdev.mech[3].joint[3].jpos_d_old rdev.mech[3].joint[3].mpos_d_old rdev.mech[3].joint[3].jvel_d rdev.mech[3].joint[3].mvel_d rdev.mech[3].joint[3].enc_offset rdev.mech[3].joint[3].perror_int rdev.mech[3].joint[3].merr rdev.mech[3].joint[3].merr_p rdev.mech[3].joint[3].merr_d ");
	logIn->Write("rdev.mech[3].joint[4].mech_type rdev.mech[3].joint[4].type rdev.mech[3].joint[4].state rdev.mech[3].joint[4].enc_val rdev.mech[3].joint[4].filler rdev.mech[3].joint[4].current_cmd rdev.mech[3].joint[4].jpos_off rdev.mech[3].joint[4].jpos rdev.mech[3].joint[4].mpos rdev.mech[3].joint[4].jvel rdev.mech[3].joint[4].mvel rdev.mech[3].joint[4].tau rdev.mech[3].joint[4].tau_d rdev.mech[3].joint[4].tau_g rdev.mech[3].joint[4].jpos_d rdev.mech[3].joint[4].mpos_d rdev.mech[3].joint[4].jcon_jpos_d rdev.mech[3].joint[4].jcon_jpos_t rdev.mech[3].joint[4].jpos_d_old rdev.mech[3].joint[4].mpos_d_old rdev.mech[3].joint[4].jvel_d rdev.mech[3].joint[4].mvel_d rdev.mech[3].joint[4].enc_offset rdev.mech[3].joint[4].perror_int rdev.mech[3].joint[4].merr rdev.mech[3].joint[4].merr_p rdev.mech[3].joint[4].merr_d ");
	logIn->Write("rdev.mech[3].joint[5].mech_type rdev.mech[3].joint[5].type rdev.mech[3].joint[5].state rdev.mech[3].joint[5].enc_val rdev.mech[3].joint[5].filler rdev.mech[3].joint[5].current_cmd rdev.mech[3].joint[5].jpos_off rdev.mech[3].joint[5].jpos rdev.mech[3].joint[5].mpos rdev.mech[3].joint[5].jvel rdev.mech[3].joint[5].mvel rdev.mech[3].joint[5].tau rdev.mech[3].joint[5].tau_d rdev.mech[3].joint[5].tau_g rdev.mech[3].joint[5].jpos_d rdev.mech[3].joint[5].mpos_d rdev.mech[3].joint[5].jcon_jpos_d rdev.mech[3].joint[5].jcon_jpos_t rdev.mech[3].joint[5].jpos_d_old rdev.mech[3].joint[5].mpos_d_old rdev.mech[3].joint[5].jvel_d rdev.mech[3].joint[5].mvel_d rdev.mech[3].joint[5].enc_offset rdev.mech[3].joint[5].perror_int rdev.mech[3].joint[5].merr rdev.mech[3].joint[5].merr_p rdev.mech[3].joint[5].merr_d ");
	logIn->Write("rdev.mech[3].joint[6].mech_type rdev.mech[3].joint[6].type rdev.mech[3].joint[6].state rdev.mech[3].joint[6].enc_val rdev.mech[3].joint[6].filler rdev.mech[3].joint[6].current_cmd rdev.mech[3].joint[6].jpos_off rdev.mech[3].joint[6].jpos rdev.mech[3].joint[6].mpos rdev.mech[3].joint[6].jvel rdev.mech[3].joint[6].mvel rdev.mech[3].joint[6].tau rdev.mech[3].joint[6].tau_d rdev.mech[3].joint[6].tau_g rdev.mech[3].joint[6].jpos_d rdev.mech[3].joint[6].mpos_d rdev.mech[3].joint[6].jcon_jpos_d rdev.mech[3].joint[6].jcon_jpos_t rdev.mech[3].joint[6].jpos_d_old rdev.mech[3].joint[6].mpos_d_old rdev.mech[3].joint[6].jvel_d rdev.mech[3].joint[6].mvel_d rdev.mech[3].joint[6].enc_offset rdev.mech[3].joint[6].perror_int rdev.mech[3].joint[6].merr rdev.mech[3].joint[6].merr_p rdev.mech[3].joint[6].merr_d ");
	logIn->Write("rdev.mech[3].inputs rdev.mech[3].outputs rdev.mech[3].filler2 rdev.mech[3].enabled rdev.mech[3].homing_mode rdev.mech[3].arm_homing_procedure_state rdev.mech[3].tool_plate_placement_procedure_state rdev.mech[3].tool_placement_procedure_state rdev.mech[3].tool_homing_procedure_state rdev.mech[3].processed_packets_from_master ");

	/*
	logIn->Write("rdev.mech[0].type rdev.mech[0].filler1 rdev.mech[0].pos.x rdev.mech[0].pos.y rdev.mech[0].pos.z rdev.mech[0].pos_d.x rdev.mech[0].pos_d.y rdev.mech[0].pos_d.z rdev.mech[0].base_pos.x rdev.mech[0].base_pos.y rdev.mech[0].base_pos.z rdev.mech[0].ori.R[0][0] rdev.mech[0].ori.R[1][0] rdev.mech[0].ori.R[2][0] rdev.mech[0].ori.R[0][1] rdev.mech[0].ori.R[1][1] rdev.mech[0].ori.R[2][1] rdev.mech[0].ori.R[0][2] rdev.mech[0].ori.R[1][2] rdev.mech[0].ori.R[2][2] rdev.mech[0].ori.yaw rdev.mech[0].ori.pitch rdev.mech[0].ori.roll rdev.mech[0].ori.grasp rdev.mech[0].ori_d.R[0][0] rdev.mech[0].ori_d.R[1][0] rdev.mech[0].ori_d.R[2][0] rdev.mech[0].ori_d.R[0][1] rdev.mech[0].ori_d.R[1][1] rdev.mech[0].ori_d.R[2][1] rdev.mech[0].ori_d.R[0][2] rdev.mech[0].ori_d.R[1][2] rdev.mech[0].ori_d.R[2][2] rdev.mech[0].ori_d.yaw rdev.mech[0].ori_d.pitch rdev.mech[0].ori_d.roll rdev.mech[0].ori_d.grasp rdev.mech[0].base_ori.R[0][0] rdev.mech[0].base_ori.R[1][0] rdev.mech[0].base_ori.R[2][0] rdev.mech[0].base_ori.R[0][1] rdev.mech[0].base_ori.R[1][1] rdev.mech[0].base_ori.R[2][1] rdev.mech[0].base_ori.R[0][2] rdev.mech[0].base_ori.R[1][2] rdev.mech[0].base_ori.R[2][2] rdev.mech[0].base_ori.yaw rdev.mech[0].base_ori.pitch rdev.mech[0].base_ori.roll rdev.mech[0].base_ori.grasp rdev.mech[0].move_to_pos.x rdev.mech[0].move_to_pos.y rdev.mech[0].move_to_pos.z ");
	logIn->Write("rdev.mech[0].joint[0].mech_type rdev.mech[0].joint[0].type rdev.mech[0].joint[0].state rdev.mech[0].joint[0].enc_val rdev.mech[0].joint[0].filler rdev.mech[0].joint[0].current_cmd rdev.mech[0].joint[0].jpos_off rdev.mech[0].joint[0].jpos rdev.mech[0].joint[0].mpos rdev.mech[0].joint[0].jvel rdev.mech[0].joint[0].mvel rdev.mech[0].joint[0].tau rdev.mech[0].joint[0].tau_d rdev.mech[0].joint[0].tau_g rdev.mech[0].joint[0].jpos_d rdev.mech[0].joint[0].mpos_d rdev.mech[0].joint[0].jcon_jpos_d rdev.mech[0].joint[0].jcon_jpos_t rdev.mech[0].joint[0].jpos_d_old rdev.mech[0].joint[0].mpos_d_old rdev.mech[0].joint[0].jvel_d rdev.mech[0].joint[0].mvel_d rdev.mech[0].joint[0].enc_offset rdev.mech[0].joint[0].perror_int rdev.mech[0].joint[0].merr rdev.mech[0].joint[0].merr_p rdev.mech[0].joint[0].merr_d ");
	logIn->Write("rdev.mech[0].joint[1].mech_type rdev.mech[0].joint[1].type rdev.mech[0].joint[1].state rdev.mech[0].joint[1].enc_val rdev.mech[0].joint[1].filler rdev.mech[0].joint[1].current_cmd rdev.mech[0].joint[1].jpos_off rdev.mech[0].joint[1].jpos rdev.mech[0].joint[1].mpos rdev.mech[0].joint[1].jvel rdev.mech[0].joint[1].mvel rdev.mech[0].joint[1].tau rdev.mech[0].joint[1].tau_d rdev.mech[0].joint[1].tau_g rdev.mech[0].joint[1].jpos_d rdev.mech[0].joint[1].mpos_d rdev.mech[0].joint[1].jcon_jpos_d rdev.mech[0].joint[1].jcon_jpos_t rdev.mech[0].joint[1].jpos_d_old rdev.mech[0].joint[1].mpos_d_old rdev.mech[0].joint[1].jvel_d rdev.mech[0].joint[1].mvel_d rdev.mech[0].joint[1].enc_offset rdev.mech[0].joint[1].perror_int rdev.mech[0].joint[1].merr rdev.mech[0].joint[1].merr_p rdev.mech[0].joint[1].merr_d ");
	logIn->Write("rdev.mech[0].joint[2].mech_type rdev.mech[0].joint[2].type rdev.mech[0].joint[2].state rdev.mech[0].joint[2].enc_val rdev.mech[0].joint[2].filler rdev.mech[0].joint[2].current_cmd rdev.mech[0].joint[2].jpos_off rdev.mech[0].joint[2].jpos rdev.mech[0].joint[2].mpos rdev.mech[0].joint[2].jvel rdev.mech[0].joint[2].mvel rdev.mech[0].joint[2].tau rdev.mech[0].joint[2].tau_d rdev.mech[0].joint[2].tau_g rdev.mech[0].joint[2].jpos_d rdev.mech[0].joint[2].mpos_d rdev.mech[0].joint[2].jcon_jpos_d rdev.mech[0].joint[2].jcon_jpos_t rdev.mech[0].joint[2].jpos_d_old rdev.mech[0].joint[2].mpos_d_old rdev.mech[0].joint[2].jvel_d rdev.mech[0].joint[2].mvel_d rdev.mech[0].joint[2].enc_offset rdev.mech[0].joint[2].perror_int rdev.mech[0].joint[2].merr rdev.mech[0].joint[2].merr_p rdev.mech[0].joint[2].merr_d ");
	logIn->Write("rdev.mech[0].joint[3].mech_type rdev.mech[0].joint[3].type rdev.mech[0].joint[3].state rdev.mech[0].joint[3].enc_val rdev.mech[0].joint[3].filler rdev.mech[0].joint[3].current_cmd rdev.mech[0].joint[3].jpos_off rdev.mech[0].joint[3].jpos rdev.mech[0].joint[3].mpos rdev.mech[0].joint[3].jvel rdev.mech[0].joint[3].mvel rdev.mech[0].joint[3].tau rdev.mech[0].joint[3].tau_d rdev.mech[0].joint[3].tau_g rdev.mech[0].joint[3].jpos_d rdev.mech[0].joint[3].mpos_d rdev.mech[0].joint[3].jcon_jpos_d rdev.mech[0].joint[3].jcon_jpos_t rdev.mech[0].joint[3].jpos_d_old rdev.mech[0].joint[3].mpos_d_old rdev.mech[0].joint[3].jvel_d rdev.mech[0].joint[3].mvel_d rdev.mech[0].joint[3].enc_offset rdev.mech[0].joint[3].perror_int rdev.mech[0].joint[3].merr rdev.mech[0].joint[3].merr_p rdev.mech[0].joint[3].merr_d ");
	logIn->Write("rdev.mech[0].joint[4].mech_type rdev.mech[0].joint[4].type rdev.mech[0].joint[4].state rdev.mech[0].joint[4].enc_val rdev.mech[0].joint[4].filler rdev.mech[0].joint[4].current_cmd rdev.mech[0].joint[4].jpos_off rdev.mech[0].joint[4].jpos rdev.mech[0].joint[4].mpos rdev.mech[0].joint[4].jvel rdev.mech[0].joint[4].mvel rdev.mech[0].joint[4].tau rdev.mech[0].joint[4].tau_d rdev.mech[0].joint[4].tau_g rdev.mech[0].joint[4].jpos_d rdev.mech[0].joint[4].mpos_d rdev.mech[0].joint[4].jcon_jpos_d rdev.mech[0].joint[4].jcon_jpos_t rdev.mech[0].joint[4].jpos_d_old rdev.mech[0].joint[4].mpos_d_old rdev.mech[0].joint[4].jvel_d rdev.mech[0].joint[4].mvel_d rdev.mech[0].joint[4].enc_offset rdev.mech[0].joint[4].perror_int rdev.mech[0].joint[4].merr rdev.mech[0].joint[4].merr_p rdev.mech[0].joint[4].merr_d ");
	logIn->Write("rdev.mech[0].joint[5].mech_type rdev.mech[0].joint[5].type rdev.mech[0].joint[5].state rdev.mech[0].joint[5].enc_val rdev.mech[0].joint[5].filler rdev.mech[0].joint[5].current_cmd rdev.mech[0].joint[5].jpos_off rdev.mech[0].joint[5].jpos rdev.mech[0].joint[5].mpos rdev.mech[0].joint[5].jvel rdev.mech[0].joint[5].mvel rdev.mech[0].joint[5].tau rdev.mech[0].joint[5].tau_d rdev.mech[0].joint[5].tau_g rdev.mech[0].joint[5].jpos_d rdev.mech[0].joint[5].mpos_d rdev.mech[0].joint[5].jcon_jpos_d rdev.mech[0].joint[5].jcon_jpos_t rdev.mech[0].joint[5].jpos_d_old rdev.mech[0].joint[5].mpos_d_old rdev.mech[0].joint[5].jvel_d rdev.mech[0].joint[5].mvel_d rdev.mech[0].joint[5].enc_offset rdev.mech[0].joint[5].perror_int rdev.mech[0].joint[5].merr rdev.mech[0].joint[5].merr_p rdev.mech[0].joint[5].merr_d ");
	logIn->Write("rdev.mech[0].joint[6].mech_type rdev.mech[0].joint[6].type rdev.mech[0].joint[6].state rdev.mech[0].joint[6].enc_val rdev.mech[0].joint[6].filler rdev.mech[0].joint[6].current_cmd rdev.mech[0].joint[6].jpos_off rdev.mech[0].joint[6].jpos rdev.mech[0].joint[6].mpos rdev.mech[0].joint[6].jvel rdev.mech[0].joint[6].mvel rdev.mech[0].joint[6].tau rdev.mech[0].joint[6].tau_d rdev.mech[0].joint[6].tau_g rdev.mech[0].joint[6].jpos_d rdev.mech[0].joint[6].mpos_d rdev.mech[0].joint[6].jcon_jpos_d rdev.mech[0].joint[6].jcon_jpos_t rdev.mech[0].joint[6].jpos_d_old rdev.mech[0].joint[6].mpos_d_old rdev.mech[0].joint[6].jvel_d rdev.mech[0].joint[6].mvel_d rdev.mech[0].joint[6].enc_offset rdev.mech[0].joint[6].perror_int rdev.mech[0].joint[6].merr rdev.mech[0].joint[6].merr_p rdev.mech[0].joint[6].merr_d ");
	logIn->Write("rdev.mech[0].inputs rdev.mech[0].outputs rdev.mech[0].filler2 rdev.mech[0].enabled rdev.mech[0].homing_mode rdev.mech[0].arm_homing_procedure_state rdev.mech[0].tool_plate_placement_procedure_state rdev.mech[0].tool_placement_procedure_state rdev.mech[0].tool_homing_procedure_state rdev.mech[0].processed_packets_from_master ");

	logIn->Write("rdev.mech[1].type rdev.mech[1].filler1 rdev.mech[1].pos.x rdev.mech[1].pos.y rdev.mech[1].pos.z rdev.mech[1].pos_d.x rdev.mech[1].pos_d.y rdev.mech[1].pos_d.z rdev.mech[1].base_pos.x rdev.mech[1].base_pos.y rdev.mech[1].base_pos.z rdev.mech[1].ori.R[0][0] rdev.mech[1].ori.R[1][0] rdev.mech[1].ori.R[2][0] rdev.mech[1].ori.R[0][1] rdev.mech[1].ori.R[1][1] rdev.mech[1].ori.R[2][1] rdev.mech[1].ori.R[0][2] rdev.mech[1].ori.R[1][2] rdev.mech[1].ori.R[2][2] rdev.mech[1].ori.yaw rdev.mech[1].ori.pitch rdev.mech[1].ori.roll rdev.mech[1].ori.grasp rdev.mech[1].ori_d.R[0][0] rdev.mech[1].ori_d.R[1][0] rdev.mech[1].ori_d.R[2][0] rdev.mech[1].ori_d.R[0][1] rdev.mech[1].ori_d.R[1][1] rdev.mech[1].ori_d.R[2][1] rdev.mech[1].ori_d.R[0][2] rdev.mech[1].ori_d.R[1][2] rdev.mech[1].ori_d.R[2][2] rdev.mech[1].ori_d.yaw rdev.mech[1].ori_d.pitch rdev.mech[1].ori_d.roll rdev.mech[1].ori_d.grasp rdev.mech[1].base_ori.R[0][0] rdev.mech[1].base_ori.R[1][0] rdev.mech[1].base_ori.R[2][0] rdev.mech[1].base_ori.R[0][1] rdev.mech[1].base_ori.R[1][1] rdev.mech[1].base_ori.R[2][1] rdev.mech[1].base_ori.R[0][2] rdev.mech[1].base_ori.R[1][2] rdev.mech[1].base_ori.R[2][2] rdev.mech[1].base_ori.yaw rdev.mech[1].base_ori.pitch rdev.mech[1].base_ori.roll rdev.mech[1].base_ori.grasp rdev.mech[1].move_to_pos.x rdev.mech[1].move_to_pos.y rdev.mech[1].move_to_pos.z ");
	logIn->Write("rdev.mech[1].joint[0].mech_type rdev.mech[1].joint[0].type rdev.mech[1].joint[0].state rdev.mech[1].joint[0].enc_val rdev.mech[1].joint[0].filler rdev.mech[1].joint[0].current_cmd rdev.mech[1].joint[0].jpos_off rdev.mech[1].joint[0].jpos rdev.mech[1].joint[0].mpos rdev.mech[1].joint[0].jvel rdev.mech[1].joint[0].mvel rdev.mech[1].joint[0].tau rdev.mech[1].joint[0].tau_d rdev.mech[1].joint[0].tau_g rdev.mech[1].joint[0].jpos_d rdev.mech[1].joint[0].mpos_d rdev.mech[1].joint[0].jcon_jpos_d rdev.mech[1].joint[0].jcon_jpos_t rdev.mech[1].joint[0].jpos_d_old rdev.mech[1].joint[0].mpos_d_old rdev.mech[1].joint[0].jvel_d rdev.mech[1].joint[0].mvel_d rdev.mech[1].joint[0].enc_offset rdev.mech[1].joint[0].perror_int rdev.mech[1].joint[0].merr rdev.mech[1].joint[0].merr_p rdev.mech[1].joint[0].merr_d ");
	logIn->Write("rdev.mech[1].joint[1].mech_type rdev.mech[1].joint[1].type rdev.mech[1].joint[1].state rdev.mech[1].joint[1].enc_val rdev.mech[1].joint[1].filler rdev.mech[1].joint[1].current_cmd rdev.mech[1].joint[1].jpos_off rdev.mech[1].joint[1].jpos rdev.mech[1].joint[1].mpos rdev.mech[1].joint[1].jvel rdev.mech[1].joint[1].mvel rdev.mech[1].joint[1].tau rdev.mech[1].joint[1].tau_d rdev.mech[1].joint[1].tau_g rdev.mech[1].joint[1].jpos_d rdev.mech[1].joint[1].mpos_d rdev.mech[1].joint[1].jcon_jpos_d rdev.mech[1].joint[1].jcon_jpos_t rdev.mech[1].joint[1].jpos_d_old rdev.mech[1].joint[1].mpos_d_old rdev.mech[1].joint[1].jvel_d rdev.mech[1].joint[1].mvel_d rdev.mech[1].joint[1].enc_offset rdev.mech[1].joint[1].perror_int rdev.mech[1].joint[1].merr rdev.mech[1].joint[1].merr_p rdev.mech[1].joint[1].merr_d ");
	logIn->Write("rdev.mech[1].joint[2].mech_type rdev.mech[1].joint[2].type rdev.mech[1].joint[2].state rdev.mech[1].joint[2].enc_val rdev.mech[1].joint[2].filler rdev.mech[1].joint[2].current_cmd rdev.mech[1].joint[2].jpos_off rdev.mech[1].joint[2].jpos rdev.mech[1].joint[2].mpos rdev.mech[1].joint[2].jvel rdev.mech[1].joint[2].mvel rdev.mech[1].joint[2].tau rdev.mech[1].joint[2].tau_d rdev.mech[1].joint[2].tau_g rdev.mech[1].joint[2].jpos_d rdev.mech[1].joint[2].mpos_d rdev.mech[1].joint[2].jcon_jpos_d rdev.mech[1].joint[2].jcon_jpos_t rdev.mech[1].joint[2].jpos_d_old rdev.mech[1].joint[2].mpos_d_old rdev.mech[1].joint[2].jvel_d rdev.mech[1].joint[2].mvel_d rdev.mech[1].joint[2].enc_offset rdev.mech[1].joint[2].perror_int rdev.mech[1].joint[2].merr rdev.mech[1].joint[2].merr_p rdev.mech[1].joint[2].merr_d ");
	logIn->Write("rdev.mech[1].joint[3].mech_type rdev.mech[1].joint[3].type rdev.mech[1].joint[3].state rdev.mech[1].joint[3].enc_val rdev.mech[1].joint[3].filler rdev.mech[1].joint[3].current_cmd rdev.mech[1].joint[3].jpos_off rdev.mech[1].joint[3].jpos rdev.mech[1].joint[3].mpos rdev.mech[1].joint[3].jvel rdev.mech[1].joint[3].mvel rdev.mech[1].joint[3].tau rdev.mech[1].joint[3].tau_d rdev.mech[1].joint[3].tau_g rdev.mech[1].joint[3].jpos_d rdev.mech[1].joint[3].mpos_d rdev.mech[1].joint[3].jcon_jpos_d rdev.mech[1].joint[3].jcon_jpos_t rdev.mech[1].joint[3].jpos_d_old rdev.mech[1].joint[3].mpos_d_old rdev.mech[1].joint[3].jvel_d rdev.mech[1].joint[3].mvel_d rdev.mech[1].joint[3].enc_offset rdev.mech[1].joint[3].perror_int rdev.mech[1].joint[3].merr rdev.mech[1].joint[3].merr_p rdev.mech[1].joint[3].merr_d ");
	logIn->Write("rdev.mech[1].joint[4].mech_type rdev.mech[1].joint[4].type rdev.mech[1].joint[4].state rdev.mech[1].joint[4].enc_val rdev.mech[1].joint[4].filler rdev.mech[1].joint[4].current_cmd rdev.mech[1].joint[4].jpos_off rdev.mech[1].joint[4].jpos rdev.mech[1].joint[4].mpos rdev.mech[1].joint[4].jvel rdev.mech[1].joint[4].mvel rdev.mech[1].joint[4].tau rdev.mech[1].joint[4].tau_d rdev.mech[1].joint[4].tau_g rdev.mech[1].joint[4].jpos_d rdev.mech[1].joint[4].mpos_d rdev.mech[1].joint[4].jcon_jpos_d rdev.mech[1].joint[4].jcon_jpos_t rdev.mech[1].joint[4].jpos_d_old rdev.mech[1].joint[4].mpos_d_old rdev.mech[1].joint[4].jvel_d rdev.mech[1].joint[4].mvel_d rdev.mech[1].joint[4].enc_offset rdev.mech[1].joint[4].perror_int rdev.mech[1].joint[4].merr rdev.mech[1].joint[4].merr_p rdev.mech[1].joint[4].merr_d ");
	logIn->Write("rdev.mech[1].joint[5].mech_type rdev.mech[1].joint[5].type rdev.mech[1].joint[5].state rdev.mech[1].joint[5].enc_val rdev.mech[1].joint[5].filler rdev.mech[1].joint[5].current_cmd rdev.mech[1].joint[5].jpos_off rdev.mech[1].joint[5].jpos rdev.mech[1].joint[5].mpos rdev.mech[1].joint[5].jvel rdev.mech[1].joint[5].mvel rdev.mech[1].joint[5].tau rdev.mech[1].joint[5].tau_d rdev.mech[1].joint[5].tau_g rdev.mech[1].joint[5].jpos_d rdev.mech[1].joint[5].mpos_d rdev.mech[1].joint[5].jcon_jpos_d rdev.mech[1].joint[5].jcon_jpos_t rdev.mech[1].joint[5].jpos_d_old rdev.mech[1].joint[5].mpos_d_old rdev.mech[1].joint[5].jvel_d rdev.mech[1].joint[5].mvel_d rdev.mech[1].joint[5].enc_offset rdev.mech[1].joint[5].perror_int rdev.mech[1].joint[5].merr rdev.mech[1].joint[5].merr_p rdev.mech[1].joint[5].merr_d ");
	logIn->Write("rdev.mech[1].joint[6].mech_type rdev.mech[1].joint[6].type rdev.mech[1].joint[6].state rdev.mech[1].joint[6].enc_val rdev.mech[1].joint[6].filler rdev.mech[1].joint[6].current_cmd rdev.mech[1].joint[6].jpos_off rdev.mech[1].joint[6].jpos rdev.mech[1].joint[6].mpos rdev.mech[1].joint[6].jvel rdev.mech[1].joint[6].mvel rdev.mech[1].joint[6].tau rdev.mech[1].joint[6].tau_d rdev.mech[1].joint[6].tau_g rdev.mech[1].joint[6].jpos_d rdev.mech[1].joint[6].mpos_d rdev.mech[1].joint[6].jcon_jpos_d rdev.mech[1].joint[6].jcon_jpos_t rdev.mech[1].joint[6].jpos_d_old rdev.mech[1].joint[6].mpos_d_old rdev.mech[1].joint[6].jvel_d rdev.mech[1].joint[6].mvel_d rdev.mech[1].joint[6].enc_offset rdev.mech[1].joint[6].perror_int rdev.mech[1].joint[6].merr rdev.mech[1].joint[6].merr_p rdev.mech[1].joint[6].merr_d ");
	logIn->Write("rdev.mech[1].inputs rdev.mech[1].outputs rdev.mech[1].filler2 rdev.mech[1].enabled rdev.mech[1].homing_mode rdev.mech[1].arm_homing_procedure_state rdev.mech[1].tool_plate_placement_procedure_state rdev.mech[1].tool_placement_procedure_state rdev.mech[1].tool_homing_procedure_state rdev.mech[1].processed_packets_from_master ");
	*/
	/*
	logIn->Write("rdev.mech[id].type rdev.mech[id].filler1 rdev.mech[id].pos.x rdev.mech[id].pos.y rdev.mech[id].pos.z rdev.mech[id].pos_d.x rdev.mech[id].pos_d.y rdev.mech[id].pos_d.z rdev.mech[id].base_pos.x rdev.mech[id].base_pos.y rdev.mech[id].base_pos.z rdev.mech[id].ori.R[0][0] rdev.mech[id].ori.R[1][0] rdev.mech[id].ori.R[2][0] rdev.mech[id].ori.R[0][1] rdev.mech[id].ori.R[1][1] rdev.mech[id].ori.R[2][1] rdev.mech[id].ori.R[0][2] rdev.mech[id].ori.R[1][2] rdev.mech[id].ori.R[2][2] rdev.mech[id].ori.yaw rdev.mech[id].ori.pitch rdev.mech[id].ori.roll rdev.mech[id].ori.grasp rdev.mech[id].ori_d.R[0][0] rdev.mech[id].ori_d.R[1][0] rdev.mech[id].ori_d.R[2][0] rdev.mech[id].ori_d.R[0][1] rdev.mech[id].ori_d.R[1][1] rdev.mech[id].ori_d.R[2][1] rdev.mech[id].ori_d.R[0][2] rdev.mech[id].ori_d.R[1][2] rdev.mech[id].ori_d.R[2][2] rdev.mech[id].ori_d.yaw rdev.mech[id].ori_d.pitch rdev.mech[id].ori_d.roll rdev.mech[id].ori_d.grasp rdev.mech[id].base_ori.R[0][0] rdev.mech[id].base_ori.R[1][0] rdev.mech[id].base_ori.R[2][0] rdev.mech[id].base_ori.R[0][1] rdev.mech[id].base_ori.R[1][1] rdev.mech[id].base_ori.R[2][1] rdev.mech[id].base_ori.R[0][2] rdev.mech[id].base_ori.R[1][2] rdev.mech[id].base_ori.R[2][2] rdev.mech[id].base_ori.yaw rdev.mech[id].base_ori.pitch rdev.mech[id].base_ori.roll rdev.mech[id].base_ori.grasp rdev.mech[id].move_to_pos.x rdev.mech[id].move_to_pos.y rdev.mech[id].move_to_pos.z ");
	logIn->Write("rdev.mech[id].joint[0].mech_type rdev.mech[id].joint[0].type rdev.mech[id].joint[0].state rdev.mech[id].joint[0].enc_val rdev.mech[id].joint[0].filler rdev.mech[id].joint[0].current_cmd rdev.mech[id].joint[0].jpos_off rdev.mech[id].joint[0].jpos rdev.mech[id].joint[0].mpos rdev.mech[id].joint[0].jvel rdev.mech[id].joint[0].mvel rdev.mech[id].joint[0].tau rdev.mech[id].joint[0].tau_d rdev.mech[id].joint[0].tau_g rdev.mech[id].joint[0].jpos_d rdev.mech[id].joint[0].mpos_d rdev.mech[id].joint[0].jcon_jpos_d rdev.mech[id].joint[0].jcon_jpos_t rdev.mech[id].joint[0].jpos_d_old rdev.mech[id].joint[0].mpos_d_old rdev.mech[id].joint[0].jvel_d rdev.mech[id].joint[0].mvel_d rdev.mech[id].joint[0].enc_offset rdev.mech[id].joint[0].perror_int rdev.mech[id].joint[0].merr rdev.mech[id].joint[0].merr_p rdev.mech[id].joint[0].merr_d ");
	logIn->Write("rdev.mech[id].joint[1].mech_type rdev.mech[id].joint[1].type rdev.mech[id].joint[1].state rdev.mech[id].joint[1].enc_val rdev.mech[id].joint[1].filler rdev.mech[id].joint[1].current_cmd rdev.mech[id].joint[1].jpos_off rdev.mech[id].joint[1].jpos rdev.mech[id].joint[1].mpos rdev.mech[id].joint[1].jvel rdev.mech[id].joint[1].mvel rdev.mech[id].joint[1].tau rdev.mech[id].joint[1].tau_d rdev.mech[id].joint[1].tau_g rdev.mech[id].joint[1].jpos_d rdev.mech[id].joint[1].mpos_d rdev.mech[id].joint[1].jcon_jpos_d rdev.mech[id].joint[1].jcon_jpos_t rdev.mech[id].joint[1].jpos_d_old rdev.mech[id].joint[1].mpos_d_old rdev.mech[id].joint[1].jvel_d rdev.mech[id].joint[1].mvel_d rdev.mech[id].joint[1].enc_offset rdev.mech[id].joint[1].perror_int rdev.mech[id].joint[1].merr rdev.mech[id].joint[1].merr_p rdev.mech[id].joint[1].merr_d ");
	logIn->Write("rdev.mech[id].joint[2].mech_type rdev.mech[id].joint[2].type rdev.mech[id].joint[2].state rdev.mech[id].joint[2].enc_val rdev.mech[id].joint[2].filler rdev.mech[id].joint[2].current_cmd rdev.mech[id].joint[2].jpos_off rdev.mech[id].joint[2].jpos rdev.mech[id].joint[2].mpos rdev.mech[id].joint[2].jvel rdev.mech[id].joint[2].mvel rdev.mech[id].joint[2].tau rdev.mech[id].joint[2].tau_d rdev.mech[id].joint[2].tau_g rdev.mech[id].joint[2].jpos_d rdev.mech[id].joint[2].mpos_d rdev.mech[id].joint[2].jcon_jpos_d rdev.mech[id].joint[2].jcon_jpos_t rdev.mech[id].joint[2].jpos_d_old rdev.mech[id].joint[2].mpos_d_old rdev.mech[id].joint[2].jvel_d rdev.mech[id].joint[2].mvel_d rdev.mech[id].joint[2].enc_offset rdev.mech[id].joint[2].perror_int rdev.mech[id].joint[2].merr rdev.mech[id].joint[2].merr_p rdev.mech[id].joint[2].merr_d ");
	logIn->Write("rdev.mech[id].joint[3].mech_type rdev.mech[id].joint[3].type rdev.mech[id].joint[3].state rdev.mech[id].joint[3].enc_val rdev.mech[id].joint[3].filler rdev.mech[id].joint[3].current_cmd rdev.mech[id].joint[3].jpos_off rdev.mech[id].joint[3].jpos rdev.mech[id].joint[3].mpos rdev.mech[id].joint[3].jvel rdev.mech[id].joint[3].mvel rdev.mech[id].joint[3].tau rdev.mech[id].joint[3].tau_d rdev.mech[id].joint[3].tau_g rdev.mech[id].joint[3].jpos_d rdev.mech[id].joint[3].mpos_d rdev.mech[id].joint[3].jcon_jpos_d rdev.mech[id].joint[3].jcon_jpos_t rdev.mech[id].joint[3].jpos_d_old rdev.mech[id].joint[3].mpos_d_old rdev.mech[id].joint[3].jvel_d rdev.mech[id].joint[3].mvel_d rdev.mech[id].joint[3].enc_offset rdev.mech[id].joint[3].perror_int rdev.mech[id].joint[3].merr rdev.mech[id].joint[3].merr_p rdev.mech[id].joint[3].merr_d ");
	logIn->Write("rdev.mech[id].joint[4].mech_type rdev.mech[id].joint[4].type rdev.mech[id].joint[4].state rdev.mech[id].joint[4].enc_val rdev.mech[id].joint[4].filler rdev.mech[id].joint[4].current_cmd rdev.mech[id].joint[4].jpos_off rdev.mech[id].joint[4].jpos rdev.mech[id].joint[4].mpos rdev.mech[id].joint[4].jvel rdev.mech[id].joint[4].mvel rdev.mech[id].joint[4].tau rdev.mech[id].joint[4].tau_d rdev.mech[id].joint[4].tau_g rdev.mech[id].joint[4].jpos_d rdev.mech[id].joint[4].mpos_d rdev.mech[id].joint[4].jcon_jpos_d rdev.mech[id].joint[4].jcon_jpos_t rdev.mech[id].joint[4].jpos_d_old rdev.mech[id].joint[4].mpos_d_old rdev.mech[id].joint[4].jvel_d rdev.mech[id].joint[4].mvel_d rdev.mech[id].joint[4].enc_offset rdev.mech[id].joint[4].perror_int rdev.mech[id].joint[4].merr rdev.mech[id].joint[4].merr_p rdev.mech[id].joint[4].merr_d ");
	logIn->Write("rdev.mech[id].joint[5].mech_type rdev.mech[id].joint[5].type rdev.mech[id].joint[5].state rdev.mech[id].joint[5].enc_val rdev.mech[id].joint[5].filler rdev.mech[id].joint[5].current_cmd rdev.mech[id].joint[5].jpos_off rdev.mech[id].joint[5].jpos rdev.mech[id].joint[5].mpos rdev.mech[id].joint[5].jvel rdev.mech[id].joint[5].mvel rdev.mech[id].joint[5].tau rdev.mech[id].joint[5].tau_d rdev.mech[id].joint[5].tau_g rdev.mech[id].joint[5].jpos_d rdev.mech[id].joint[5].mpos_d rdev.mech[id].joint[5].jcon_jpos_d rdev.mech[id].joint[5].jcon_jpos_t rdev.mech[id].joint[5].jpos_d_old rdev.mech[id].joint[5].mpos_d_old rdev.mech[id].joint[5].jvel_d rdev.mech[id].joint[5].mvel_d rdev.mech[id].joint[5].enc_offset rdev.mech[id].joint[5].perror_int rdev.mech[id].joint[5].merr rdev.mech[id].joint[5].merr_p rdev.mech[id].joint[5].merr_d ");
	logIn->Write("rdev.mech[id].joint[6].mech_type rdev.mech[id].joint[6].type rdev.mech[id].joint[6].state rdev.mech[id].joint[6].enc_val rdev.mech[id].joint[6].filler rdev.mech[id].joint[6].current_cmd rdev.mech[id].joint[6].jpos_off rdev.mech[id].joint[6].jpos rdev.mech[id].joint[6].mpos rdev.mech[id].joint[6].jvel rdev.mech[id].joint[6].mvel rdev.mech[id].joint[6].tau rdev.mech[id].joint[6].tau_d rdev.mech[id].joint[6].tau_g rdev.mech[id].joint[6].jpos_d rdev.mech[id].joint[6].mpos_d rdev.mech[id].joint[6].jcon_jpos_d rdev.mech[id].joint[6].jcon_jpos_t rdev.mech[id].joint[6].jpos_d_old rdev.mech[id].joint[6].mpos_d_old rdev.mech[id].joint[6].jvel_d rdev.mech[id].joint[6].mvel_d rdev.mech[id].joint[6].enc_offset rdev.mech[id].joint[6].perror_int rdev.mech[id].joint[6].merr rdev.mech[id].joint[6].merr_p rdev.mech[id].joint[6].merr_d ");
	logIn->Write("rdev.mech[id].inputs rdev.mech[id].outputs rdev.mech[id].filler2 rdev.mech[id].enabled rdev.mech[id].homing_mode rdev.mech[id].arm_homing_procedure_state rdev.mech[id].tool_plate_placement_procedure_state rdev.mech[id].tool_placement_procedure_state rdev.mech[id].tool_homing_procedure_state rdev.mech[id].processed_packets_from_master ");
	*/
	logIn->Write("rdev.ending_int\n");

	char ch;

	while (true) {
		
		if(_kbhit()) {
			ch = _getch(); 
			if (ch == 'q') {
				delete logIn;
				delete logInBin; 
				delete logInBinWhatIWant; 
				exit(0); 	
			} else { 
				logInBinWhatIWant->WriteBinary((char *) &rdev, sizeof(robot_device));
			}
		}
		
		if (recv_UDP_from_send()) {
			receiver_to_sender.received = true;
			send_UDP_to_send();
			receiver_to_sender.received = false;
			if (sender_to_receiver.quit == true) { break; }
			timestamp(time_s);
			logIn->Write("[%s] # %s", time_s, sender_to_receiver.str_in);
			cout << endl << "logging ... " << sender_to_receiver.str_in << endl;
		}
		
		for (int i=0; i<10; i++) {
			if (received = recv_UDP_from_slave()) {
				break;
			}
		}
		
		//while (!recv_UDP_from_slave()) { }
		
		if (received) {
		
		if(!dataStart) { 
			cout << "Logging data starting" << endl; 
			dataStart = true; 
		}
		
		/*if (toggle)
			cout << "." << endl;
		else
			cout << ".." << endl;
		toggle = !toggle;*/
		
		cout << ".";
		cout.flush();
		usleep(100000);
		
		logInBin->WriteBinary((char *) &rdev, sizeof(robot_device));
		
		// TODO: (ALEX) FIXME!!!!!!!
		logInReplay->Write("%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
						timeAlex->elapsed(), 
						rdev.mech[2].joint[0].jpos RAD2DEG,
						rdev.mech[2].joint[1].jpos RAD2DEG,
						rdev.mech[2].joint[2].jpos RAD2DEG,
						rdev.mech[2].joint[3].jpos *1000.0,
						rdev.mech[2].joint[4].jpos RAD2DEG,
						rdev.mech[2].joint[5].jpos RAD2DEG,
						rdev.mech[2].joint[6].jpos RAD2DEG);
		
		timestamp(time_s);
		logIn->Write("[%s] ", time_s);
		
		logIn->Write("%u %hu %hd %u %hu %hu %u %d ", 
									rdev.Ox12345678, 	// for checking endianness
  								rdev.type,
									rdev.filler1, 									// for 32 bit alignment for endianness
									rdev.timestamp,								// time of last update
									rdev.runlevel,									// nothing/init/joints/kinematics/e-stop
									rdev.sublevel,									// which experimental mode are we running
									rdev.filler2, 									// for 32 bit alignment for endianness
									rdev.surgeon_mode);						// Clutching/indexing state - 1==engaged; 0==disengaged
		
		for(id=2; id<4; id++) { //id=0 left robot; id=1 right robot
			logIn->Write("%hu %u %d %d %d %d %d %d %d %d %d %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %d %d %d %d %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %d %d %d %d %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %d %d %d %d %d %d %d ",
				
										rdev.mech[id].type,
										rdev.mech[id].filler1, // for 32 bit alignment for endianness
										
										rdev.mech[id].pos.x,  //actual x
										rdev.mech[id].pos.y,  //actual y
										rdev.mech[id].pos.z,	//actual z
										
										rdev.mech[id].pos_d.x, 
										rdev.mech[id].pos_d.y, 
										rdev.mech[id].pos_d.z, 
										
										rdev.mech[id].base_pos.x, 
										rdev.mech[id].base_pos.y, 
										rdev.mech[id].base_pos.z,
										
										rdev.mech[id].ori.R[0][0], rdev.mech[id].ori.R[1][0], rdev.mech[id].ori.R[2][0], rdev.mech[id].ori.R[0][1], rdev.mech[id].ori.R[1][1], rdev.mech[id].ori.R[2][1], rdev.mech[id].ori.R[0][2], rdev.mech[id].ori.R[1][2], rdev.mech[id].ori.R[2][2],
										rdev.mech[id].ori.yaw, 
										rdev.mech[id].ori.pitch, 
										rdev.mech[id].ori.roll, 
										rdev.mech[id].ori.grasp,
										
										rdev.mech[id].ori_d.R[0][0], rdev.mech[id].ori_d.R[1][0], rdev.mech[id].ori_d.R[2][0], rdev.mech[id].ori_d.R[0][1], rdev.mech[id].ori_d.R[1][1], rdev.mech[id].ori_d.R[2][1], rdev.mech[id].ori_d.R[0][2], rdev.mech[id].ori_d.R[1][2], rdev.mech[id].ori_d.R[2][2],
										rdev.mech[id].ori_d.yaw, 
										rdev.mech[id].ori_d.pitch, 
										rdev.mech[id].ori_d.roll, 
										rdev.mech[id].ori_d.grasp,
										
										rdev.mech[id].base_ori.R[0][0], rdev.mech[id].base_ori.R[1][0], rdev.mech[id].base_ori.R[2][0], rdev.mech[id].base_ori.R[0][1], rdev.mech[id].base_ori.R[1][1], rdev.mech[id].base_ori.R[2][1], rdev.mech[id].base_ori.R[0][2], rdev.mech[id].base_ori.R[1][2], rdev.mech[id].base_ori.R[2][2],
										rdev.mech[id].base_ori.yaw, 
										rdev.mech[id].base_ori.pitch, 
										rdev.mech[id].base_ori.roll, 
										rdev.mech[id].base_ori.grasp,
										
										rdev.mech[id].move_to_pos.x, 	//desired x
										rdev.mech[id].move_to_pos.y, 	//desired y
										rdev.mech[id].move_to_pos.z); //desired z
									
				for(joint=0; joint<7; joint++) { // 7 joints
					logIn->Write("%hu %hu %d %d %hd %hd %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %d %.4f %.4f %.4f %.4f ",
												rdev.mech[id].joint[joint].mech_type, 
												rdev.mech[id].joint[joint].type, 
												rdev.mech[id].joint[joint].state,           // is this DoF enabled?
												rdev.mech[id].joint[joint].enc_val, 				// encoder value

												rdev.mech[id].joint[joint].filler,  				// for 32 bit alignment for endianness

												rdev.mech[id].joint[joint].current_cmd, 		// DAC command to achieve tau at actuator

												rdev.mech[id].joint[joint].jpos_off, 				// offset to jpos to corrent for tool placement
												rdev.mech[id].joint[joint].jpos, 						// actual DOF coordinate (rad)
												rdev.mech[id].joint[joint].mpos, 

												rdev.mech[id].joint[joint].jvel,  					// actual DOF velocity(q-dot) 
												rdev.mech[id].joint[joint].mvel, 
												rdev.mech[id].joint[joint].tau, 						// actual DOF force/torque
												rdev.mech[id].joint[joint].tau_d, 					// desired DOF force/torque 
												rdev.mech[id].joint[joint].tau_g, 					// Estimated gravity force/torque on joint.
												rdev.mech[id].joint[joint].jpos_d, 					// desired DOF coordinate (rad)
												rdev.mech[id].joint[joint].mpos_d, 

												rdev.mech[id].joint[joint].jcon_jpos_d,  		// when in joint control - this is the intermediate desired joint position
												rdev.mech[id].joint[joint].jcon_jpos_t,  		// when in joint control - this is the target joint position

												rdev.mech[id].joint[joint].jpos_d_old,      // previous desired DOF coordinate (rad)
												rdev.mech[id].joint[joint].mpos_d_old,     
												rdev.mech[id].joint[joint].jvel_d, 					// desired DOF velocity (q-dot-desired)
												rdev.mech[id].joint[joint].mvel_d, 
												rdev.mech[id].joint[joint].enc_offset,      // Encoder offset to "zero"
												rdev.mech[id].joint[joint].perror_int,      // integrated position error for joint space position control

												rdev.mech[id].joint[joint].merr, 						// Current motor error
												rdev.mech[id].joint[joint].merr_p, 					// Previous motor error
												rdev.mech[id].joint[joint].merr_d); 					// Error difference
				}
				
				logIn->Write("%hu %hu %u %d %d %d %d %d %d %d ",
											rdev.mech[id].inputs,                  							// input pins
											rdev.mech[id].outputs,                							// output pins
											rdev.mech[id].filler2, 															// for 32 bit alignment for endianness
											rdev.mech[id].enabled, 															// This variable is set at startup (initialization). It's the manual enable/disable switch of the robots.
														 																							// If the robot is not enables - it's motors are inhibited and no motion can be produced.
											rdev.mech[id].homing_mode, 													// Homing mode
											rdev.mech[id].arm_homing_procedure_state,
											rdev.mech[id].tool_plate_placement_procedure_state,
											rdev.mech[id].tool_placement_procedure_state,
											rdev.mech[id].tool_homing_procedure_state,
											rdev.mech[id].processed_packets_from_master);				//not in use
		}
		logIn->Write("%u\n", rdev.ending_int); // for endiannness
		
		}
	}
	return 0;
}
