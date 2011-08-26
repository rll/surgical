#include "ravenMasterSender.h"

struct u_struct msgHeader;
struct cmd2sender cmd_to_sender;
struct sender2receiver sender_to_receiver;
struct receiver2sender receiver_to_sender;
	
UDPSocket *UDP_to_slave;
UDPSocket *UDP_from_recv;
UDPSocket *UDP_to_recv;
UDPSocket *UDP_from_cmd;
char addr_Robot[16];
unsigned short port_Robot;
char addr_to_cmd[16];
unsigned short port_to_cmd;
unsigned short port_from_cmd;
char addr_to_recv[16];
unsigned short port_to_recv;
unsigned short port_from_recv;

//This two have to updated
double position[2][3] = { {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0} };
double rotation[2][3] = { {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0} };
int buttonstate[2] = {0, 0};

Log *logOut;

void socketInit() {	
	strcpy(addr_Robot, RT_LINUX);
	cout << "To slave IP: " << addr_Robot << endl;
	port_Robot = (unsigned short) SENDER_TO_RT_PORT;
	cout << "To slave port: " << port_Robot << endl << endl;

	UDP_to_slave = new UDPSocket();
	UDP_to_slave->setNonblocking();
	
	
	strcpy(addr_to_cmd, SURGICAL5);
	cout << "To command IP: " << addr_to_cmd << endl;
	port_to_cmd = (unsigned short) SENDER_TO_CMD_PORT;
	cout << "To command port: " << port_to_cmd << endl;
	port_from_cmd = (unsigned short) CMD_TO_SENDER_PORT;
	cout << "From command port: " << port_from_cmd << endl << endl;
	
	UDP_from_cmd = new UDPSocket(port_from_cmd);
	UDP_from_cmd->setNonblocking();
	
	
	strcpy(addr_to_recv, SURGICAL5);
	cout << "To receiver IP: " << addr_to_recv << endl;
	port_to_recv = (unsigned short) SENDER_TO_RECEIVER_PORT;
	cout << "To receiver port: " << port_to_recv << endl;
	port_from_recv = (unsigned short) RECEIVER_TO_SENDER_PORT;
	cout << "From receiver port: " << port_from_recv << endl << endl;
	
	UDP_to_recv = new UDPSocket();
	UDP_to_recv->setNonblocking();
	UDP_from_recv = new UDPSocket(port_from_recv);
	UDP_from_recv->setNonblocking();
}

void msgHeaderInit() {
	for (int k=0; k<2; k++) {
		msgHeader.delx[k] = 0;
		msgHeader.dely[k] = 0;
		msgHeader.delz[k] = 0;
		msgHeader.delyaw[k] = 0;
		msgHeader.delpitch[k] = 0;
		msgHeader.delroll[k] = 0;
		msgHeader.buttonstate[k] = 0;
	}
	msgHeader.Ox12345678 = 0x12345678;
	msgHeader.pactyp = 1;
	msgHeader.version = 43;
	msgHeader.master_id = 1123581321;
	msgHeader.sequence = getSequenceFromFile() + 1;
	msgHeader.surgeon_mode=SURGEON_ENGAGED;
}

void update_UDP_to_slave(double pos[2][3], double rot[2][3], int bttn[2]) {
	static int i, j, k;
	Matrix3d rot_m[2];

	for (k=0; k<2; k++) {
		msgHeader.delx[k] = int(round((pos[k][0]-position[k][0])*THOUSAND));
		msgHeader.dely[k] = int(round((pos[k][1]-position[k][1])*THOUSAND));
		msgHeader.delz[k] = int(round((pos[k][2]-position[k][2])*THOUSAND));
		msgHeader.delyaw[k] = int(round((rot[k][0]-rotation[k][0])*MILLION));
		msgHeader.delpitch[k] = int(round((rot[k][1]-rotation[k][1])*MILLION));
		msgHeader.delroll[k] = int(round((rot[k][2]-rotation[k][2])*MILLION));
		
		for (i=0; i<3; i++) {
			position[k][i] = pos[k][i];
			rotation[k][i] = rot[k][i];
		}
		
		rot_m[k] = Eigen::AngleAxisd(rot[k][0], Vector3d::UnitX()) 
							 * Eigen::AngleAxisd(rot[k][1], Vector3d::UnitY()) 
							 * Eigen::AngleAxisd(rot[k][2], Vector3d::UnitZ());
							 
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				msgHeader.rotation_matrix[k][i][j] = (int) (rot_m[k][j*3 + i]*MILLION);
			}
		}
		
		msgHeader.buttonstate[k] = bttn[k];
	}
	
	msgHeader.sequence++;
	msgHeader.checksum=checksumUDPData();	
}

bool recv_UDP_from_cmd() {
	string addr;
	unsigned short port;
	int ret;
	try {
		ret = UDP_from_cmd->recvFrom(&cmd_to_sender, sizeof(cmd2sender), addr, port);
	}	catch (SocketException SE){
		cout << "failed to recv_UDP from cmd\n";
	}
	return (ret == sizeof(cmd2sender));
}

void send_UDP_to_slave() {
	try {
		UDP_to_slave->sendTo(&msgHeader,sizeof(u_struct),addr_Robot,port_Robot);
	}	catch (SocketException SE) {
		cerr << "failed to send_UDP\n";
	}
}

bool recv_UDP_from_recv() {
	string addr;
	unsigned short port;
	int ret;
	try {
		ret = UDP_from_recv->recvFrom(&receiver_to_sender, sizeof(receiver2sender), addr, port);
	}	catch (SocketException SE){
		cout << "failed to recv_UDP from cmd\n";
	}
	return (ret == sizeof(receiver2sender));
}

void send_UDP_to_recv() {
	try {
		UDP_to_recv->sendTo(&sender_to_receiver,sizeof(sender2receiver),addr_to_recv,port_to_recv);
	}	catch (SocketException SE) {
		cerr << "failed to send_UDP\n";
	}
}

bool msgHeaderZero() {
	bool result = true;
	for (int k=0; k<2; k++) {
		result = result && msgHeader.delx[k]==0.0 && msgHeader.dely[k]==0.0 && msgHeader.delz[k]==0.0 && msgHeader.delyaw[k]==0.0 && msgHeader.delpitch[k]==0.0 && msgHeader.delroll[k]==0.0;
	}
	return result;
}

int checksumUDPData() {
	int chk=0;
	chk =  (msgHeader.surgeon_mode);
	chk += (msgHeader.delx[0])+(msgHeader.dely[0])+(msgHeader.delz[0]);
	chk += (msgHeader.delx[1])+(msgHeader.dely[1])+(msgHeader.delz[1]);
	chk += (msgHeader.delyaw[0])+(msgHeader.delpitch[0])+(msgHeader.delroll[0]);
	chk += (msgHeader.delyaw[1])+(msgHeader.delpitch[1])+(msgHeader.delroll[1]);
	chk += (msgHeader.buttonstate[0]);
	chk += (msgHeader.buttonstate[1]);
	chk += (int)(msgHeader.sequence);
	return chk;
}

int main(int argc, char* argv[]) {
	//sender2receiver.received = false;
	sender_to_receiver.quit = false;
	
	socketInit();
	msgHeaderInit();
	cout << "Debug: initial msgHeader.sequence is " << msgHeader.sequence << endl;
	
	bool received = false;
	char time_s[25];
	
	logOut = new Log("logs/out");
	timestamp(time_s);
	logOut->Write("[%s] delx[0] dely[0] delz[0] delyaw[0] delpitch[0] delroll[0] buttonstate[0] grasp[0] rotation_matrix[0][0][0] rotation_matrix[0][0][1] rotation_matrix[0][0][2] rotation_matrix[0][1][0] rotation_matrix[0][1][1] rotation_matrix[0][1][2] rotation_matrix[0][2][0] rotation_matrix[0][2][1] rotation_matrix[0][2][2] delx[1] dely[1] delz[1] delyaw[1] delpitch[1] delroll[1] buttonstate[1] grasp[1] rotation_matrix[1][0][0] rotation_matrix[1][0][1] rotation_matrix[1][0][2] rotation_matrix[1][1][0] rotation_matrix[1][1][1] rotation_matrix[1][1][2] rotation_matrix[1][2][0] rotation_matrix[1][2][1] rotation_matrix[1][2][2] sequence pactyp version 00master_id surgeon_mode checksum\n", time_s);
	
	while (true) {
		for (int i=0; i<10; i++) {
			if (received = recv_UDP_from_cmd()) {
				update_UDP_to_slave(cmd_to_sender.pos, cmd_to_sender.rot, cmd_to_sender.bttn);
				break;
			} else if (i==9) {
				update_UDP_to_slave(position, rotation, buttonstate);
				break;
			}
		}
		
		if (received) {
			
			if (cmd_to_sender.quit == true) {
				setSequenceToFile(msgHeader.sequence);			
				sender_to_receiver.quit = true;
				bool answer;
				for (int i=0; i<10; i++) {
					send_UDP_to_recv();
					usleep(1000);
					for (int j=0; j<10; j++) {
						if (answer = (recv_UDP_from_recv() && receiver_to_sender.received)) {	break; }
					}
					if (answer) { break; }
				}
				if (!answer)
					cout << "Sender coudln't send a quit signal to Receiver." << endl;
				break;
			}			
			
			if (strcmp(cmd_to_sender.str_out, "") != 0) {
				timestamp(time_s);
				logOut->Write("[%s] # %s", time_s, cmd_to_sender.str_out);
				cout << "logging ... " << cmd_to_sender.str_out << endl;
			}
			if (strcmp(cmd_to_sender.str_in, "") != 0) {
				strcpy(sender_to_receiver.str_in, cmd_to_sender.str_in);
				bool answer;
				for (int i=0; i<10; i++) {
					send_UDP_to_recv();
					usleep(1000);
					for (int j=0; j<10; j++) {
						if (answer = (recv_UDP_from_recv() && receiver_to_sender.received)) {	break; }
					}
					if (answer) { break; }
				}
			}			
		
			timestamp(time_s);
			logOut->Write("[%s] %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %u %u %u %u %d %d\n", 
										time_s,
									 	msgHeader.delx[0], msgHeader.dely[0], msgHeader.delz[0],
									 	msgHeader.delyaw[0], msgHeader.delpitch[0], msgHeader.delroll[0], 
									 	msgHeader.buttonstate[0], msgHeader.grasp[0],
									 	msgHeader.rotation_matrix[0][0][0], msgHeader.rotation_matrix[0][0][1], msgHeader.rotation_matrix[0][0][2], 
									 	msgHeader.rotation_matrix[0][1][0], msgHeader.rotation_matrix[0][1][1], msgHeader.rotation_matrix[0][1][2], 
									 	msgHeader.rotation_matrix[0][2][0], msgHeader.rotation_matrix[0][2][1], msgHeader.rotation_matrix[0][2][2], 
									 	msgHeader.delx[1], msgHeader.dely[1], msgHeader.delz[1], 
									 	msgHeader.delyaw[1], msgHeader.delpitch[1], msgHeader.delroll[1], 
									 	msgHeader.buttonstate[1], msgHeader.grasp[1],
									 	msgHeader.rotation_matrix[1][0][0], msgHeader.rotation_matrix[1][0][1], msgHeader.rotation_matrix[1][0][2], 
									 	msgHeader.rotation_matrix[1][1][0], msgHeader.rotation_matrix[1][1][1], msgHeader.rotation_matrix[1][1][2], 
									 	msgHeader.rotation_matrix[1][2][0], msgHeader.rotation_matrix[1][2][1], msgHeader.rotation_matrix[1][2][2],
									 	msgHeader.sequence, msgHeader.pactyp, msgHeader.version, msgHeader.master_id, msgHeader.surgeon_mode, msgHeader.checksum);

			for (int k=0; k<2; k++) {
				cout << "---------------------------------------------- From Sender to Slave (" << k << ") is ----------------------------------------------" << endl;
				cout << "delx\t" << msgHeader.delx[k] << "\t\tdelyaw\t\t" << msgHeader.delyaw[k] << endl;
				cout << "dely\t" << msgHeader.dely[k] << "\t\tdelpitch\t" << msgHeader.delpitch[k] << endl;
				cout << "delz\t" << msgHeader.delz[k] << "\t\tdelroll\t\t" << msgHeader.delroll[k] << endl;
				cout << "rotation_matrix";
				cout << "\t\t" << msgHeader.rotation_matrix[k][0][0] << "\t" << msgHeader.rotation_matrix[k][0][1] << "\t" << msgHeader.rotation_matrix[k][0][2] << endl;
				cout << "\t\t\t" << msgHeader.rotation_matrix[k][1][0] << "\t" << msgHeader.rotation_matrix[k][1][1] << "\t" << msgHeader.rotation_matrix[k][1][2] << endl;
				cout << "\t\t\t" << msgHeader.rotation_matrix[k][2][0] << "\t" << msgHeader.rotation_matrix[k][2][1] << "\t" << msgHeader.rotation_matrix[k][2][2] << endl;
				cout << "buttonstate\t" << msgHeader.buttonstate[k] << endl;
			}
		  cout << "-------------------------------------------------------------------------------------------------------------------------" << endl;
		  
		  for (int k=0; k<2; k++) {
				cout << "---------------------------------------------- From Cmd to Sender (" << k << ") ----------------------------------------------" << endl;
				cout << "pos_x\t" << cmd_to_sender.pos[k][0] << "\t\tyaw\t\t" << cmd_to_sender.rot[k][0] << endl;
				cout << "pos_y\t" << cmd_to_sender.pos[k][1] << "\t\tpitch\t\t" << cmd_to_sender.rot[k][1] << endl;
				cout << "pos_z\t" << cmd_to_sender.pos[k][2] << "\t\troll\t\t" << cmd_to_sender.rot[k][2] << endl;
				cout << "bttn\t" << cmd_to_sender.bttn[k] << endl;
			}
			cout << "str_in\t" << cmd_to_sender.str_in << endl;
			cout << "str_out\t" << cmd_to_sender.str_out << endl;
			cout << "--------------------------------------------------------------------------------------------------------------------" << endl;
		}
		
		usleep(1000);
		send_UDP_to_slave();
	}
	return 0;
}
