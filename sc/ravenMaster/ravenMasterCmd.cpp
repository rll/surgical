#include "ravenMasterCmd.h"

UDPSocket *UDP_to_sender;
char addr_to_sender[16];
unsigned short port_to_sender;
unsigned short port_from_sender;

cmd2sender cmd_to_sender;

void socketInit() {	
	strcpy(addr_to_sender, SURGICAL5);
	cout << "To sender IP: " << addr_to_sender << endl;
	port_to_sender = (unsigned short) CMD_TO_SENDER_PORT;
	cout << "To sender port: " << port_to_sender << endl;
	port_from_sender = (unsigned short) SENDER_TO_CMD_PORT;
	cout << "From sender port: " << port_from_sender << endl << endl;

	UDP_to_sender = new UDPSocket();
	UDP_to_sender->setNonblocking();
}

void send_UDP_to_sender() {
	try {
		UDP_to_sender->sendTo(&cmd_to_sender,sizeof(cmd2sender),addr_to_sender,port_to_sender);
	}	catch (SocketException SE) {
		cerr << "failed to send_UDP\n";
	}
}

double cast_str(string s) {
  return boost::lexical_cast<double>(s.c_str());
}

int cast_str_int(string s) {
  return boost::lexical_cast<int>(s.c_str());
}

void printHelp() {
	printf("Command Options:\n");
	printf("Command arguments are represented by a parenthesis segment, including both parenthesis.\n");
	printf("Press ENTER after writing a command. Don't use extra spaces.\n\n");
	printf("q                                                                 quit\n");
	printf("h                                                                 print command options\n");
	printf("p                                                                 print current pos and rot (according to master)\n");
	printf("pos (pos0[0]) (pos0[1]) (pos0[2]) (pos1[0]) (pos1[1]) (pos1[2])   update pos0 and pos1 (arguments are double)\n");
	printf("pos0 (pos0[0]) (pos0[1]) (pos0[2])                                update pos0 only\n");
	printf("pos1 (pos1[0]) (pos1[1]) (pos1[2])                                update pos1 only\n");
	printf("rot (rot0[0]) (rot0[1]) (rot0[2]) (rot1[0]) (rot1[1]) (rot1[2])   update rot0 and rot1 (arguments are double)\n");
	printf("rot0 (rot0[0]) (rot0[1]) (rot0[2])                                update rot0 only\n");
	printf("rot1 (rot1[0]) (rot1[1]) (rot1[2])                                update rot1 only\n");
	printf("bttn (bttn0) (bttn1)                                              update bttn0 and bttn1 (arguments are int)\n");
	printf("bttn0 (bttn0)                                                     update bttn0 only\n");
	printf("bttn1 (bttn1)                                                     update bttn1 only\n");
	printf("log (log_text)                                                    log some text into the in and out log file (argument is string)\n");
	printf("logOut (log_text)                                                 log some text into the out log file only\n");
	printf("logIn (log_text)                                                  log some text into the in log file only\n");
	printf("-----------------------------------------------------------------------------------------------------------------\n");
}

void printState() {
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

int main(int argc, char* argv[]) {
	for (int k=0; k<2; k++) {
		for (int i=0; i<2; i++) {
			cmd_to_sender.pos[k][i] = 0.0;
			cmd_to_sender.rot[k][i] = 0.0;
		}
		cmd_to_sender.bttn[k] = 0;
	}
	cmd_to_sender.quit = false;
	string empty_str = "";
	
	socketInit();
	
	string cmd;
	string line;
	vector<string> vect;
	printHelp();
	
	while (true) {
		getline(cin, line);
		
		boost::split(vect, line, boost::is_any_of(" "));
		
		if (vect.size() == 0) {
			cout << "You didn't enter anything. Enter data again." << endl;
			continue;
		}
		
		cmd = vect.front();
		
		if (vect.size() == 1) {
			if (cmd == "quit" || cmd == "q") {
				cmd_to_sender.quit = true;
				strcpy(cmd_to_sender.str_out, empty_str.c_str());
				strcpy(cmd_to_sender.str_in, empty_str.c_str());
				usleep(100);
				send_UDP_to_sender();				
				break;
			} else if (cmd == "print" || cmd == "p") {
				printState();
				continue;
			} else if (cmd == "help" || cmd == "h") {
				printHelp();
				continue;
			}
		}
		
		if ((cmd == "log") || (cmd == "logOut") || (cmd == "logIn")) {
			if (cmd == "log") {
				string line_cmdless = line.substr(4);				
				strcpy(cmd_to_sender.str_out, line_cmdless.c_str());
				strcpy(cmd_to_sender.str_in, line_cmdless.c_str());
			} else if (cmd == "logOut") {
				string line_cmdless = line.substr(7);
				strcpy(cmd_to_sender.str_out, line_cmdless.c_str());
				strcpy(cmd_to_sender.str_in, empty_str.c_str());
			} else if (cmd == "logIn") {
				string line_cmdless = line.substr(6);
				strcpy(cmd_to_sender.str_out, empty_str.c_str());
				strcpy(cmd_to_sender.str_in, line_cmdless.c_str());
			}
			printState();
			usleep(100);
			send_UDP_to_sender();
			continue;
		} else {
			strcpy(cmd_to_sender.str_out, empty_str.c_str());
			strcpy(cmd_to_sender.str_in, empty_str.c_str());
	  }
		
		if (((cmd=="pos" || cmd=="rot") 																&& (vect.size() != 7)) ||
				((cmd=="bttn")																							&& (vect.size() != 3)) ||
				((cmd=="pos0" || cmd=="pos1" || cmd=="rot0" || cmd=="rot1") && (vect.size() != 4)) ||
				((cmd=="bttn0" || cmd=="bttn1")															&& (vect.size() != 2)) ) {
			cout << "Wrong number of tokens. Enter data again." << endl;
			continue;
		}
		
		try {
			if (cmd == "pos") {
				for (int i=0; i<2; i++)
					for (int j=0; j<3; j++)
						cmd_to_sender.pos[i][j] = cast_str(vect[1+j+3*i]);
			} else if (cmd == "pos0") {
				for (int j=0; j<3; j++)
					cmd_to_sender.pos[0][j] = cast_str(vect[1+j]);
			} else if (cmd == "pos1") {
				for (int j=0; j<3; j++)
					cmd_to_sender.pos[1][j] = cast_str(vect[1+j]);
			
			} else if (cmd == "rot") {
				for (int i=0; i<2; i++)
					for (int j=0; j<3; j++)
						cmd_to_sender.rot[i][j] = cast_str(vect[1+j+3*i]);
			} else if (cmd == "rot0") {
				for (int j=0; j<3; j++)
					cmd_to_sender.rot[0][j] = cast_str(vect[1+j]);
			} else if (cmd == "rot1") {
				for (int j=0; j<3; j++)
					cmd_to_sender.rot[1][j] = cast_str(vect[1+j]);
			
			} else if (cmd == "bttn") {
				for (int i=0; i<2; i++)
					cmd_to_sender.bttn[i] = cast_str_int(vect[1+i]);
			} else if (cmd == "bttn0") {
				cmd_to_sender.bttn[0] = cast_str_int(vect[1]);
			} else if (cmd == "bttn1") {
				cmd_to_sender.bttn[1] = cast_str_int(vect[1]);
			
			} else {
				cout << "Command " << cmd << " not recognized. Enter data again." << endl;
				continue;
			}
		} catch(bad_lexical_cast &) {
			cout << "Input was formatted incorrectly. Enter data again." << endl;
			continue;
		}
		printState();
		usleep(100);
		send_UDP_to_sender();
	}
	return 0;
}
