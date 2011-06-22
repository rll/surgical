/* thread_socket_interface.cpp */

#include "thread_socket_interface.h"

UDPSocket receiver = UDPSocket();
UDPSocket sender;
string buf;
char* outputString = "HEAD%d,%.4f,%.4f,%.4f,%d,TAIL\nHEAD%d,%.4f,%.4f,%.4f,%d,TAIL";
vector<string> vectbuf, vect1, vect2;
int i,j;

void parse(string buf, vector<string> &vect) {
  int front, back;
  // get data segment
  front = buf.find("HEAD") + 6;
  back = buf.find("TAIL", front);
  string data = buf.substr(front, back-front-1);
  
  boost::split(vect, data, boost::is_any_of(","));
}

void connectionInit() {
  // Initialize sockets
  if(! (receiver.create())) {
		cout << "Error creating socket!" << endl;
		exit(-1);
	}
	if(! (receiver.bind(RECEIVE_PORT))) {
		cout << "Client: error connecting to ip: " << IP << "  port: " << RECEIVE_PORT << endl;
		exit(-1);
	}
  //receiver.set_non_blocking(true);
  receiver.set_timeout(0);             // using recv()    
  
  if(! (sender.create())) {
		cout << "Error creating socket!" << endl;
		exit(-1);
	}
	if(! (sender.bind(SEND_PORT))) {
		cout << "Client: error connecting to ip: " << IP << "  port: " << SEND_PORT << endl;
		exit(-1);
	}
	if(! (sender.setDestination(IP, SEND_PORT))) {
		cout << "error setting destination" << endl;
		exit(-1);
	}
	//sender.set_non_blocking(true);
	sender.set_timeout(0);    
}

void getDeviceState (double start_proxyxform[], bool start_proxybutton[], double end_proxyxform[], bool end_proxybutton[]) {
  if (receiver.recv(buf)) {
    while (receiver.recv(buf));
    
    //cout << buf << endl;
    
    boost::split(vectbuf, buf, boost::is_any_of("\n"));
    parse(vectbuf[0], vect1);
    parse(vectbuf[1], vect2);
    /*
    int i;
    for (i=0;i<vect1.size();i++)
    cout << vect1[i] << endl;
    for (i=0;i<vect1.size();i++)
    cout << vect2[i] << endl;
    */
    // Extract transform matrix
    for (i=0; i<16; i++) {
        start_proxyxform[i] = boost::lexical_cast<double>(vect1[i].c_str());
        end_proxyxform[i]   = boost::lexical_cast<double>(vect2[i].c_str());
    }
    //cout << vect1[16] << "\t" << vect2[16] << endl;
    start_proxybutton[0] = boost::lexical_cast<int>(vect1[16].c_str());
    start_proxybutton[1] = boost::lexical_cast<int>(vect1[17].c_str());
    end_proxybutton[0]  = boost::lexical_cast<int>(vect2[16].c_str());
    end_proxybutton[1]  = boost::lexical_cast<int>(vect2[17].c_str());
  }
}

void sendDeviceState (double start_feedback_pos[], bool start_feedback_enabled, double end_feedback_pos[], bool end_feedback_enabled) {
	char buf[1024];
	memset(&buf, 0, sizeof(buf));
	sprintf((char*)&buf, outputString, 0, start_feedback_pos[0], start_feedback_pos[1], start_feedback_pos[2], start_feedback_enabled ? 1 : 0,
																		 1, end_feedback_pos[0],   end_feedback_pos[1],   end_feedback_pos[2],   end_feedback_enabled   ? 1 : 0);
	sender.send(string(buf));
}

/*// (1) right         (2) left
void getDeviceState (Vector3d &leftPosition, Matrix3d &leftRotation, Vector3d &rightPosition, Matrix3d &rightRotation) {
    if (receiver.recv(buf)) {
        while (receiver.recv(buf));
    
        boost::split(vectbuf, buf, boost::is_any_of("\n"));
        parse(vectbuf[0], vect1);
        parse(vectbuf[1], vect2);
        */ 
       /*
        // Build transformation matrix
        for (i=0; i<4; i++) {
            for (j=0; j<4; j++) {
                t1[i][j] = boost::lexical_cast<double>(vect1[i*4 + j].c_str());
                t2[i][j] = boost::lexical_cast<double>(vect2[i*4 + j].c_str());
            }
        }
        
        // Extract rotation matrix from transformation matrix
        t1.getRotationMatrix(r1);, bool start_force_feedback
        t2.getRotationMatrix(r2);
        for (i=0; i<3; i++) {
            for (j=0; j<3; j++) {
                leftRotation (i,j)  = r2[i][j];
                rightRotation (i,j) = r1[i][j];
            }
        }
        
        // Extract position vector from transformation matrix
        for (i=0; i<3; i++) {
            leftPosition(i)  = t2[3][i];
            rightPosition(i) = t1[3][i];
        }
        */
			/*
        // Extract rotation matrix from transformation matrix
        for (i=0; i<3; i++) {
            for (j=0; j<3; j++) {
                leftRotation(i,j) = boost::lexical_cast<double>(vect2[i*3 + j].c_str());
                rightRotation(i,j) = boost::lexical_cast<double>(vect1[i*3 + j].c_str());
            }
        }
        
        // Extract position vector from transformation matrix
        for (j=0; j<3; j++) {
            leftPosition(j)  = boost::lexical_cast<double>(vect2[9 + j].c_str());
            rightPosition(j) = boost::lexical_cast<double>(vect1[9 + j].c_str());
        }
        */
        /*
        for (i=0; i<3; i++)
            leftPosition(i) = boost::lexical_cast<double>(vect2[i+9].c_str());
        for (i=0; i<3; i++)
            leftRotation(0,i) = boost::lexical_cast<double>(vect2[i].c_str());
        for (i=0; i<3; i++)
            leftRotation(1,i) = boost::lexical_cast<double>(vect2[i+3].c_str());
        for (i=0; i<3; i++)
            leftRotation(2,i) = boost::lexical_cast<double>(vect2[i+6].c_str());
        //leftRotation.transpose();
            
        for (i=0; i<3; i++)
            rightPosition(i) = boost::lexical_cast<double>(vect1[i+9].c_str());
        for (i=0; i<3; i++)
            rightRotation(0,i) = boost::lexical_cast<double>(vect1[i].c_str());
        for (i=0; i<3; i++)
            rightRotation(1,i) = boost::lexical_cast<double>(vect1[i+3].c_str());
        for (i=0; i<3; i++)
            rightRotation(2,i) = boost::lexical_cast<double>(vect1[i+6].c_str());
        //rightRotation.transpose();
        */
    /*}
}*/
