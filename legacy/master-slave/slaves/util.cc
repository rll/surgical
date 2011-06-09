#include "util.h"
// Mathematic utilities
double sq(double x) {
    return x*x;
}

double angle_law_of_cos(double a, double b, double c) {
  return acos( (sq(a) + sq(b) - sq(c)) / (2*a*b) );
}

double side_law_of_cos(double a, double b, double angle) {
  return sqrt( sq(a) + sq(b) - 2*a*b * cos(angle) );
}

int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

int _getch(void) {
    struct termios oldt, newt;
    int ch;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
    return ch;
}

void getperm(int addr, int num)
{
  if(ioperm(addr, num, 1)) {
    printf("Access denied, quitting.\n");
    exit(1);
  }
}

std::string build_cmd(const char* base, const char channel, const char type, double arg, const char argtype){
    if (channel < 'A' || channel > 'H') {
        throw "Channel out of bounds in build_cmd\n";
    }
    char command[100];
    if (type == 'c') {
        char commas[8];
        commas[0] = '\0';
        for(int i=0;i<channel-'A'; i++) {
            commas[i] = ',';
            commas[i+1] = '\0';
        }
        if (argtype == 'i') {
            sprintf(command, "%s %s%d", base, commas, (int) arg);
        } else if (argtype == 'd') {
            sprintf(command, "%s %s%.3f", base, commas, arg);
        } else {
            sprintf(command, "ERROR");
        }
    } else if (type == 'l') {
        sprintf(command, "%s%c", base, channel);
    } else {
        sprintf(command, "ERROR");
    }
    return std::string(command);
}


void miranova_init_channel(int channel, long ncount, int quadmode) {
	unsigned char quad, pr0, pr1, pr2;

	if(quadmode == 4) quad = 0xC3;
	else quad = 0xC0 | (quadmode & 0x03);

	pr0 = (char) (ncount & 0x0000FFL);
	pr1 = (char) ((ncount & 0x00FF00L) >> 8);
	pr2 = (char) ((ncount & 0xFF0000L) >> 16);

	int addr = MIRANOVA_BASE + channel * 4;

	getperm(addr, 4);
	outb(0x20, addr+1);		// Master reset MCR channel 1								 
	outb(0x79, addr+1);		// Initialize counter inputs, ICR channel 1	 
	outb(0x80, addr+1);		// Initialize counter outputs, OCCR channel 1

	outb(quad, addr+1);		// Select quadrature register, QR channel 1 

	//	Enable quadrature mode.	0xC1, 0xC2 and 0xC3 for x1, x2 and x4 resp.
	outb(0x01, addr+1);		// Reset pr address pointer
	outb(pr0, addr);		// Load pr0	 LSB
	outb(pr1, addr);		// Load pr1	 MID
	outb(pr2, addr);		// Load pr2	 MSB
	outb(0x08, addr+1);		// Transfer pr to CNTR
}

long miranova_read(int channel, int &status) {
	int addr = MIRANOVA_BASE + channel*4;
	long ol0, ol1, ol2;		 // Output latch byte variables

	getperm(addr, 3);
	outb(0x03, addr+1);		// Begin read count, MCR channel 1
	ol0 = inb(addr);		// Read ol0 
	ol1 = inb(addr);		// Read ol1
	ol2 = inb(addr);		// Read ol2
	status = inb(addr+1);	// Read status byte
	return ( ol0 | (ol1<<8) | (ol2<<16) );
}

/* low-level quatech read/write */
void quatech_outdac8_s(int channel, int value, int base)
{
    
    // get permission to write between QUATECH_DABASE1 and
    // QUATECH_DABASE1+4
    getperm(base,4);

    // write the channel number to base+1
    outb(channel, base + 1);

    // write the voltage value to base
    outb(value, base + 0);

    // ?
    outb(0x0, base + 2);
    outb(0x1, base + 2);
}


void quatech_vout(int channel, double volt, int slave_num) 
{
  int val = (int)(1.0 + 255.0 * (5.0-volt) / 10.0);
  int base;
  // Assumed only 2 slaves, slave1 and slave2
  if (slave_num == 1)
    base = QUATECH_DABASE1;
  else if (slave_num == 2)
    base = QUATECH_DABASE2;
  quatech_outdac8_s(channel, (val > 255) ? 255 : ((val<0)? 0 : val), base);
}


double timediff(timeval now, timeval then) {
    return (((double) now.tv_sec) * 1000000 + now.tv_usec) - (((double) then.tv_sec) * 1000000 + then.tv_usec);
}

Timer::Timer() {
    gettimeofday(&last_call, NULL);
}

Timer::~Timer() { }

double Timer::dt() {
    timeval now;
    gettimeofday(&now, NULL);
    double out = timediff(now, last_call);
    last_call = now;
    return out;
}

Time_file::Time_file(const char* filename, int d_size) : std::ifstream(filename) {
    remainder = -1;
    data_size = d_size;
    last_load = -1;  // TIME IN FILE STARTS AT 0 OR ELSE
}

Time_file::~Time_file() { }

std::vector<double> Time_file::data() {
    if(last_load == -1) {
        gettimeofday(&last_call, NULL);
    }
    while (update_now()) {
        //printf("remainder: %f\n", remainder);
        get_data();
    }
    return last_data;
}

void copy_pos(const double* original, double* copy){
    for(int i = 0; i < num_dof; i++){
        copy[i] = original[i];
    }
}

void add_pos(const double* toAdd1, const double* toAdd2, double* sum){
    for(int i = 0; i < num_dof; i++){
        sum[i] = toAdd1[i] + toAdd2[i];
    }
}

void Time_file::get_data() {
    std::string s;
    std::getline(*this, s);
    std::stringstream ss(s);
    //printf("%s\n", ss.str().c_str());
    //printf("get_data called\n");
    if (!ss.eof() && !(this->eof()) ) {
        //printf("loading new data\n");
        double clock; 
        ss >> clock;
        //printf("clock %f\n", clock);
        //getchar();
        remainder   += clock - last_load;
        last_load   = clock;
        
        last_data.clear();
        for(int i=0;i<data_size;i++) {
            double d;
            ss >> d;
            last_data.push_back(d);
        }
    } else {
        remainder += std::numeric_limits<double>::max();
    }
}

bool Time_file::update_now() {
    timeval now;
    gettimeofday(&now, NULL);
    
    remainder -= timediff(now, last_call);
    last_call = now;
    return remainder < 0;
}

void Time_file::reset_time(){
    last_load = -1;
    remainder = -1;
}


