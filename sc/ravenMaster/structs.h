struct cmd2sender {
	double pos[2][3];
	double rot[2][3];
	int bttn[2];
	char str_out[2048];
	char str_in[2048];
	bool quit;
};

struct sender2receiver {
	char str_in[2048];
	bool received;
	bool quit;
};

struct receiver2sender {
	bool received;
};
