#include "utility.h"
#include <iomanip>
#include <cstdlib>

char* time_stamped_string(const char name[150]){
	time_t rawtime;
	struct tm * tinfo;
	time ( &rawtime );
	tinfo = localtime ( &rawtime );
	char *Filename = (char*) malloc(299);
	sprintf(Filename, "%s.%.4d.%.2d.%.2d-%.2d.%.2d.%.2d.log", name, tinfo->tm_year+1900, tinfo->tm_mon+1, tinfo->tm_mday, tinfo->tm_hour, tinfo->tm_min, tinfo->tm_sec);
	return Filename;
}


void sleep_soft_real_time(double start, double end, double dt){
	static int first_time = 1;
	double over_sleep;
	static double over_sleep_old, start_old;
	if(first_time){
		over_sleep = 0;
		first_time = 0;
	}else{
		double diff_start = start-start_old;
		over_sleep = diff_start+over_sleep_old-dt;
	//	std::cout << "start: " << start << "\tstart_old: " << start_old << "\tdiff_start: " << diff_start << "\tover_sleep_old: " << over_sleep_old << std::endl;
	}
	start_old = start;
	over_sleep_old = over_sleep;

	double sleep_time = dt-(end-start)-over_sleep;

	//std::cout << std::setprecision (12) <<"start: " << start << "\tend: " << end << "\tover_sleep: " << over_sleep << "\tsleep: " << sleep_time << "\tend-start: " << end-start << "\n" << std::endl;
	if(sleep_time>0)
		usleep(sleep_time*1e6);
}
