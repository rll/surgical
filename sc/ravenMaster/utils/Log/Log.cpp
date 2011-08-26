#include "Log.h"

Log::Log() {
	char filename[30];
	timestamp(time_s);
  strcpy(filename, time_s);
  strcat(filename, ".log");
  m_stream.open(filename);
}

Log::Log(char* filename, bool concatTimeStamp, bool bin) {
	char newfilename[strlen(filename) + 25];
	strcpy(newfilename, filename);
	if (concatTimeStamp) {
		strcat(newfilename, "_");
		timestamp(time_s);
		strcat(newfilename, time_s);
	}
	strcat(newfilename, ".log");
	if (binary = bin) { 
		m_stream.open(newfilename, ios::out | ios::binary);
	} else {
 		m_stream.open(newfilename);
 	}
}

void Log::WriteBinary(const char* data, unsigned int length) { 
	m_stream.write(data, length); 
}

void Log::Write(const char* logline, ...) {
		va_list argList;
		char cbuffer[5000];
		va_start(argList, logline);
		vsnprintf(cbuffer, 5000, logline, argList);
		va_end(argList);
		m_stream << cbuffer;
		m_stream.flush();
}

Log::~Log() {
  m_stream.close();
}

void timestamp(char* time_s) {
	time_t ltime;
	struct tm *Tm;

	ltime=time(NULL);
	Tm=localtime(&ltime);
	
	sprintf(time_s, "%02d%02d%04d_%02d%02d%02d",
		Tm->tm_mon+1,
		Tm->tm_mday,
		Tm->tm_year+1900,
		Tm->tm_hour,
		Tm->tm_min,
		Tm->tm_sec);
}
