#include <iostream>
#include "tools.h"

Tools::Tools() {
	f_log.open("log.csv", std::ios_base::out);
	f_log<<"time,px,py,psi,v,cte,epsi,steer,throttle"<<std::endl;
}

Tools::~Tools() {}

void Tools::Log(double px,double py,double psi,double v,double cte,double epsi,double steer,double throttle){
	struct timespec now;
	timespec_get(&now, TIME_UTC);
	char buff1[50];
	sprintf (buff1, ".%09ld", now.tv_nsec);
	std::string nano_sec_string(buff1);
	char buff2[100];
	strftime(buff2, sizeof buff2, "%D %T", gmtime(&now.tv_sec));
	std::string time_string(buff2);
	time_string = time_string + nano_sec_string;

	f_log<<time_string<<","<<px<<","<<py<<","<<psi<<","<<v<<","<<cte<<","<<epsi<<","
		<<steer<<","<<throttle<<std::endl;

}

