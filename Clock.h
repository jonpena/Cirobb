
#ifndef CLOCK_H
#define CLOCK_H

#include <chrono>

using namespace std::chrono;
typedef std::chrono::high_resolution_clock clock_time;
typedef std::chrono::nanoseconds clock_freq;

class Clock
{

private:

	clock_time::time_point t_start;
	clock_time::time_point t_current;

public:

	//Constructor by Default of the Class Clock
	Clock(void);

	//Establece El Tiempo Transcurrido En La Variable
	void Elapsed(float&);
	
	//Establece El Valor Minimo y Maximo Que La Variable Puede Obtener
	void Clamp(const float&, const float&, float&);
};

#endif