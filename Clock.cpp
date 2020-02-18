#include "Clock.h"


//Constructor by Default Of This Class
Clock::Clock(void)
{
	t_start = clock_time::now();
}


//Sets The time Elapsed Current To Accumulator
void Clock::Elapsed(float& accumulator)
{
	t_current = clock_time::now();
	accumulator += duration_cast<clock_freq>(t_current - t_start).count() / float(duration_cast<clock_freq>(seconds(1)).count());
	t_start = clock_time::now();
	Clamp(0.0f, 0.033f, accumulator); //Minimum 30 fps
}


//Set The Values Minimum and Maximum That The Variable Must Have it.
void Clock::Clamp(const float& min, const float& max, float& a)
{
	if (a < min) a = min;
	if (a > max) a = max;
}