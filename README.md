
The following addresses each of the rubric points. 

### Code Compiles 

Sure, enough! Just try it! I would never, ever, ever push code that doesn't compile. I am not a monster. 

### PID procedure implementation is standard 

This is implemented in `PID.cpp`, methods `PID::UpdateError`  and `PID::TotalError`. Nothing surprising there. 

## Reflection 

### What each of the P, I , D components does. 

For some reason I wasn't able to record a video from the simulator, even though I tried ... :( 
When just using the P component, the car starts oscillating  around the line path parallel to the center of the road, the higher the value of P, the bigger the  higher the frequency in the oscilations. 

The D component causes a damping of the oscillations. The higher the value of D, the faster the damping tends to happens. 

The effect of the I component is a lot more subtle and it affects the longer term average deviation of the car from the center of the road. 
Due to errors in the finetuning of the wheels there might be a systematic deviation in the angle that needs to be compensated by this term. 

A non optimal value of the I component causes the car to follow a path that is parallel to the middle of the road but somewhat deviated from it. 

### How the hyperparameters where choosen 

I wrote a somewhat involved implementation of the Twiddle hyper-parameter optimization algorithm from Sebastian's lecture.
You can find said implementation in `TwiddleOpt.h`. The meat of the logic is in method `eval_result` which is meant to be called 
periodically by `main.cpp` to get a new set of parameter values with which to drive after each call. 

The initiall call  (`main.cpp`, line 158) simply gets the initial set of parameters. 

After that, `main.cpp` calls `TwiddleOpt::eval_result` only after the car has travelled 10000 "meters"  (which are measured by integrating
the sampled speeds from the update messages). This happens on line 95. 
During those 10000 "meters", we measure the average absolute CTE ( by simply summing `abs(cte)` over 
all messages and diving by number of messages) as well as the average speed. Our objective function that TwiddleOpt attempts to minimize 
is the result of deviding the mean average cte by the average speed. 

After the call, the distance counter is reset, so as to let the car travel another 10000 meters. 

The results of many laps with many different sets of parameters can be perused the tab separated data file `record_varthr.txt`
The columns in this file are `Kp`, `Ki`, `Kd`, `max_throttle`, `msg_count`, `mean_avg_cte`, `distance_travelled`, `avg_speed`.

The max_throttle parameter controls the maximum acceleration that the car has assuming that `cte = 0` and `d(cte)/dt = 0`, any deviation 
from these conditions causes the throttle to decrease significantly and even become negative as a safety measure to avoid falling off the
road to easily. The detailed formula of how the throttle is calculated at every step can be found at. 

### The car is able to drive a lap  around the track 

Calling `pid` with no arguments drives the car with the best parameters I could find : `Kp=0.185, Ki=0.000415, Kd=4.50` and `max_throttle=3.0`.
It is able to drive at least two laps without incident at an average speed of 56 mph.  Just try it. 

I would give you a video but I wasn't able to record it :( 


