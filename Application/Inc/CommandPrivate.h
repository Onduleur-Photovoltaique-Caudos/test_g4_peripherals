#pragma once

void statusDisplay(bool bHeader);
void measurementsDisplay(int val1, int val2);

typedef enum _command_t {
none,
st,
rt,
tt,
ac,
ar,
sm,
po,
fa,
z1,
z2,
d1,
d2,
t1,
t2,
v1,
v2
 } command_t;

const command_t firstCompositeCommand = z1;
const command_t commandArray[] =  {
none,
st,
rt,
tt,
ac,
ar,
sm,
po,
fa, // end of simple commands
z1, // start of composite commands
z2,
d1,
d2,
t1,
t2,
v1,
v2
};
const char * simpleCommandListStr[] = {
"no",
"st",
"rt",
"tt",
"ac",
"ar",
"sm",
"po",
"fa"
};
command_t command;
/* Commands 2 alphanumerical, = sign, integer
st=  // display status
tt=  // display temperatures
ac = {0,1} // generate sinewave
ar = {0,1} // 0=breaker test  1=rearm breaker
sm= report measurements
po=n    max power
rt=n    ratio
z1=n zvs pulse width ns for low side (going up)
z2=n zvs pulse width in ns for hi side (going down)
d1=n low side pulse delay  (negative means overlap)
d2=n hi side pulse delay
t1=n low side trailing edge delay (negative means shorter trailing edge)
t2=n hi side trailing edge delay
v1=n low side zvs voltage (225 V)
v2=n hi size zvs voltage (175 V)
*/
class commandAndValue{
public:
commandAndValue(command_t command, int value)
: command(command)
, value(value)
{
}
command_t command;
int value;
};


