#include <stdio.h>
#include "../includes/gpio.h"
#define DIRECTION "in"
#define STATE 0
#define EDGE  "none"

gpio gpio27(27,DIRECTION,STATE,EDGE);

int main(){
	printf("\n%d\n",gpio27.getvalue());
	return 0;
}
