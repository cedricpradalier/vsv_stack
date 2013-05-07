#include <stdlib.h>
#include "bsplines/Function.h"

using namespace splines;

int main() {
	Function f;
	unsigned int i;

	f.set(0,0);
	for (i=0;i<1000;i++) {
		f.set(i/1000.0,i);
	}
	f.set(1,1000);

	printf("F(0.00)   : %f\n",f(0));
	printf("F(0.45)   : %f\n",f(0.45));
	printf("F(1.00): %f\n",f(1.0));

	return 0;

}

