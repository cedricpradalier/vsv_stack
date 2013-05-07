
#include "bsplines/SplinePath.h"

using namespace splines;

int main() {

	double sampling = 0.1;
	double L;
	double V,V0,A,B;
	Function s_of_t, v_of_t;
	bool res,stop;

	// Positive speed
	L=20.; V0 = 0.0; V = 1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling);
	printf("Test 1: %s\n",res?"success":"failure");
	s_of_t.print("S1");
	v_of_t.print("V1");

	L=5.; V0 = 0.0; V = 1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling);
	printf("Test 2: %s\n",res?"success":"failure");
	s_of_t.print("S2");
	v_of_t.print("V2");

	L=20.; V0 = 0.2; V = 1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling);
	printf("Test 3: %s\n",res?"success":"failure");
	s_of_t.print("S3");
	v_of_t.print("V3");

	L=5.; V0 = 0.1; V = 1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling);
	printf("Test 4: %s\n",res?"success":"failure");
	s_of_t.print("S4");
	v_of_t.print("V4");

	L=20.; V0 = 1.5; V = 1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling);
	printf("Test 5: %s\n",res?"success":"failure");
	s_of_t.print("S5");
	v_of_t.print("V5");

	L=5.; V0 = 1.5; V = 1.0; A = 0.1; B = 0.25; stop = false;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling);
	printf("Test 6: %s\n",res?"success":"failure");
	s_of_t.print("S6");
	v_of_t.print("V6");

	L=5.; V0 = 1.5; V = 1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling);
	printf("Test 7: %s\n",res?"success":"failure");
	s_of_t.print("S7");
	v_of_t.print("V7");

	L=2.; V0 = 1.5; V = 1.0; A = 0.1; B = 0.25; stop = false;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling);
	printf("Test 8: %s\n",res?"success":"failure");
	s_of_t.print("S8");
	v_of_t.print("V8");

	L=4.; V0 = 1.5; V = 1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling);
	printf("Test 9: %s\n",res?"success":"failure");
	s_of_t.print("S9");
	v_of_t.print("V9");

	// Negative speed
	L=20.; V0 = 0.0; V = -1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling).map(fabs);
	printf("Test -1: %s\n",res?"success":"failure");
	s_of_t.print("S-1");
	v_of_t.print("V-1");

	L=5.; V0 = 0.0; V = -1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling).map(fabs);
	printf("Test -2: %s\n",res?"success":"failure");
	s_of_t.print("S-2");
	v_of_t.print("V-2");

	L=20.; V0 = -0.2; V = -1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling).map(fabs);
	printf("Test -3: %s\n",res?"success":"failure");
	s_of_t.print("S-3");
	v_of_t.print("V-3");

	L=5.; V0 = -0.1; V = -1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling).map(fabs);
	printf("Test -4: %s\n",res?"success":"failure");
	s_of_t.print("S-4");
	v_of_t.print("V-4");

	L=20.; V0 = -1.5; V = -1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.map(fabs).primitive(0,sampling);
	printf("Test -5: %s\n",res?"success":"failure");
	s_of_t.print("S-5");
	v_of_t.print("V-5");

	L=5.; V0 = -1.5; V = -1.0; A = 0.1; B = 0.25; stop = false;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.map(fabs).primitive(0,sampling);
	printf("Test -6: %s\n",res?"success":"failure");
	s_of_t.print("S-6");
	v_of_t.print("V-6");

	L=5.; V0 = -1.5; V = -1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.map(fabs).primitive(0,sampling);
	printf("Test -7: %s\n",res?"success":"failure");
	s_of_t.print("S-7");
	v_of_t.print("V-7");

	L=2.; V0 = -1.5; V = -1.0; A = 0.1; B = 0.25; stop = false;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.map(fabs).primitive(0,sampling);
	printf("Test -8: %s\n",res?"success":"failure");
	s_of_t.print("S-8");
	v_of_t.print("V-8");

	L=4.; V0 = -1.5; V = -1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.map(fabs).primitive(0,sampling);
	printf("Test -9: %s\n",res?"success":"failure");
	s_of_t.print("S-9");
	v_of_t.print("V-9");

	// Change of speed
	L=20.; V0 = -0.5; V = 1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling).map(fabs);
	printf("Test A: %s\n",res?"success":"failure");
	s_of_t.print("SA");
	v_of_t.print("VA");

	L=5.; V0 = -0.5; V = 1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling).map(fabs);
	printf("Test B: %s\n",res?"success":"failure");
	s_of_t.print("SB");
	v_of_t.print("VB");

	L=20.; V0 = 0.5; V = -1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling).map(fabs);
	printf("Test C: %s\n",res?"success":"failure");
	s_of_t.print("SC");
	v_of_t.print("VC");

	L=5.; V0 = 0.5; V = -1.0; A = 0.1; B = 0.25; stop = true;
	res = buildTraversalTimeMaps(L,V0,V,B,A,stop,v_of_t);
	s_of_t = v_of_t.primitive(0,sampling).map(fabs);
	printf("Test D: %s\n",res?"success":"failure");
	s_of_t.print("SD");
	v_of_t.print("VD");


	return 0;

}

