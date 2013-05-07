
#include "bsplines/SplinePath.h"
#include <set>
#ifdef USE_GNUPLOT
#include <CGnuplot.h>
#endif

using namespace std;
using namespace splines;

SplinePath::SplinePath(const Point * s, unsigned int n, double res, double vel) :
	BSpline(s,n), i_to_s(n)
{
	buildSMap(res);
	velocity_of_s.set(-1e-3,0);
	velocity_of_s.set(0,vel);
	velocity_of_s.set(L,vel);
	velocity_of_s.set(L+1e-3,0);
	buildTMap();
}

SplinePath::SplinePath(const std::vector<Point> & s, double res, double vel) :
	BSpline(s), i_to_s(s.size())
{
	buildSMap(res);
	velocity_of_s.set(-1e-3,0);
	velocity_of_s.set(0,vel);
	velocity_of_s.set(L,vel);
	velocity_of_s.set(L+1e-3,0);
	buildTMap();
}

SplinePath::SplinePath(const Point * s, const double * vel, unsigned int n, double res) :
	BSpline(s,n), i_to_s(n)
{
	unsigned int i;
	buildSMap(res);
	velocity_of_s.set(-1e-3,0);
	for (i=0;i<n;i++) {
		velocity_of_s.set(i_to_s[i],vel[i]);
	}
	velocity_of_s.set(L+1e-3,0);
	buildTMap();
}

SplinePath::SplinePath(const std::vector<Point> & s, const std::vector<double> & vel, double res) :
	BSpline(s), i_to_s(s.size())
{
	unsigned int i;
	buildSMap(res);
	velocity_of_s.set(-1e-3,0);
	for (i=0;i<s.size();i++) {
		velocity_of_s.set(i_to_s[i],vel[i]);
	}
	velocity_of_s.set(L+1e-3,0);
	buildTMap();
}

void SplinePath::printPath(const std::string & filename, double sampling) const
{
	double s;
	FILE * fp = fopen(filename.c_str(),"w");
	if (!fp) return;
	//printf("SplinePath::plot t in [0,%f] l in [0,%f]\n",T,L);
#if 0
    double t;
	for (t=0;t<T+sampling;t+=sampling) {
		s = getAbscissaAtTime(t);
		Point P = value(s);
		double v = velocity(s);
		double h = orientation(s);
		fprintf(fp,"%f %f %f %f %f %f\n",t,s,P.x,P.y,h,v);
	}
#else
	for (s=0;s<L+sampling;s+=sampling) {
		Point P = value(s);
		double v = velocity(s);
		double h = orientation(s);
		fprintf(fp,"%f %f %f %f %f %f\n",s,s,P.x,P.y,h,v);
	}
#endif
	fclose(fp);
}

bool SplinePath::buildSMap(double res)
{
	double u;
	Point P = S[0];
	double s = 0;
	unsigned int k,n=S.size()-1;
	s_to_u.set(0,0);
	i_to_s[0] = 0;
	k = 1;
	res = std::min(res,1.0/(2*n));
	for (u=res;u<1;u+=res) {
		Point Q = BSpline::value(u);
		double ds = (Q-P).norm();
		assert(ds>=0);
		if ((k<S.size()) && (fabs(u - ((double)k)/n) < res/2)) {
			i_to_s[k] = s;
#ifdef DEBUG_SPLINE
			printf("Control %d: s = %f u %f\n",k,s,u);
#endif
			k += 1;
		}
		s += ds;
		s_to_u.set(s,u);
		assert(s>=0);
		P = Q;
	}
	L = s;
	// In case we did not affect it
	i_to_s[S.size()-1] = L;
#if DEBUG_SPLINE
	printf("Control %d: s = %f t %f\n",S.size()-1,L,1.0);
#endif
	resolution = res;
	s_to_u_dot = s_to_u.derivative();
	s_to_u_dotdot = s_to_u_dot.derivative();

#if DEBUG_SPLINE
	Point tP = value(0);
	Point tQ = BSpline::value(0);
	printf("buildSMap: S %f %f Q %f %f | %f P %f %f \n",S[0].x,S[0].y,tQ.x,tQ.y,s_to_u(0), tP.x,tP.y);
#endif
	assert(fabs(s_to_u(0)) < res/10);

	return true;
}

bool SplinePath::buildTMap()
{
	unsigned int k;
	// Compute the time to curvilinear absicca map
	double tcur = 0;
	double scur = 0;
	t_to_s.set(tcur,scur);
#if DEBUG_SPLINE
	printf("k = %d V = %f \n",0,velocity_of_s(0));
#endif
	for (k=1;k<S.size();k++) {
		double s1 = i_to_s[k];
		double vmean = (fabs(velocity_of_s(scur))+fabs(velocity_of_s(s1)))/2;
#if DEBUG_SPLINE
		printf("k = %d V = %f vmean = %f\n",k,velocity_of_s(s1),vmean);
#endif
		if (fabs(vmean)>1e-2) {
			tcur += (s1 - scur)/vmean;
		}
		scur = s1;
		t_to_s.set(tcur,scur);
	}
	T = tcur;
#if DEBUG_SPLINE
	printf("Spline duration: %fs\n",T);
#endif

	return true;
}

double SplinePath::curvature(double s) const
{
	Point V = speed(s);
	Point A = acceleration(s);
	double vel = V.norm();
    if (fabs(vel)>1e-5) {
        return (V.cross(A)).norm() / (vel*vel*vel);
    } else {
        return 0.0;
    }
}

SplinePath SplinePath::resampleByCurvature(double min_inter_point, double max_inter_point, double sampling) const
{
	double l,l0,L=length();
	vector<Point> approx;
	vector<double> appvel;
	approx.push_back(value(0));
	appvel.push_back(velocity_of_s(0));
#if DEBUG_SPLINE
	printf("Decimate, adding %f %f V %f\n",approx.back().x,approx.back().y,appvel.back());
#endif
	l0 = l = 0;
	while (l < L) {
#if DEBUG_SPLINE
		printf("Decimate, sampling %f\n",l);
#endif
		l += sampling;
		double kappa = fabs(curvature(l));
		double d = max_inter_point/kappa;
		if (d > max_inter_point) {
			d = max_inter_point;
		} else if (d < min_inter_point) {
			d = min_inter_point;
		}
		if ((l-l0) > d) {
			l0 = l;
			approx.push_back(value(l));
			appvel.push_back(velocity_of_s(l));
#if DEBUG_SPLINE
			printf("Decimate, adding %f %f V %f\n",approx.back().x,approx.back().y,appvel.back());
#endif
		}
	}
	if ((approx.back()-value(L)).norm() > 1e-3) {
		approx.push_back(value(L));
		appvel.push_back(velocity_of_s(l));
#if DEBUG_SPLINE
		printf("Decimate, adding %f %f V %f\n",approx.back().x,approx.back().y,appvel.back());
#endif
	}
#if DEBUG_SPLINE
	printf("Decimation completed, creating final spline with %d wp\n",approx.size());
#endif
	return SplinePath(approx,appvel,resolution);
}



// Mike implementation
SplinePath SplinePath::decimate(double error_thres, double *error_res) const
{
#ifdef USE_GNUPLOT
	CGnuplot gpl;
#endif

	if (S.size() <= 4) {
		if (error_res) {
			*error_res = 0;
		}
		// No need to optimize here
		return *this;
	}


	set< unsigned int, less<unsigned int> > selection, candidate;
	set< unsigned int, less<unsigned int> >::iterator it;

	const unsigned int nsample = 4;
	for (unsigned int i=0; i<nsample; i++) {
		selection.insert((i*S.size())/nsample);
	}
	selection.insert(S.size()-1);
	candidate = selection;
	printPath("target");
	printControl("tcontrol");

	while (1) {
		printf("Iteration: selected %d points out of %d\n",(int)selection.size(),(int)S.size());
		vector<Point> approx;
		vector<double> appvel;
#ifdef DEBUG_SPLINE
		printf("Selection: ");
#endif
		for (it=selection.begin();it!=selection.end();it++) {
#ifdef DEBUG_SPLINE
			printf("%d ",*it);
#endif
			approx.push_back(S[*it]);
			appvel.push_back(velocity_of_s(i_to_s[*it]));
		}
#ifdef DEBUG_SPLINE
		printf("\n");
#endif
		SplinePath current(approx,appvel,resolution);
#ifdef DEBUG_SPLINE
		printf("current %f %f cS[0] %f %f S[0] %f %f this %f %f\n",current(0).x,current(0).y,
				current.S[0].x,current.S[0].y,S[0].x,S[0].y,value(0).x,value(0).y);
#endif
		current.printPath("current");
		current.printControl("ccontrol");

		unsigned int iprev = 0,selindex;
		double max_error = 0;
		for (it=selection.begin(),it++,selindex=1;it!=selection.end();it++,selindex++) {
			unsigned int i,i1,i0 = iprev, intlength = *it  - iprev;
			i1 = iprev = *it;

			if (intlength < 2) {
				continue;
			}
			unsigned int imax = i0;
			double max_local = 0;
			double l0 = current.i_to_s[selindex-1];
			double l1 = current.i_to_s[selindex];
			double segment_length = l1 - l0;
			if (segment_length < 1e-10) {
				continue;
			}
			for (i=i0+1;i<i1;i++) {
				double e_min = (current(l0) - S[i]).norm();
				Point cmp = current(l0);
				// This is a stupid way to find the closest point on the
				// current spline from S[i]. Using the gradient would be much
				// smarter, but the GSL doen't have a gradient descent for a 1D
				// function, and the multidim version seems stupid in 1D.
				for (int k=0;k<=100;k++) {
					Point P = current(l0+k*segment_length/100);
					double e = (P-S[i]).norm();
					if (e < e_min) {
						cmp = P;
						e_min = e;
					}
				}
#ifdef DEBUG_SPLINE
				printf("i = %d: error %f\n",i,e_min);
				FILE * fp = fopen("error","w");
				fprintf(fp,"%f %f\n%f %f\n",S[i].x,S[i].y,cmp.x,cmp.y);
				fclose(fp);
				fflush(NULL);
#ifdef USE_GNUPLOT
				gpl.plot("plot \"target\" u 3:4 w l, \"current\" u 3:4 w l, \"ccontrol\" w p, \"error\" w lp");
#endif
				//getchar();
#endif
				if (e_min > max_local) {
					imax = i;
					max_local = e_min;
				}
			}
#ifdef DEBUG_SPLINE
			printf("Max local %f at %d\n",max_local,imax);
#endif
			if (max_local > error_thres) {
#ifdef DEBUG_SPLINE
				printf("Inserted imax\n");
#endif
				candidate.insert(imax);
			}
			if (max_local > max_error) {
				max_error = max_local;
			}


		}
		if (error_res) {
			*error_res = max_error;
		}
		printf("Max error: %f / %f\n",max_error,error_thres);
		fflush(NULL);
#ifdef USE_GNUPLOT
		gpl.plot("plot \"target\" u 3:4 w l, \"current\" u 3:4 w l, \"ccontrol\" w p");
#endif
		//getchar();
		if (max_error < error_thres) {
			return current;
		}
		if (candidate == selection) {
			return current;
		}
		selection = candidate;
	}

}


#define sign(x) (((x)<0)?-1:+1)

bool splines::buildTraversalTimeMaps(double L, double V0, double V, double max_brake, double max_accel, bool stopAtEnd,
		Function & v_of_t) 
{
	v_of_t.clear();
	v_of_t.set(0,V0);
	max_brake = fabs(max_brake);
	max_accel = fabs(max_accel);
	double init_accel;
	double t0 = 0;
	double timetozero = fabs(V0)/max_brake;
	double disttozero = V0*V0 / (2*max_brake);
	double disttogo = L;

	if ((stopAtEnd || (V*V0<0)) && (disttogo < disttozero)) {
		/** We don't even have the time to brake to zero 
		 * on the path length, so let's reach the minimum possible speed **/
		double D = V0*V0 - 2*max_brake*disttogo; // |V0| > D > 0, by def
		double dt = (fabs(V0) - sqrt(D))/(max_brake); // This is the first solution
		double vmin = sign(V0) * (fabs(V0) - dt*max_brake);
		v_of_t.set(dt,vmin); // Function represents exactly piecewise linear functions
		return false;
	}

	double timetoV,disttoV;
	if (V0 * V < 0) {
		/** V0 and V are of opposite sign, so we will have to brake to zero,
		 * then accelerate to V!
		 * At least, we know from previous computation, that we have the 
		 * time to reach zero
		 * **/
		v_of_t.set(timetozero,0);
		t0 = timetozero;
		V0 = 0;
		disttogo -= disttozero;
	} 
	
	/** From here, we can assume that V0 and V are of the same sign, or V0 = 0
	 * */
	if (fabs(V) > fabs(V0)) {
		init_accel = max_accel;
	} else /* |V| < |V0| */ {
		init_accel = -max_brake;
	}
	timetoV = fabs(V-V0)/fabs(init_accel);
	disttoV = fabs(V0)*timetoV + init_accel*timetoV*timetoV/2;

	double timetobrake = 0;
	double disttobrake = 0;
	if (stopAtEnd) {
		timetobrake = fabs(V)/max_brake;
		disttobrake = V*V/(2*max_brake);
	}

	/* We know that we can brake from initial speed
	 * The question is, can we reach the desired speed? */
	if (disttoV + disttobrake > disttogo) {
		/** the answer is NO. 
		 * Let see what speed we can reach **/
		if (fabs(V0) <= fabs(V)) {
			// First the peak velocity abscissa
			double s = (max_brake*disttogo - V0*V0)/(max_brake+max_accel);
			V = sign(V) * sqrt(V0*V0+2*max_accel*s);
			timetoV = fabs(V-V0)/max_accel;
			disttoV = s;
			if (stopAtEnd) {
				timetobrake = fabs(V)/max_brake;
				disttobrake = V*V/(2*max_brake);
			}
		} else {
			/* |V0| > |V|: we can only reach this situation if we don't stop at
			 * the end. Let's assume that |V0|>|V| and we stop at the end. Then
			 * disttozero < disttogo. But we also have
			 * disttoV+disttobrake<disttozero so disttoV+distobrake<disttogo.
			 * So let's find the minimum velocity we can reach, and plan a
			 * maximum breaking path */
			double D = V0*V0 - 2*max_brake*disttogo; // |V0| > D > 0, by def
			double dt = (fabs(V0) - sqrt(D))/(max_brake); // This is the first solution
			double vmin = sign(V0) * (fabs(V0) - dt*max_brake);
#if 0
			printf("Don't have time to reach required speed.\n");
			printf("V0 %f B %f Dtg %f D %f\n",V0,max_brake,disttogo,D);
			printf("End of path reached in %fs, v=%fm/s\n",dt,vmin);
#endif
			v_of_t.set(dt,vmin); // we don't need t0 in this case
			return false;
		}
	}
	/* From this point we know that we have the time to reach V from V0,
	 * eventually stay at cruising speed V, then eventually descelerate to
	 * the final speed
	 * Let's find when V is reached, and plan a continuous acceleration to V */
	v_of_t.set(t0+timetoV,V);
	/* Now check how long we stay at V */
	double distatV = disttogo - (disttoV + disttobrake);
	double timeatV = distatV / fabs(V);
	if (distatV > 0) {
		v_of_t.set(t0+timetoV+timeatV,V);
	} else {
		timeatV = 0;
		distatV = 0;
	}

	/* Finally brake if required */
	if (stopAtEnd) {
		v_of_t.set(t0+timetoV+timeatV+timetobrake,0);
	}
	return true;
}

