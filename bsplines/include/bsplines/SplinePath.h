#ifndef SPLINE_PATH_H
#define SPLINE_PATH_H


#include "BSpline.h"
#include "Function.h"

namespace splines {
	/**
	 * \class SplinePath: re-parameterize a spline so as to make its value the 
	 * pose of a object moving at unit speed on the curve 
	 * **/
	class SplinePath : public BSpline
	{
		protected:
			Function t_to_s;
			Function s_to_u;
			Function s_to_u_dot;
			Function s_to_u_dotdot;
			Function velocity_of_s;
			std::vector<double> i_to_s;
			double L,T;
			double resolution;

			bool buildSMap(double res);
			bool buildTMap();
		public:

			/** Constructor from an array of BSpline::Point, \arg res
			 * is the sampling resolution of the initial [0,1] interval
			 * */
			SplinePath(const Point * s, unsigned int n, double res=1e-3, double velocity=1.0);
			/** idem, with a vector of point */
			SplinePath(const std::vector<Point> & s, double res=1e-3, double velocity=1.0);

			/** Constructor from an array of BSpline::Point, \arg res
			 * is the sampling resolution of the initial [0,1] interval
			 * */
			SplinePath(const Point * s, const double * vel, unsigned int n, double res=1e-3);
			/** idem, with a vector of point */
			SplinePath(const std::vector<Point> & s, const std::vector<double> & vel, double res=1e-3);
			
			
			~SplinePath() {}

			/** returns the value of the curve at distance s 
			 * from the initial point **/
			Point operator()(double s) const {
				return BSpline::value(s_to_u(s));
			}
			// Alias
			Point value(double s) const {
				return BSpline::value(s_to_u(s));
			}
			double velocity(double s) const{
				// This must be equal to 1.0 or close
				// return BSpline::velocity(s_to_u(s)) * s_to_u_dot(s);
				return velocity_of_s(s);
			}

			Point speed(double s) const {
				return BSpline::speed(s_to_u(s)) * s_to_u_dot(s) * velocity_of_s(s);
			}

			Point acceleration(double s) const {
				return (BSpline::acceleration(s_to_u(s)) * s_to_u_dot(s)) +
					(BSpline::speed(s_to_u(s)) * s_to_u_dotdot(s));
			}

			double orientation(double s) const{
				return BSpline::orientation(s_to_u(s));
			}
			double curvature(double s) const;

			/** returns the length of the spline **/
			double length() const {
				return L;
			}

			/** returns the duration of the spline **/
			double duration() const {
				return T;
			}

			void printPath(const std::string & filename, double sampling=0.1) const;

			/** Return the position (along the path) corresponding
			 * to control point \arg i. The number of control point can 
			 * be optained using the controlSize() function
			 * **/
			double getControlPointAbscissa(unsigned int i) const {
				if (i >= S.size()) {
					i = S.size()-1;
				}
				return i_to_s[i];
			}

			double getAbscissaAtTime(double t) const {
				return t_to_s(t);
			}

			/** 
			 * Create a new spline path that approximate the current one, with a
			 * reduced number of point. The algorithm is quite ad-hoc and should 
			 * not be trusted to be optimal, but it does the job for a smooth
			 * original path. The density of control point augment as a function of
			 * the square root of the curvature (just because this function seem to
			 * have the right shape).
			 * \arg min_inter_point is the minimum distance between two control
			 * points. If this is too big, the spline will not be able to follow
			 * high curvature paths.
			 * \arg max_inter_point is the maximum distance between two control
			 * point. 
			 * \arg sampling is the resolution of the curvature sampling
			 * **/
			SplinePath resampleByCurvature(double min_inter_point, double max_inter_point, double sampling) const;

			/** 
			 * Create a new spline path that approximate the current one, with a
			 * reduced number of point. 
			 * **/
			SplinePath decimate(double error_threshold, double *error_res=NULL) const;



			/*** Template constructor for any container of BSpline::Point. 
			 * The point are accessed using an iterator, so (*iterator) must
			 * results in a BSpline::Point **/
			template <class iterator>
				SplinePath(const iterator & begin, const iterator & end, double res=1e-3, double vel=1.0) :
					BSpline(begin,end)
			{
				i_to_s.resize(S.size());
				buildSMap(res);
				velocity_of_s.set(-1e-3,0);
				velocity_of_s.set(0,vel);
				velocity_of_s.set(L,vel);
				velocity_of_s.set(L+1e-3,0);
				buildTMap();
			}
	};

	/***
	 * Build a time map to follow a path of length L at speed V, starting at speed V0, eventually stopping 
	 * at the end of the path. Maximum acceleration and braking are use to
	 * compute the slope of the speed profile. 
	 * The output is a functions:  v(t) which gives the velocity at time t. 
	 * The function return false if no exact solution can be found. In this
	 * case v(t) is set to the closest solution. This usually means
	 * that the path is to short to brake or accelerate.
	 * If the maximum speed cannot be reached, a new maximum speed is computed.
	 * The corresponding curvilinear abscissa can be obtained with
	 * s_of_t = v_of_t.primitive(0,sampling);
	 * or
	 * s_of_t = v_of_t.primitive(0,sampling).map(fabs);
	 * if s is required to be positive
	 * or even
	 * s_of_t = v_of_t.map(fabs).primitive(0,sampling);
	 * if only the length of the motion is desired
	 *
	 * If V0 and V are not of the same sign, the semantic is tricky. It is
	 * currently assumed that L is the total length of the path, including the
	 * braking down to zero and the acceleration. This is subject to
	 * modification if somebody has a better idea
	 * */
	bool buildTraversalTimeMaps(double L,double V0, double V, double max_brake, double max_accel, bool stopAtEnd,
			Function & v_of_t) ;
}


#endif // SPLINE_PATH_H

