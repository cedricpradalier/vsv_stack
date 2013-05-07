#ifndef B_SPLINES_H
#define B_SPLINES_H

#include <assert.h>
#include <math.h>
#include <vector>
#include <string>

namespace splines {

	struct Point {
		double x,y,z;
		Point() : x(0),y(0),z(0) {}
		Point(double a, double b, double c=0.0) : x(a), y(b), z(c) {
			assert(!isnan(x));
			assert(!isnan(y));
			assert(!isnan(z));
		}
		Point(const Point & p) : x(p.x), y(p.y), z(p.z) {
			assert(!isnan(x));
			assert(!isnan(y));
			assert(!isnan(z));
		}
        // Dot product
		double operator*(const Point & p) const {
			return p.x*x+p.y*y+p.z*z;
		}
        // cross product
		Point cross(const Point & P) const {
			return Point(y*P.z-z*P.y, P.x*z-P.z*x, x*P.y-y*P.x);
		}
		Point operator*(double k) const {
			return Point(k*x,k*y,z*z);
		}
		Point operator/(double k) const {
			return Point(x/k,y/k,z/k);
		}
		Point operator+(const Point & Q) const {
			return Point(x+Q.x,y+Q.y,z+Q.z);
		}
		Point operator-(const Point & Q) const {
			return Point(x-Q.x,y-Q.y,z-Q.z);
		}
		double norm() const {
			return sqrt(x*x+y*y+z*z);
		}
	};

	struct Bezier {
		Point P[4];
		Bezier() {}
		Bezier(Point P1,Point P2,Point P3,Point P4) {
			P[0]=P1; P[1]=P2;
			P[2]=P3; P[3]=P4;
		}
		Bezier(Point Q[4]) {
			P[0]=Q[0]; P[1]=Q[1];
			P[2]=Q[2]; P[3]=Q[3];
		}
		Bezier(const Bezier & c) {
			P[0]=c.P[0]; P[1]=c.P[1];
			P[2]=c.P[2]; P[3]=c.P[3];
		}

		// evaluate bezier curve at t in [0,1]
		Point operator()(double t) const;
		// Alias 
		Point value(double t) const {
			return operator()(t);
		}
		Point speed(double t) const;
		Point acceleration(double t) const;
	};

	/**
	 * \class BSpline: implement an relaxed interpolating BSpline
	 * http://www.math.ucla.edu/~baker/149.1.02w/handouts/dd_splines.pdf
	 * \author: C. Pradalier
	 * */
	class BSpline
	{
		protected:

			std::vector<Bezier> bezier;
			// interpolated
			std::vector<Point> S;
			// polygons
			std::vector<Point> B;

			bool computeB();
			bool computeBezier();

		public:
			/** Constructor based on an array of BSpline::Point */
			BSpline(const Point * s, unsigned int n);
			/** Constructor based on a vector of BSpline::Point */
			BSpline(const std::vector<Point> & s);

			/** Constructor based on a container of BSpline::Point,
			 * *iterator must resolve as a BSpline::Point. map or multimap 
			 * cannot be used */
			template <class iterator>
				BSpline(const iterator & begin, const iterator & end) {
					for (iterator it = begin;it!=end;it++) {
						S.push_back(*it);
					}
					B.resize(S.size());
					bezier.resize(S.size()-1);

					computeB();
					computeBezier();
				}

			/** Destructor: does nothing */
			~BSpline() { }

			unsigned int getControlSize() const {
				return S.size();
			}

			const Point & getControl(unsigned int i) const {
				return S[i];
			}

			/** The interpolating Bspline is built from a set of point. The
			 * interpolating curves is between in [0,1]. Returns the value of the
			 * curve at u. Usage: 
			 *   BSpline s(my_points);
			 *   BSpline::Point P = s(0.4);
			 * **/
			Point operator()(double u) const;
			//Alias
			Point value(double u) const {
				return operator()(u);
			}
			Point speed(double u) const;
			Point acceleration(double u) const;

			double velocity(double u) const;
			double orientation(double u) const;
			double curvature(double u) const;

			/* debug tools: */
			/** output the input point in file "fname" **/
			void printControl(const std::string & fname) const;
			/** output the control point in file "fname" **/
			void printPolygon(const std::string & fname) const;
			/** output the curves hull in file "fname" **/
			void printHull(const std::string & fname) const;

	};

}

#endif // B_SPLINES_H
