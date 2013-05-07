#ifndef FUNCTION_H
#define FUNCTION_H

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <set>
#include <map>
#include <vector>
#include <functional>

namespace splines {

    template <class C>
    class TFunction {
		protected:
			typedef std::map< double,C,std::less<double> > FDB;
			FDB vals;
		public:
			typedef std::set< double, std::less<double> > Support;
			typedef typename FDB::iterator iterator;
			typedef typename FDB::const_iterator const_iterator;

			TFunction() {}
			TFunction(const TFunction<C> & f):vals(f.vals) {}
			~TFunction() {}

            // Assumes C implements operators +,-, *double, /double
            // i.e. C is algebraically embedded in a vector field
			virtual C operator()(double x) const {
				typename FDB::const_iterator it2 = vals.lower_bound(x);
				if (it2==vals.begin()) {/*printf("R1\n");*/return it2->second;}
				typename FDB::const_iterator it1 = it2;
				it1--;
				if (it2==vals.end()) {/*printf("R2\n");*/return it1->second;}

				C dy = (it2->second-it1->second);
				return it1->second + (x-it1->first) * dy/(it2->first-it1->first);
			}

			void clear() {vals.clear();}
			iterator begin() {return vals.begin();}
			iterator end() {return vals.end();}
			bool empty() const {return vals.empty();}

			const_iterator begin() const {return vals.begin();}
			const_iterator end() const {return vals.end();}

			void set(double x, const C & value) {
                vals[x] = value;
            }

			void tabulate(double xmin,double xmax,double dx,std::vector<C> & values) const {
                double x;
                for (x=xmin;x<=xmax;x+=dx) {
                    values.push_back(operator()(x));
                }
            }

            void extract_support(Support & s) const {
                typename FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++) {
                    s.insert(it->first);
                }
            }

            Support support() const {
                Support s;
                extract_support(s);
                return s;
            }

			void print(FILE * fp,void (*print_function)(FILE*,const C&)) const {
                typename FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++) {
                    fprintf(fp,"%f\t",it->first);
                    print_function(fp,it->second);
                    fprintf(fp,"\n");
                }
            }

			void print(const char * fname,void (*print_function)(FILE*,const C&)) const {
                FILE * fp = fopen(fname,"w");
                if (fp == NULL) {
                    perror("Function::print");
                    return;
                }
                this->print(fp,print_function);
                fclose(fp);
            }

			double xmin() const {
				typename FDB::const_iterator it = vals.begin();
				if (it == vals.end()) return 0.0;
				return it->first;
			}
			double xmax() const {
				typename FDB::const_iterator it = vals.end();
				if (it == vals.begin()) return 0.0;
				return (--it)->first;
			}

			double mindx() const
            {
                double dx,mdx = -1;
                typename FDB::const_iterator it = vals.begin();
                while (1) {
                    double x0, x1;
                    x0 = it->first;
                    it++;
                    if (it == vals.end()) return mdx;
                    x1 = it->first;
                    dx = x1 - x0;
                    if ((mdx<0) || (dx < mdx)) mdx = dx;
                }
                // never reached
                assert(mdx >= 0);
                return -1;
            }

			TFunction<C> operator+(double s) const
            {
                TFunction<C> res;
                typename FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++)
                    res.set(it->first,it->second + s);
                return res;
            }

            TFunction<C> operator-() const
            {
                TFunction<C> res;
                typename FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++)
                    res.set(it->first,-it->second);
                return res;
            }

            TFunction<C> operator-(double s) const
            {
                TFunction<C> res;
                typename FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++)
                    res.set(it->first,it->second - s);
                return res;
            }

            TFunction<C> operator*(double s) const
            {
                TFunction<C> res;
                typename FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++)
                    res.set(it->first,it->second * s);
                return res;
            }
            TFunction<C> operator/(double s) const
            {
                TFunction<C> res;
                typename FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++)
                    res.set(it->first,it->second / s);
                return res;
            }

            TFunction<C> operator+(const TFunction<C> & f) const
            {
                TFunction<C> res;
                Support s;
                extract_support(s);
                f.extract_support(s);
                typename Support::const_iterator it = s.begin();
                for (;it != s.end();it++) {
                    res.set(*it,(*this)(*it) + f(*it));
                }
                return res;
            }
            TFunction<C> operator-(const TFunction<C> & f) const
            {
                TFunction<C> res;
                Support s;
                extract_support(s);
                f.extract_support(s);
                typename Support::const_iterator it = s.begin();
                for (;it != s.end();it++) {
                    res.set(*it,(*this)(*it) - f(*it));
                }
                return res;
            }

            // Assumes C implements a C*C operator
            TFunction<C> operator*(const TFunction<C> & f) const
            {
                TFunction<C> res;
                Support s;
                extract_support(s);
                f.extract_support(s);
                typename Support::const_iterator it = s.begin();
                for (;it != s.end();it++) {
                    res.set(*it,(*this)(*it) * f(*it));
                }
                return res;
            }
            // Assumes C implements a C/C operator
            TFunction<C> operator/(const TFunction<C> & f) const
            {
                TFunction<C> res;
                Support s;
                extract_support(s);
                f.extract_support(s);
                typename Support::const_iterator it = s.begin();
                for (;it != s.end();it++) {
                    res.set(*it,(*this)(*it) / f(*it));
                }
                return res;
            }

            template <class T>
			TFunction<C> map(C (T::*f)(const C &),T*that) const {
                TFunction<C> res;
                typename FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++)
                    res.set(it->first,that->f(it->second));
                return res;
            }

            template <class T>
			TFunction<C> map(C (T::*f)(C),T*that) const {
                TFunction<C> res;
                typename FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++)
                    res.set(it->first,that->f(it->second));
                return res;
            }

			TFunction<C> map(C (*f)(const C &)) const {
                TFunction<C> res;
                typename FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++)
                    res.set(it->first,f(it->second));
                return res;
            }

			TFunction<C> map(C (*f)(C)) const {
                TFunction<C> res;
                typename FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++)
                    res.set(it->first,f(it->second));
                return res;
            }

			// Select the point of F between x0 and x1
			TFunction<C> select(double x0, double x1)const 
            {
                TFunction<C> res;
                if (empty()) return res;
                typename FDB::const_iterator it0 = vals.lower_bound(x0);
                typename FDB::const_iterator it1 = vals.upper_bound(x1);
                res.set(x0,this->operator()(x0));
                while (it0 != it1) {
                    res.set(it0->first,it0->second);
                    it0 ++;
                }
                res.set(x1,this->operator()(x1));
                return res;
            }

			// Resample f over [x0,x1] with sampling step h
			// When negative, the smallest necessary h is computed
			TFunction<C> sample(double x0, double x1, double h=-1)const
            {
                TFunction<C> res;
                double x;
                if (h <= 0) { h = mindx(); }
                for (x=x0;x<x1;x+=h) {
                    res.set(x,this->operator()(x));
                }
                res.set(x1,this->operator()(x1));
                return res;

            }

            TFunction<C> reverse() const {
                TFunction<C> res;
                double x_max = xmax();
                typename FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++) {
                    res.set(x_max - it->first,it->second);
                }
                return res;
            }

    };

	class Function : public TFunction<double>
	{
		public:
			Function() {}
            Function(const TFunction<double> & f) : TFunction<double>(f) {}

			void print(FILE * fp = stdout) const ;
            void print(const char * fname) const ;

			std::pair<double,double> fmin() const;
			std::pair<double,double> fmax() const;

			static Function map(double (*f)(double,double),
                    const Function & f1,const Function & f2) {
                Function res;
                Support s;
                f1.extract_support(s);
                f2.extract_support(s);
                typename Support::const_iterator it = s.begin();
                for (;it != s.end();it++) {
                    res.set(*it,f(f1(*it),f2(*it)));
                }
                return res;
            }
            template<class C>
			Function map(double (C::*f)(double),C*that) const {
                Function res;
                FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++)
                    res.set(it->first,that->f(it->second));
                return res;
            }
			Function map(double (*f)(double)) const {
                Function res;
                FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++)
                    res.set(it->first,f(it->second));
                return res;
            }
			Function map(const Function & f) const;

            // Compute the inverse, only works if the function is monotonic
			Function inverse() const {
                Function res;
                FDB::const_iterator it = vals.begin();
                for (;it != vals.end();it++)
                    res.set(it->second,it->first);
                return res;
            }

			// compute F(x) = int(f(u),u=x0..x) for x varying in
			// [xlow,xup] with a sampling of dx
            // This function cannot be const because we use gsl for the
            // integration and gsl uses void*, not const void*
			Function primitive(double x0, double dx) ;

			// compute df/dx with step size of h
			// When negative, the smallest necessary h is computed
            // This function cannot be const because we use gsl for the
            // differentiation and gsl uses void*, not const void*
			Function derivative(double h=-1) ;

			// compute df/dx with step size of h at point x
			// When negative, the smallest necessary h is computed
            // This function cannot be const because we use gsl for the
            // integration and gsl uses void*, not const void*
			double derivativeAt(double x, double h=-1) ;

            // Returns the intersection of the curve with y=0. This assumes
            // that the function is not identically zero. If it is, then all
            // the sample points where the function is zero are returned
            Support zeros() const;

            // Returns the minima of the function, i.e. the point 
            // where the derivative is zero, and the second derivative positive
            // This function cannot be const because we use gsl for the
            // differentiation and gsl uses void*, not const void*
            Support minima(double dead_zone) ;

            // Returns the maxima of the function, i.e. the point 
            // where the derivative is zero, and the second derivative positive
            // This function cannot be const because we use gsl for the
            // differentiation and gsl uses void*, not const void*
            Support maxima(double dead_zone) ;
	};

	class AngularFunction : public Function
	{
		public:
			AngularFunction(){}
            AngularFunction(const TFunction<double> & f) : Function(f) {}

			virtual double operator()(double x) const {
				FDB::const_iterator it2 = vals.lower_bound(x);
				if (it2==vals.begin()) {/*printf("R1\n");*/return it2->second;}
				FDB::const_iterator it1 = it2;
				it1--;
				if (it2==vals.end()) {/*printf("R2\n");*/return it1->second;}
				//printf("x %f : u %f->%f l %f->%f\n",x,it1->first,it1->second,it2->first,it2->second);
				double dy = remainder(it2->second-it1->second,2*M_PI);
				double y = it1->second + (x-it1->first) *
					dy/(it2->first-it1->first);
				return remainder(y,2*M_PI);
			}
	};

}

#endif // FUNCTION_H
