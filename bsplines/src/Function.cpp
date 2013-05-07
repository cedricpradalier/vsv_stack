#include <assert.h>
#include "bsplines/Function.h"

using namespace std;
using namespace splines;

void Function::print(FILE * fp) const {
    FDB::const_iterator it = vals.begin();
    for (;it != vals.end();it++) {
        fprintf(fp,"%f\t%f\n",it->first,it->second);
    }
}

void Function::print(const char * fname) const {
    FILE * fp = fopen(fname,"w");
    if (fp == NULL) {
        perror("Function::print");
        return;
    }
    print(fp);
    fclose(fp);
}

pair<double,double> Function::fmin() const 
{
	FDB::const_iterator it = vals.begin();
	if (it == vals.end()) return pair<double,double>(0.0,0.0);
	double m = it->second;
	double x = it->first;
	for(;it!=vals.end();it++) 
		if (it->second < m) {
			m = it->second;
			x = it->first;
		}
	return pair<double,double>(x,m);
}

pair<double,double> Function::fmax() const 
{
	FDB::const_iterator it = vals.begin();
	if (it == vals.end()) return pair<double,double>(0.0,0.0);
	double m = it->second;
	double x = it->first;
	for(;it!=vals.end();it++) 
		if (it->second > m) {
			m = it->second;
			x = it->first;
		}
	return pair<double,double>(x,m);
}


Function Function::map(const Function & f) const
{
	Function res;
	FDB::const_iterator it = vals.begin();
	for (;it != vals.end();it++)
		res.set(it->first,f(it->second));
	return res;
}

Function::Support Function::minima(double dead_zone) /* const */
{
    Function d = derivative();
    Support s = d.zeros(), res; 
    Function dd = d.derivative();

    std::map<double,double> mins;
    // First, store all candidates in a map, ordered by value
    for (Support::const_iterator it = s.begin(); it != s.end(); it++) {
        double d2 = d.derivativeAt(*it);
        if (d2 > 0) {
            mins.insert(std::map<double,double>::value_type((*this)(*it),*it));
        }
    }
    // Then remove anything closer than deadzone
    for (std::map<double,double>::iterator it = mins.begin();
            it != mins.end(); it++) {
        res.insert(it->second);
        for (std::map<double,double>::iterator jt = mins.begin();
                jt != mins.end(); jt++) {
            if (jt == it) continue;
            if (fabs(jt->second-it->second) < dead_zone) {
                mins.erase(jt);
            }
        }
    }
    return res;
}

Function::Support Function::maxima(double dead_zone) /* const */
{
    Function d = derivative();
    Support s = d.zeros(), res; 
    Function dd = d.derivative();

    std::map<double,double> mins;
    // First, store all candidates in a map, ordered by value
    for (Support::const_iterator it = s.begin(); it != s.end(); it++) {
        double d2 = d.derivativeAt(*it);
        if (d2 < 0) {
            mins.insert(std::map<double,double>::value_type((*this)(*it),*it));
        }
    }
    // Then remove anything closer than deadzone
    for (std::map<double,double>::iterator it = mins.begin();
            it != mins.end(); it++) {
        res.insert(it->second);
        for (std::map<double,double>::iterator jt = mins.begin();
                jt != mins.end(); jt++) {
            if (jt == it) continue;
            if (fabs(jt->second-it->second) < dead_zone) {
                mins.erase(jt);
            }
        }
    }
    return res;
}


Function::Support Function::zeros() const
{
	Support res;
	FDB::const_iterator it = vals.begin(),itp = it;
    if (it->second==0.0) {
        res.insert(it->first);
    }
    it++;
	for (;it != vals.end();it++,itp++) {
        if (it->second==0.0) {
            res.insert(it->first);
        } else if ((itp->second < 0) && (it->second > 0)) {
            res.insert(itp->first - itp->second * (it->first - itp->first) / (it->second - itp->second));
        } else if ((itp->second > 0) && (it->second < 0)) {
            res.insert(itp->first + itp->second * (it->first - itp->first) / (it->second - itp->second));
        }
    }
	return res;
}


#include <gsl/gsl_errno.h>
#include <gsl/gsl_integration.h>
#include <gsl/gsl_deriv.h>
static double feval(double x, void * params) {
	const Function & f = *(Function*)params;
	return f(x);
}

Function Function::primitive(double x0, double dx) 
{
	Function res;
#if 0
	gsl_integration_workspace * ws = gsl_integration_workspace_alloc (1000);
	double x;
	gsl_function F;
	F.function = feval;
	F.params = this;
	double xlow = xmin();
	double xup  = xmax();
	for (x=xlow;x<=xup;x+=dx) {
		double intval=0,errval=0;
        size_t neval = 0;
		int st;
        st = gsl_integration_qng(&F,x0,x,1e-3,1e-2,&intval,&errval,&neval);
        if (st == GSL_SUCCESS) {
            res.set(x,intval);
        } 
	}
	gsl_integration_workspace_free(ws);
#else
    double sum = 0;
	FDB::const_iterator it = vals.begin(), itp = it;
    res.set(it->first,sum);
    it++;
	for (;it != vals.end();it++,itp++) {
        sum += 0.5*(it->second+itp->second) * (it->first-itp->first);
        res.set(it->first,sum);
    }
    res = res - (*this)(x0);
#endif
	return res;
}

Function Function::derivative(double h) 
{
	Function res;
	gsl_function F;
	F.function = feval;
	F.params = this;
	if (h <= 0) {
		h = mindx();
		//printf("min dx: %f\n", h);
	}
	FDB::const_iterator it = vals.begin();
	for (;it != vals.end();it++) {
		double dval=0, derr=0;
		int st = gsl_deriv_central (&F, it->first, h, &dval,&derr);
		if (st == GSL_SUCCESS) {
            res.set(it->first,dval);
        }
	}
	return res;
}

double Function::derivativeAt(double x, double h) 
{
    Function res;
    gsl_function F;
    F.function = feval;
    F.params = this;
    if (h <= 0) {
        FDB::const_iterator it2 = vals.lower_bound(x);
        FDB::const_iterator it1 = it2;
        if (it2==vals.begin()) {it2++;}
        it1--;
        // return (it2->second - it1->second)/(it2->first - it1->second);
        h = (it2->first - it1->first)*0.1;
        //printf("min dx: %f\n", h);
    }
    double dval=0, derr=0;
    int st = gsl_deriv_central (&F, x, h, &dval,&derr);
    if (st == GSL_SUCCESS) {
        return dval;
    } else {
        return 0;
    }
}


