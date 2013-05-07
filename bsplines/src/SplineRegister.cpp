#include <string>
#include <string.h>
#include <ros/ros.h>

#include "bsplines/SplineRegister.h"

using namespace splines;
using namespace std;

SplineRegister::SplineRegister()
{
	max_inter_points = 3.0;
	min_inter_points = 0.5;
}

void SplineRegister::clear()
{
	PathMap::iterator i;
	for (i=paths.begin();i!=paths.end();i++) {
		delete *i;
	}
	paths.clear();
}

SplineRegister::~SplineRegister()
{
	clear();
}

static string trim(const string & s)
{
	size_t p1,p2;
	p1 = s.find_first_not_of("\r\t\n ");
	p2 = s.find_last_not_of("\r\t\n ");
	if ((p1==string::npos) || (p2==string::npos)) {
		return string();
	}
	return s.substr(p1,p2-p1+1);
}

bool SplineRegister::load(const std::string & filename)
{
	FILE * fp = fopen(filename.c_str(),"r");
	if (fp == NULL) {
        ROS_ERROR("Cannot open '%s' for reading",filename.c_str());
		return false;
	}
	string pathname;
	double velocity = 0;
	vector<Point> vp;
	vector<double> vv;
	unsigned int linenum=0;
	clear();
	while (!feof(fp)) {
		char tmp[1024]="";
		if (fgets(tmp,1023,fp) <= 0) {
			break;
		}
		fflush(NULL);
		linenum += 1;
		string s = trim(string(tmp));
		if (strncasecmp(s.c_str(),"spline",6)==0) {
			if (!vp.empty() && !pathname.empty()) {
				NameMap::iterator i = std::find(names.begin(),names.end(),pathname);
				if (i != names.end()) {
					ROS_ERROR("Warning: duplicated path name '%s'",pathname.c_str());
				}
				paths.push_back(new SplinePath(vp,vv,1e-3));
				names.push_back(pathname);
			}
			pathname = trim(s.substr(7,s.length()-7));
			vp.clear();
			velocity = 0;
			continue;
		}
		if (strncasecmp(s.c_str(),"waypoint",8)==0) {
			double x,y,v=0;
			int ret = sscanf(s.c_str()," waypoint %le %le %le ",&x,&y,&v);
			if (ret>=2) {
				if (ret >= 3) {
					velocity = v;
				}
				vp.push_back(Point(x,y));
				vv.push_back(velocity);
			}
			continue;
		}
		// Comment or empty line
		if ((s[0]=='#') || s.empty()) {
			continue;
		}
		ROS_ERROR("%s: invalid line %d: '%s'",filename.c_str(),linenum,s.c_str());
	}
	fclose(fp);
	if (!vp.empty() && !pathname.empty()) {
		NameMap::iterator i = std::find(names.begin(),names.end(),pathname);
		if (i != names.end()) {
			ROS_ERROR("Warning: duplicated path name '%s'",pathname.c_str());
		}
		paths.push_back(new SplinePath(vp,vv,1e-3));
		names.push_back(pathname);
	}

	return true;
}


bool SplineRegister::save(const std::string & filename, bool append) const
{
	FILE * fp = fopen(filename.c_str(),append?"a":"w");
	if (fp == NULL) {
		ROS_ERROR("Cannot open '%s' for writing",filename.c_str());
		return false;
	}
	unsigned int i,j;
	for (i=0;i<paths.size();i++) {
		fprintf(fp,"spline %s\n",names[i].c_str());
		const SplinePath * sp = paths[i];
		for (j=0;j<sp->getControlSize();j++) {
			const Point & ctrl = sp->getControl(j);
			double pos = sp->getControlPointAbscissa(j);
			fprintf(fp,"waypoint %f\t%f\t%f\n",ctrl.x,ctrl.y,sp->velocity(pos));
		}
		fprintf(fp,"\n");
	}
	fclose(fp);

	return true;
}


void SplineRegister::printList(FILE *out, bool all) const
{
	unsigned int i,j;
	for (i=0;i<paths.size();i++) {
		fprintf(out,"Path %s\n",names[i].c_str());
		if (all) {
			const SplinePath * sp = paths[i];
			for (j=0;j<sp->getControlSize();j++) {
				const Point & ctrl = sp->getControl(j);
				double pos = sp->getControlPointAbscissa(j);
				fprintf(out,"P %6.2f\t%6.2f\tV %3.2f\n",ctrl.x,ctrl.y,sp->velocity(pos));
			}
			fprintf(out,"\n");
		}
	}
}

const SplinePath * SplineRegister::getPathByName(const std::string & s) const 
{
	NameMap::const_iterator it = std::find(names.begin(),names.end(),s);
	if (it == names.end()) {
		return NULL;
	}
	return paths[it - names.begin()];
}

const SplinePath * SplineRegister::getPathById(unsigned int id) const 
{
	if (id > paths.size()) {
		return NULL;
	}

	return paths[id];
}

bool SplineRegister::addPath(const std::string & name, const std::vector<Point> & points, double velocity)
{
	SplinePath orig(points,1e-3,velocity);
	SplinePath decim = orig.resampleByCurvature(min_inter_points,max_inter_points,0.1);

	NameMap::iterator i = std::find(names.begin(),names.end(),name);
	if (i != names.end()) {
		printf("Warning: duplicate path '%s'\n",name.c_str());
	}
	paths.push_back(new SplinePath(decim));
	names.push_back(name);
	return true;
}


bool SplineRegister::addPath(const std::string & name, const std::vector<Point> & points, const std::vector<double> & velocities)
{
	SplinePath orig(points,velocities,1e-3);
#if DEB
	printf("addPath: created initial spline\n");
#endif
	orig.printPath("orig.path",0.1);
#if DEB
	printf("addPath: plotted initial spline\n");
#endif
	//SplinePath decim = orig.decimate(0.1);
	SplinePath decim = orig.resampleByCurvature(min_inter_points,max_inter_points,0.1);
#if DEB
	printf("addPath: decimated initial spline\n");
#endif

	NameMap::iterator i = std::find(names.begin(),names.end(),name);
	if (i != names.end()) {
		printf("Warning: duplicate path '%s'\n",name.c_str());
	}
	paths.push_back(new SplinePath(decim));
	names.push_back(name);
	return true;
}





