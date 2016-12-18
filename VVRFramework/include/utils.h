#ifndef UTILS_H
#define UTILS_H

#include "vvrscenedll.h"

#include <string>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

#define echo(x) cout<<#x<<" = "<<x<<endl
#define msg(x) cout<<x<<endl
#define SQUARE(x) ((x)*(x))

#define VAR_CLASS_DEFS(x)        \
typedef std::shared_ptr< x > Ptr;      \
typedef std::shared_ptr<const x > ConstPtr;   \
static inline std::shared_ptr<x> Make()    \
{              \
 return std::make_shared<x>();      \
}              \
template<typename T0>         \
static inline std::shared_ptr<x> Make(const T0& t0)   \
{              \
 return std::make_shared<x>(t0);     \
}              \
template<typename T0, typename T1>      \
static inline std::shared_ptr<x> Make(const T0& t0, const T1& t1) \
{              \
 return std::make_shared<x>(t0, t1);    \
}              \
template<typename T0, typename T1, typename T2>   \
static inline std::shared_ptr<x> Make(const T0& t0, const T1& t1, const T2& t2) \
{              \
return std::make_shared<x>(t0, t1, t2);    \
}              \
template<typename T0, typename T1, typename T2, typename T3> \
static inline std::shared_ptr<x> Make(const T0& t0, const T1& t1, const T2& t2, const T3& t3)   \
{              \
return std::make_shared<x>(t0, t1, t2, t3);   \
}              \
template<typename T0, typename T1, typename T2, typename T3, typename T4> \
static inline std::shared_ptr<x> Make(const T0& t0, const T1& t1, const T2& t2, const T3& t3, const T4& t4)   \
{              \
return std::make_shared<x>(t0, t1, t2, t3, t4);  \
}              \
template<typename T0, typename T1, typename T2, typename T3, typename T4, typename T5> \
static inline std::shared_ptr<x> Make(const T0& t0, const T1& t1, const T2& t2, const T3& t3, const T4& t4, const T5& t5)   \
{              \
return std::make_shared<x>(t0, t1, t2, t3, t4, t5); \
}              \
template<typename T0, typename T1, typename T2, typename T3, typename T4, typename T5, typename T6> \
static inline std::shared_ptr<x> Make(const T0& t0, const T1& t1, const T2& t2, const T3& t3, const T4& t4, const T5& t5, const T6& t6)   \
{              \
return std::make_shared<x>(t0, t1, t2, t3, t4, t5, t6);\
}              \

namespace vvr {

float VVRScene_API getSeconds();
double VVRScene_API  normalizeAngle(double angle);
string VVRScene_API getExePath();
string VVRScene_API getBasePath();
vector<string> &split(const string &s, char delim, vector<string> &elems);
vector<string> split(const string &s, char delim);

}

#endif
