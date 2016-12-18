#ifndef SETTINGS_H
#define SETTINGS_H

#include "vvrscenedll.h"

#include <string>
#include <map>
#include <vector>

using namespace std;

namespace vvr {

class VVRScene_API Settings
{
    map<string, string> sMap;

public:
    Settings () {}
    Settings (string file);

    void getKeys(vector<string> &keys) const;

    string  getStr(const string &Key) const;
    double  getDbl(const string &key) const;
    int     getInt(const string &key) const;
    bool    getBool(const string &key) const;
};

}

#endif // SETTINGS_H
