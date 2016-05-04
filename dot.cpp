//
// Created by nick on 23/06/15.
//

#include <iostream>
#include "dot.h"

double dot(std::vector<float>& u, std::vector<float>& v) {
	double accum = 0.;
	for (int i = 0; i < u.size(); ++i) {
		accum += u[i] * v[i];
	}
	return accum;
}

