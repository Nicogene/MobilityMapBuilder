/*
 * Copyright (C) 2018
 * Authors: Nicolò Genesio
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <iostream>
#include <dot.h>

double dot(std::vector<float>& u, std::vector<float>& v) {
	double accum = 0.;
	for (int i = 0; i < u.size(); ++i) {
		accum += u[i] * v[i];
	}
	return accum;
}

