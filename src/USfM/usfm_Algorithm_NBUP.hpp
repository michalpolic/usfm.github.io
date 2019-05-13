// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Algorithm_NBUP.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_ALGORITHM_NBUP_H
#define USFM_ALGORITHM_NBUP_H

#include "USfM/usfm_Algorithm.hpp"

namespace usfm {

	class Algorithm_NBUP: public Algorithm {
	public:
		void compute(Scene& scene, Statistic& statistic);
	private:
		void computeSchurComplementUsingNullspace(const Scene &scene, const SM &Ms, const DM &Hs, DM &Zs, SM &invAs, SM &Bs);
	};
}

#endif