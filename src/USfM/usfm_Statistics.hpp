// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Statistics.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_STATISTICS_H
#define USFM_STATISTICS_H

#include <ostream>
#include <string>
#include <vector>
#include <boost/variant.hpp>

//////////////////////////////////////////////////////////////////////////////////////
#include <chrono>
typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::time_point<std::chrono::system_clock> s_clock;
typedef Clock::time_point tp;
//////////////////////////////////////////////////////////////////////////////////////


namespace usfm {

	struct StatisticItem {
		std::string _name;
		boost::variant<double, int, std::string> _value;

		StatisticItem(std::string name, boost::variant<double, int, std::string> value);
	};

	std::ostream& operator<< (std::ostream& out, const StatisticItem& s); 


	class Statistic {
	public:
		tp _first_timestamp;
		tp _last_timestamp;
		
		Statistic();

		// vector of statistics
		std::vector<StatisticItem> _items;
		void addItem(std::string name, boost::variant<double, int, std::string> value);
		void addTimestampItem(std::string name);
		void clear();

		boost::variant<double, int, std::string> getItem(std::string name);

		// time
		void updateLastTimestamp();
		double timeDuration();
		double timeDuration(const tp& from, const tp& to);
		
	};

}

#endif