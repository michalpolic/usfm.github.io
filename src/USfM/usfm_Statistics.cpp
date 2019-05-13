// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Statistics.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_Statistics.hpp"

namespace usfm {

	// structure for holding the values of any type
	StatisticItem::StatisticItem(std::string name, boost::variant<double, int, std::string> value):
		_name(name), _value(value) {}

	std::ostream& operator<< (std::ostream& out, const StatisticItem& s) {
		return out << s._name << ": " << s._value;
	}


	// init of the statistics
	Statistic::Statistic() {
		_first_timestamp = Clock::now();
		_last_timestamp = _first_timestamp;
	}

	// add/clear the statistics
	void Statistic::addItem(std::string name, boost::variant<double, int, std::string> value) {
		_items.push_back(StatisticItem(name, value));
	}
	void Statistic::addTimestampItem(std::string name) {
		addItem(name, timeDuration());
	}
	void Statistic::clear() {
		_items.clear();
	}

	boost::variant<double, int, std::string> Statistic::getItem(std::string name) {
		for (int i = 0; i < _items.size(); ++i) {
			if (strcmp(_items[i]._name.c_str(), name.c_str()))
				return _items[i]._value;
		}
		throw std::runtime_error("The required item is not in statistic object.");
	}

	void Statistic::updateLastTimestamp() {
		_last_timestamp = Clock::now();
	}

	// time function wich update the last_timestamp
	double Statistic::timeDuration() {
		double time = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - _last_timestamp).count() * 1e-3;
		updateLastTimestamp();
		return time;
	}

	// compute the duration between two timestamps
	double Statistic::timeDuration(const tp& from, const tp& to) {
		return std::chrono::duration_cast<std::chrono::milliseconds>(to - from).count() * 1e-3;
	}
}