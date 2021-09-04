#ifndef FLIGHT_STATS_H
#define FLIGHT_STATS_H

#include <boost/filesystem.hpp>
#include <iostream>

using namespace boost::filesystem;

class FLIGHT_STATS {
	private:
		void sort_by_date(void);            // function used to sort the directory list by date
		const std::string csvDailyStatFile = "FlightData.csv";    // file to save csv data

	public:
		// date string, no toff, no lndg, no fovr
                struct daily_count {
                        std::string date_str;
                        int toff;
                        int lndg;
                        int fovr;
                };

                // struct defining directory name and date
                struct dirStruct {
                        std::string fileIN;
                        time_t local_time;
                };

                // used to save directory names and directory date
                std::vector<dirStruct> dirList;

		std::vector<std::string> get_dirList(path p);        // function that returns list of directories sorted
		daily_count read_logFiles(path p);                   // function that returns the total flights recorded each day
		void consol_all(path p);                             // consolidates all data and writes to csv file
		void consol_last(path p);                            // consolidates last date data and appends to csv file
		FLIGHT_STATS();
		~FLIGHT_STATS();

};
#endif
