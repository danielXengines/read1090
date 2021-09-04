#include <iostream>
#include <ostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <iomanip>
#include <regex>
#include <ctime>
#include <vector>
#include <FLIGHT_STATS.h>

//using namespace boost::filesystem;

FLIGHT_STATS::FLIGHT_STATS(void) {
}

FLIGHT_STATS::~FLIGHT_STATS(void) {
}

// function used to sort file by timestamp
void FLIGHT_STATS::sort_by_date(void){
	std::sort(dirList.begin(), dirList.end(), [] (dirStruct lhs, dirStruct rhs) {
			return lhs.local_time < rhs.local_time;});
}

// return list of available directories in the given path
std::vector<std::string> FLIGHT_STATS::get_dirList(path p) {
	std::vector<std::string> dirStrings; // vector struct to store the file names and local time
	dirList.clear();                     // this is a global variable, clear it to ensure it is empty
	dirStruct tempData;
	std::string fileRec;
	struct tm sub_tm;
	mktime(&sub_tm);    // must be done otherwise first result returned will be incorrect

	try {
		if (exists(p)) {
			if (is_directory(p)) {
				directory_iterator end_iter;
				unsigned long dir_count = 0;
				// iterate through all directories in given path and save to vector dirList {fileIN, local_time}. Unsorted.
				for (directory_iterator dir_itr(p); dir_itr != end_iter; ++dir_itr) {
					if (is_directory(dir_itr->status())) {
						++dir_count;
					        fileRec = dir_itr->path().filename().string();
					        tempData.fileIN = fileRec;                // store the file name
					        std::string loc_str = "data_";
					        std::size_t pos = fileRec.find(loc_str);
					        std::string date_str = fileRec.substr(pos+loc_str.length());
					        strptime(date_str.c_str(), "%d%b%Y", &sub_tm);
					        tempData.local_time = mktime(&sub_tm);
					        dirList.push_back(tempData);
					}
				}
				//std::cout << dir_count << " directories found in " << p << "." << std::endl;
			} else {
				std::cout << "Path " << p << " exists, but is not a directory." << std::endl;
			}
		} else {
			std::cout << "Path " << p << " does not exist. Return directory list is empty." << std::endl;
		}
	} catch (const filesystem_error& ex) {
		std::cout << ex.what() << std::endl;
	}
	// sort dirList by local_time
	sort_by_date();
	// iterate thru dirList and save string for each dir
	for (auto it = std::begin(dirList); it != std::end(dirList); ++it) {
		dirStrings.push_back(it->fileIN);
	}
	return dirStrings;
}

// function to total flight stats per day
FLIGHT_STATS::daily_count FLIGHT_STATS::read_logFiles(path p) {
        daily_count var1;
        try {
                if (exists(p)) {
                        directory_iterator end_iter;
                        std::string readFileName;
                        std::regex toff ("(.*)(TOFF)(.*)");
                        std::regex lndg ("(.*)(LNDG)(.*)");
                        std::regex fovr ("(.*)(FOVR)(.*)");
                        uint toff_ctr = 0;
                        uint lndg_ctr = 0;
                        uint fovr_ctr = 0;
                        std::string loc_str = "data_";                // find part of string that says "data_"
                        std::size_t pos = p.string().find(loc_str);
                        var1.date_str = p.string().substr(pos+loc_str.length()); // find the substring after, this is the date
                        for (directory_iterator dir_itr(p); dir_itr !=end_iter; ++dir_itr) {
                                readFileName = (dir_itr->path().filename()).string();
                                if (std::regex_match (readFileName, toff)) {
                                        toff_ctr++;
                                }
                                if (std::regex_match (readFileName, lndg)) {
                                        lndg_ctr++;
                                }
                                if (std::regex_match (readFileName, fovr)) {
                                        fovr_ctr++;
                                }
                        }
                        var1.toff = toff_ctr;
                        var1.lndg = lndg_ctr;
                        var1.fovr = fovr_ctr;
                } else {
                        std::cout << p << " does not exist. Return struct is empty" << std::endl;
                }
        } catch (const filesystem_error& ex) {
                std::cout << ex.what() << std::endl;
        }
        return var1;
}

// consolidate all data deletes the existing file and creates a new file with all data
void FLIGHT_STATS::consol_all(path p) {
	struct tm tm;

	path sub_p(p);
        std::string dataStr = "";
        dataStr.append("epochTime, takeoff, landing, flyover\n");

        // output file
        path logFile (csvDailyStatFile);
        if (exists(logFile)){
                remove(logFile);
        }
        std::ofstream file;
        file.open(csvDailyStatFile, std::ofstream::out | std::ofstream::app);
        try {
                if (exists(p)) { // input file
                        std::vector<std::string> dirAvail = get_dirList(p);
                        std::cout << dirAvail.size() << " directories received." << std::endl;
                        unsigned int dirCnt = 0;
                        std::cout << "Reading directory, writing data to summary file";

                        for (auto it = std::begin(dirAvail); it != std::end(dirAvail); ++it) {
                                sub_p /= *it;     // assign date specific sub-dir to path
                                daily_count tempVar = read_logFiles(sub_p); // returns a struct that is date_str, toff, lndg & fovr
                                dirCnt++;         // count the number of directories read
                                sub_p = p;        // assign flight_data dir back to path
                                // format time for Google-charts plotting
                                if (strptime(tempVar.date_str.c_str(), "%d%b%Y", &tm)) {
                                        time_t epochTime = mktime(&tm);
                                        //int d = tm.tm_mday, m = tm.tm_mon + 1, y = tm.tm_year + 1900; 
                                        dataStr.append(std::to_string(epochTime));
                                        dataStr.append(", ");
                                } else {
                                        dataStr.append(std::to_string(1900));
                                        dataStr.append(", ");
                                }
                                // append the actual flight numbers for the day
                                dataStr.append(std::to_string(tempVar.toff));
                                dataStr.append(", ");
                                dataStr.append(std::to_string(tempVar.lndg));
                                dataStr.append(", ");
                                dataStr.append(std::to_string(tempVar.fovr));
                                std::cout << ".";
                                file << dataStr << std::endl;
                                dataStr.erase(dataStr.begin(), dataStr.end());
                        }
                        std::cout << std::endl;
                        //file << dataStr;
                } else {
                        std::cout << p << " does not exist" << std::endl;
                }
        } catch (const filesystem_error& ex) {
                std::cout << ex.what() << std::endl;
        }
        file.close();
	const std::string file_dest = "/var/www/flightStats/FlightData.csv";
        copy_file(csvDailyStatFile, file_dest, copy_option::overwrite_if_exists);
}

// consolidate last data appends to the existing file most recent data
void FLIGHT_STATS::consol_last(path p) {
        struct tm tm;
	bool writeNewDay = true;                                        // set to true when writing to file on a new day
	int lastPos = 0;                                                // last position in the file for seek
        path sub_p(p);
        std::string dataStr = "";
	std::vector<std::string> readBuffer;                            // buffer to store all file entries
	std::string lineStr;                                            // each line of the input file
        //dataStr.append("epochTime, takeoff, landing, flyover\n");
	
        // output file
	std::ifstream fileRead;
        path logFile (csvDailyStatFile);
	
	// find the date of the last entry. If this matches current date, then overwrite. If not, write on next line
        if (exists(logFile)){
		//std::cout << "log file exists..." << std::endl;
                fileRead.open(csvDailyStatFile, std::ofstream::in);
		while (getline(fileRead, lineStr)) {
			readBuffer.push_back(lineStr);
		}
		
		// check the last entry
		int lastIdx = readBuffer.size()-1;
		lineStr = readBuffer[lastIdx];
		size_t pos = lineStr.find(",");
                char readDate[10];
		std::string epoch_str = lineStr.substr(0, pos);
                strptime(epoch_str.c_str(), "%s", &tm);                                          // cast to str
                strftime(readDate, sizeof(readDate), "%d", &tm);                                 // get date of last entry
                
		// compare last date with current date
		time_t now = time(0);
                struct tm *sysNow = localtime(&now);
                if (atoi(readDate) == (int)sysNow->tm_mday) { // last entry is from today, delete the last element
                        readBuffer.erase(readBuffer.end());
		}
                // close the input file and delete it
		fileRead.close();
		remove(logFile);
        }
        
        // create and open output file
        std::ofstream file;
        file.open(csvDailyStatFile, std::ofstream::out | std::ofstream::app);
        try {
                if (exists(p)) { // input file
                        std::vector<std::string> dirAvail = get_dirList(p);
                        std::cout << dirAvail.size() << " directories received." << std::endl;
                        unsigned int dirCnt = 0;
                        //std::cout << "Reading directory, writing data to summary file" << std::endl;

                        unsigned i = dirAvail.size() - 1;           // get the last element in the vector
                        sub_p /= dirAvail[i];                       // assign date specific sub-dir to path
                        daily_count tempVar = read_logFiles(sub_p); // returns a struct that is date_str, toff, lndg & fovr
                        dirCnt++;                                   // count the number of directories read
                        sub_p = p;                                  // assign flight_data dir back to path
                        // format time for Google-charts plotting
                        if (strptime(tempVar.date_str.c_str(), "%d%b%Y", &tm)) {
				time_t epochTime = mktime(&tm);
                                //int d = tm.tm_mday, m = tm.tm_mon + 1, y = tm.tm_year + 1900;
                                dataStr.append(std::to_string(epochTime));
                                dataStr.append(", ");
                        } else {
				dataStr.append(std::to_string(1900));
                                dataStr.append(", ");
                        }
			// append the actual flight numbers for the day
                        dataStr.append(std::to_string(tempVar.toff));
                        dataStr.append(", ");
                        dataStr.append(std::to_string(tempVar.lndg));
                        dataStr.append(", ");
                        dataStr.append(std::to_string(tempVar.fovr));
			readBuffer.push_back(dataStr);             // update vector with most recent count
			// write data to file
			for (auto it = std::begin(readBuffer); it != std::end(readBuffer); ++it) {
				file << *it << std::endl;
			}

			readBuffer.erase(readBuffer.begin(), readBuffer.end());                            // clear the readBuffer
                        dataStr.erase(dataStr.begin(), dataStr.end()); // erase the contents of the last entry
                } else {
                        std::cout << p << " does not exist" << std::endl;
                }
        } catch (const filesystem_error& ex) {
                std::cout << ex.what() << std::endl;
	}
        file.close();                                             // close the output file
        const std::string file_dest = "/var/www/flightStats/FlightData.csv";
        copy_file(csvDailyStatFile, file_dest, copy_option::overwrite_if_exists);
}

