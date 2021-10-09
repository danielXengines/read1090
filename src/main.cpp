#include <iostream>
#include <string>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <sstream>
#include <vector>
#include <algorithm>
#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <iterator>
#include <boost/algorithm/string.hpp>
#include <functional>
#include <math.h>
#include <ctime>
#include <unistd.h>

#include <CD1090.h>
#include <TIMER.h>
#include <FLIGHT_STATS.h>

using json = nlohmann::json;

void printUsage(void) {
    std::cout << "usage ./readD1090 [int no_iter]" << std::endl;
    std::cout << "0 < no_iter < 8000000" << std::endl;
}

int main(int argc, char **argv)
{
    std::ofstream errLog;
    time_t now = time(0);                                    // used to time saving updating stats every 15 mins
    tm *ltm = localtime(&now);

    //std::time_t t = std::time(0);
    //std::string dt = ctime(&t);
    // Load the data processing class
    std::string configFile = "../config.txt";
    CD1090 dProc(configFile);
    FLIGHT_STATS fs;                                          // used for saving summary of flight data
    path p("../../FlightData");
    fs.consol_all(p);

    while (dProc.runLoop) {

        dProc.acquireJSONblock();                              // acquire JSON data from server
	    dProc.JSONblock2Vector();                              // convert JSON block to vector struct
	    dProc.runTracker();                                    // run the tracker, also saves tracks to data directory
	    dProc.broadcastData();                                 // broadcast data over UDP
	    //dProc.broadcastTOL();
	    dProc.print2screen();                                  // print tracked results to screen

    
	    // update flight stats data set every 15 mins
	    now = time(0);
	    ltm = localtime(&now);
	    
        /* todo : test & move this process into a thread */
        if ((ltm->tm_min % 15 == 0 ) && (ltm->tm_sec > 58)) {
		    fs.consol_last(p);
		    sleep(2);
	    }
        
        /* todo : test & move this process into a thread */
        if (ltm->tm_sec % 30 == 0 ) {
		    dProc.print2file();
	    }

    }
  return 0;
}
