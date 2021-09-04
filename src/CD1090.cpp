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
#include <iomanip>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <math.h>
#include <ctime>
#include <ostream>
#include <cctype>
#include <locale>

#define CSV_IO_NO_THREAD
#include <csv.h>

#include <CD1090.h>
#include <TIMER.h>
#include "udp_client_server.h"

#define MAX_LOGFILE_SIZE 9216

using json = nlohmann::json;
const float  PI_F=3.14159265358979f;

CD1090::CD1090(std::string configFile) {
    READBuffer.clear();        // clear the read buffer

    // ensure vectors are empty at initialization
    flight_vector.erase(flight_vector.begin(), flight_vector.end());    // data received from ADSB broadcast
    TOFF_vector.erase(TOFF_vector.begin(), TOFF_vector.end());          // take off trajectory for terminated tracks
    LNDG_vector.erase(LNDG_vector.begin(), LNDG_vector.end());          // landing trajectory for terminated tracks

    std::cout << "\033[H" << std::endl;        // mover cursor to upper left corner
    std::cout << "\033[2J" << std::endl;       // clear screen

    baseDir.assign(boost::filesystem::current_path().string());                 // get the current path
    std::cout << "\033[94mCurrent working directory is ..." << baseDir << "\033[0m" << std::endl;
    //dataDir.assign("training");
    //check_dataDir(dataDir);

    int fileStat   = cxFile_exists(configFile);
    int configStat = 0;
    if (fileStat == 0) {configStat = CD1090::parseConfigData(configFile);}  // load configuration file
    else {
	std::cout << "[\033[91mEnsure that the file " << configFile << " exists in the directory above build.\033[0m" << std::endl;
        configStat = 1;
    }
    int IPStat     = cx_ip_addr();
    int csvStat    = cxFile_exists(ICAO_csv);
    if (csvStat == 0) {CD1090::buildICAO();}     // read in CSV file and save to vector database
    if ((configStat == 0) && (IPStat == 0) && (csvStat == 0)) {runLoop=true;}
    else {
       runLoop=false;
       std::cout <<"[\033[91mSetup failed. Unable to run. Check status log \033[0m" << std::endl;
    }
    track_vector.reserve(100);                 // reserve 100 tracks in the tracker object
    totalIter = noIter;
    logFile("START:");
    totalTime.reset();                         // reset clock to capture total runtime
}

CD1090::~CD1090(void) {
    flight_vector.clear();                    // clear the contents of the flight vector
    flight_vector.shrink_to_fit();            // free memory space used by the vector
    std::cout << "\033[H" << std::endl;       // move cursor to upper left corner
    std::cout << "Program terminated normally" << std::endl;
    logFile("STOP:");
    float localttlTime = static_cast<float>(totalTime.elapsed());
    logFile("Elapsed:" + std::to_string(localttlTime));
    float avgFPS = static_cast<float>(totalIter) / localttlTime;
    std::cout << "Average FPS: " << avgFPS << " "  << std::endl; 
    logFile("Average FPS:" + std::to_string(avgFPS));
}


static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// function to acquire JSON block data from server
void CD1090::acquireJSONblock(void){
    CURL *curl;
    CURLcode RES;
    READBuffer.clear();                               // READBuffer needs to be cleared on each call
    curl = curl_easy_init();
    frameTime.reset();
    if(curl){
        // read dump1090 output string
	curl_easy_setopt(curl, CURLOPT_URL, IPaddress.c_str());
	curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);                 // timeout if unable to complete transaction in 2secs
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &READBuffer);
	RES = curl_easy_perform(curl);
	curl_easy_cleanup(curl);
    }
    // if running for a fixed number of iterations, decrement counter on each read
    if (totalIter > 0) {
	    noIter--;
	    if (noIter > 0) {
		runLoop = true;
	    } else {
		runLoop = false;
	    }
    }
    // check status of the curl read operation
    if ((int)RES != 0) {
	logFile("Server read error.");
	dataReady = false;
    } else dataReady = true;

    if (READBuffer.length() < 1) dataReady = false;
    else dataReady = true;
}

// function used to sort adsb vector by bearing
void CD1090::sort_by_bearing(void){
    std::sort(track_vector.begin(), track_vector.end(), [] (ADSB_brcast lhs, ADSB_brcast rhs){
    return lhs.bearing < rhs.bearing;
    });
}

// function used to sort adsb vector by recent update
void CD1090::sort_by_seen(void){
    std::sort(track_vector.begin(), track_vector.end(), [] (ADSB_brcast lhs, ADSB_brcast rhs){
    return lhs.seen < rhs.seen;
    });
}

// function used to sort adsb vector by altitude
void CD1090::sort_by_altitude(void){
    std::sort(track_vector.begin(), track_vector.end(), [] (ADSB_brcast lhs, ADSB_brcast rhs){
    return lhs.altitude < rhs.altitude;
    });
}

// function used to sort adsb vector by score
void CD1090::sort_by_score(void){
    std::sort(track_vector.begin(), track_vector.end(), [] (ADSB_brcast lhs, ADSB_brcast rhs){
    return lhs.score > rhs.score;
    });
}

// function to parse JSON block data and return a vector
void CD1090::JSONblock2Vector(void){
    if (dataReady) {
        try {
	    CD1090::ADSB_brcast rx_bcast;           // struct to sort data from each json entry
            auto jX = json::parse(READBuffer);      // json stream with all entries
            auto noRX_bcast = jX.size();            // total number of broadcasts received
            uint valid_idx = 0;                     // number of entries with position broadcast
            uint max_track = 0;                     // total number of entries
            std::vector<CD1090::icao>::iterator it; // iterator from icao database search
            flight_vector.clear();                  // clear the vector
            // iterate over message block to parse individual messages
            for (auto it: jX) {
	        // results are pruned to save broadcasts with valid positions
	        if (it["lat"] > 0){
	            // Check if flight name is contained in broadcast
	            std::string tempStr = it["flight"];
	            uint lenDelta = 9 - tempStr.length();   // Determine the difference in length, 9Char max
	            if (lenDelta > 6){
	                rx_bcast.flight = "xxxxxxxxx";
	            } else {
		        tempStr.insert(tempStr.length(), "        ", lenDelta);
	                rx_bcast.flight = tempStr;
	            }
		    rx_bcast.altitude = it["altitude"];
	            rx_bcast.hexcode = it["hex"];
	            rx_bcast.lat = (float)it["lat"];
	            rx_bcast.lon = (float)it["lon"];
	            rx_bcast.messages = it["messages"];
	            rx_bcast.seen = it["seen"];
	            rx_bcast.squawk = it["squawk"];
	            rx_bcast.bearing = it["track"];
	            rx_bcast.validposition = it["validposition"];
	            rx_bcast.validtrack = it["validtrack"];
	            rx_bcast.vert_rate = it["vert_rate"];
		    rx_bcast.latTrk.push_back((float)it["lat"]);
		    rx_bcast.lonTrk.push_back((float)it["lon"]);
		    rx_bcast.altTrk.push_back(it["altitude"]);
	            flight_vector.push_back(rx_bcast);
	            valid_idx++;
	        }
           }
        }
        catch (json::parse_error& e) {
	    logFile(e.what());           // record parse error in log file
	    dataReady = false;
	}
    }
}

// tracker method
void CD1090::runTracker(void) {
    if (dataReady) {
	int onTracker_limit  = 45;                    // seen time limit for an object that already exists on the tracker
        int newTracker_limit = 30;                    // seen time limit for a new object
        std::vector<std::string> dummyStr;            // used to retrieve flight & aircraft type information from icao db
        CD1090::ADSB_brcast acTrack;                  // struct to record individual aircraft data
        for (std::vector<CD1090::ADSB_brcast>::iterator it_flight = flight_vector.begin(); it_flight != flight_vector.end(); ++it_flight){
	    // search if current aircraft is on the track vector
            std::vector<CD1090::ADSB_brcast>::iterator it_track = std::find_if (track_vector.begin(), track_vector.end(),
                                     boost::bind ( &ADSB_brcast::hexcode, _1 ) == it_flight->hexcode );
            if (it_track != track_vector.end()) { // aircraft is an existing tracked object
		it_track->altitude = it_flight->altitude;
	        it_track->lat      = it_flight->lat;
	        it_track->lon      = it_flight->lon;
	        it_track->seen     = it_flight->seen;
	        it_track->bearing  = it_flight->bearing;
		int noPts = (*it_track).latTrk.size();
		if (noPts > 1) { // if more than one point, only save data if position is updated
		    if ((it_flight->lat != it_track->latTrk.at(noPts-1)) || (it_flight->lon != it_track->lonTrk.at(noPts-1))) {
		        it_track->latTrk.push_back(it_flight->lat);
		        it_track->lonTrk.push_back(it_flight->lon);
		        it_track->altTrk.push_back(it_flight->altitude);
			it_track->brgTrk.push_back(it_flight->bearing);
		    }
		} else { // if less that 2 points save data anyway
		    it_track->latTrk.push_back(it_flight->lat);
                    it_track->lonTrk.push_back(it_flight->lon);
                    it_track->altTrk.push_back(it_flight->altitude);
		    it_track->brgTrk.push_back(it_flight->bearing);
		}

	        it_track->frameCtr++;
	        if (it_flight->seen < onTracker_limit) {
		    // aircraft is tracked and within tracker limit
		    it_track->score = (1/it_track->frameCtr + 0.99*it_track->score  + 1/(it_flight->seen+0.99));
	        } else {
		    // aircraft is tracked but outside tracker limit
		    it_track-> score = it_track->score -(0.0015* it_track->seen);    // reduce score by x% of time since last update
	        }
 
            } else {
	        if (it_flight->seen < newTracker_limit) {
		    // aircraft is not a tracked object and shall be added to the tracker
		    dummyStr = findByHEX(it_flight->hexcode);
		    acTrack = *it_flight;
                    acTrack.airline = dummyStr[1];            // assign airline
		    acTrack.typeAC  = dummyStr[0];            // assign aircraft type
		    acTrack.typeAC.resize(9);                 // allow maximum of 8 characters for typeAC
		    track_vector.push_back(acTrack);
	        } else {
		    // aircraft is not a tracked object but is outside the time limit, no action shall be taken
	        }
            }
	    it_track-> score -= 0.1;                          // special condition to remove entries that only appear during track initialization and no subsequent entries
        }

        // search vector for entries with score < 0
        bool searchCont = true;
        while (searchCont) {
	    std::vector<CD1090::ADSB_brcast>::iterator it_track = std::find_if (track_vector.begin(), track_vector.end(),
                         boost::bind ( &ADSB_brcast::score, _1 ) <= 0 );
	    if (it_track != track_vector.end()) {
                searchCont = true;
		if (saveTracks) writeTrack(*it_track);                        // record flight trajectory to log file
	        track_vector.erase(it_track);
	    } else {
	        searchCont = false;
	    }
        }
        if (sortOption.find("BEARING") == 0) sort_by_bearing();
        if (sortOption.find("SEEN")  == 0) sort_by_seen();
        if (sortOption.find("ALTITUDE") == 0) sort_by_altitude();
        if (sortOption.find("SCORE") == 0) sort_by_score();
    }
}

// calculate aircraft bearing from gps coordinates
float CD1090::calc_gps_bearing(CD1090::ADSB_brcast acTrack, int idx) {
    int noPts = acTrack.lonTrk.size();
    float brng = -99;
    if (idx < noPts-2) {
        float lon2   = 0;
        float lon1   = 0;
        float lat2   = 0;
        float lat1   = 0;
        float theta1 = 0;
        float theta2 = 0;
        float deltalambda = 0;
        float X, Y;
        if (noPts >= 2) { 
            lon2   = acTrack.lonTrk.at(idx);
	    lon1   = acTrack.lonTrk.at(idx-1);
	    lat2   = acTrack.latTrk.at(idx);
	    lat1   = acTrack.latTrk.at(idx-1); 
	    theta1 = deg2rad(lat1);
            theta2 = deg2rad(lat2);
            deltalambda = deg2rad(lon2-lon1);
            X = cos(theta2) * sin(deltalambda);
            Y = cos(theta1) * sin(theta2) - sin(theta1) * cos(theta2) * cos(deltalambda);
            brng = atan2(X,Y) * 180.0/PI_F;
            if (brng < 0) brng += 360;
        } else { logFile("bearing calculation error. Less than 2 points in track"); }	
    } else { logFile("bearing calculation error. Requested index out of range");}
    return brng;
}

float CD1090::deg2rad(float degVal) {
    return (degVal * PI_F) / 180.0;
}

float CD1090::calc_gps_dist(float lon1, float lat1, float lon2, float lat2) {
        float R = 6371e3;                                                 //  radius of the earth in metres
        float theta1 = deg2rad(lat1);                                     // convert coordinate data to radians
        float theta2 = deg2rad(lat2);
        float deltatheta = deg2rad(lat2-lat1);
        float deltalmbda = deg2rad(lon2-lon1);
        // ---- square of half the cord length between coordinates ---- //
        float a     = sin(deltatheta/2) * sin(deltatheta/2) + cos(theta1) * cos(theta2) * sin(deltalmbda/2) * sin(deltalmbda/2);
        // ---- angular distance in radians ------ //
        float c     = 2 * atan2(sqrt(a), sqrt(1-a));
        float horz_d = R * c;
        return horz_d / 1e3;
}

// function to print data to console
void CD1090::print2screen(void){
    if (dataReady) {
        uint noLines = 0;
        uint clrIdx  = 0;
        int localClear_time = (int)clearTime.elapsed();
        if (((localClear_time  % 1) == 0) && (localClear_time > 0)) {
	    clearTime.reset();                         // reset the total time
	    std::cout << "\033[2J" << std::endl;       // clear screen
        }
        std::string fps = std::to_string(1/frameTime.elapsed());

        fps.resize(4); 
        // print the results out on screen
        std::cout << "FPS: " <<  std::setw(4) << std::setfill('0') << fps << std::endl;
        noLines ++;
	if (noIter > 0) noIter --;                                  // counter is only decrement count if input value is positive
	// only print remaining iterations when finite
	if (noIter > 0) {
            std::cout << "Remaining iterations: " <<  std::setw(7) << std::setfill('0') 
	          << std::to_string(noIter) << std::endl;
	    noLines ++;
	}
        std::cout << "\033[34;1;4mFLIGHT  \tHEX\tLAT\tLON\tALT\tBRNG\tSEEN\tAIRCRAFT\tAIRLINE\033[0m" << std::endl;
        noLines ++;
        // Add data to string buffer
        for (std::vector<CD1090::ADSB_brcast>::iterator it = track_vector.begin(); it != track_vector.end(); ++it){
            if (it->seen > 45){
	        clrIdx = 90;
	    } else if ((it->seen <=45) && (it->seen > 30)){
	        clrIdx = 33;
	    } else {
	        clrIdx = 0;
	    }
	    std::cout << "\033[" << clrIdx << "m" << it->flight << "\t" << it->hexcode << "\t" 
		      << std::setprecision(5) <<  it->lat << "\t" << std::setprecision(5) << it->lon 
		      << "\t" << std::setw(5) << std::right << it->altitude << " "
		      << "\t" << std::setw(3) << std::right << it->bearing 
		      << "\t" << std::setw(3) << std::right << it->seen 
	              << "\t" << it->typeAC << "\t\t" << it->airline <<"\033[0m" << std::endl;
            noLines++;
        }
        noLines++;
        std::cout << "\033[" << noLines << "A" << std::endl;
    }
}

// perform udp broadcast of tracked data
void CD1090::broadcastData(void) {
        std::time_t time = std::time(nullptr);
        tm *ltm = localtime(&time);               // used to check current time & broadcast toff & lndg vectors at x sec intervals
	// variables used for UDP broadcast
	int brcast_port = 9041;
        std::string brcast_addr="192.168.178.49";
	using UDPclient = udp_client_server::udp_client;
	UDPclient clientObj(brcast_addr, brcast_port);
        int UDPsocket = clientObj.get_socket();
        int UDPport   = clientObj.get_port();
        std::string UDPaddr = clientObj.get_addr();
	//size_t msg_len;                                       // length of message being broadcast

	
	// broadcast active flight data if track_vector contains data
	if (track_vector.size() > 0 ) {
		json jDataActive;                                 // json container to hold data from vector
		std::string sActive("{\"aircraft\":{");     // json data will be dumped to this string 

		for (std::vector<ADSB_brcast>::iterator it_flight=track_vector.begin(); it_flight !=track_vector.end(); ++it_flight) {
			jDataActive["flight"]   = it_flight->flight;
			//jDataActive["hexcode"]  = it_flight->hexcode;
			jDataActive["altitude"] = it_flight->altitude;
			jDataActive["lat"]      = it_flight->lat;
			jDataActive["lon"]      = it_flight->lon;
			jDataActive["bearing"]  = it_flight->bearing;
			jDataActive["airline"]  = it_flight->airline;
			jDataActive["typeAC"]   = it_flight->typeAC;
			jDataActive["seen"]     = it_flight->seen;	
			sActive.append("\"");
			sActive.append(it_flight->hexcode);
			sActive.append("\":");
			sActive.append(jDataActive.dump());            // dump the json data to string
			sActive.append(", ");
		}
		sActive.erase(sActive.end()-2, sActive.end());         // remove last comma
		sActive.append("}}\0");                                // terminate json array
		const char *cStr = sActive.c_str();                    // convert data to c_str()
		size_t msg_len = strlen(cStr);                                // determine message length
		clientObj.send(cStr, msg_len);                         // send message
		sActive.erase(sActive.begin(), sActive.end());
	}
}

// perform udp broadcast of tracked data
void CD1090::broadcastTOL(void) {
        std::time_t time = std::time(nullptr);
        tm *ltm = localtime(&time);               // used to check current time & broadcast toff & lndg vectors at x sec intervals
        // variables used for UDP broadcast
        int brcast_port = 9041;
        std::string brcast_addr="192.168.178.49";
        using UDPclient = udp_client_server::udp_client;
        UDPclient clientObj(brcast_addr, brcast_port);
        int UDPsocket = clientObj.get_socket();
        int UDPport   = clientObj.get_port();
        std::string UDPaddr = clientObj.get_addr();
        //size_t msg_len;                                       // length of message being broadcast
	//
	// broadcast landing data at 45 sec intervals and if the lndg vector contains data
        if ( ((ltm->tm_sec > 45) && (ltm->tm_sec < 49)) && (LNDG_vector.size() > 0) ) {
                json jDataLNDG;                               // json container to hold data from vector
                std::string sLNDG("{\"lndg\":{");             // json data will be dumped to this string for broadcast
                //std::cout << " ------------------- in here -------------------- " << LNDG_vector.size() << " entries" << std::endl;

                for (std::vector<flight_path>::iterator it_lndg=LNDG_vector.begin(); it_lndg !=LNDG_vector.end(); ++it_lndg) {
                        jDataLNDG["latTrk"] = it_lndg->latTrk;
                        jDataLNDG["lonTrk"] = it_lndg->lonTrk;
                        sLNDG.append("\"");
                        sLNDG.append(it_lndg->hexcode);
                        sLNDG.append("\":");
                        sLNDG.append(jDataLNDG.dump());
                        sLNDG.append(", ");
                }
                //std::cout << sLNDG << std::endl;
                sLNDG.erase(sLNDG.end()-2, sLNDG.end());              // remove the last comma
                sLNDG.append("}}\0");                                 // terminate json array
                const char *dStr = sLNDG.c_str();                                // convert data to c_str()
                size_t msg_len2 = strlen(dStr);                               // determine message length
                std::cout << clientObj.send(dStr, msg_len2) << std::endl;                        // send message
        }

        // broadcast take-off data at 50 sec intervals and if the lndg vector contains data
        if (((ltm->tm_sec % 50) == 0) && (TOFF_vector.size() > 0)) {

        }
}



// returns the aircraft type and operator as a string vector
std::vector<std::string> CD1090::findByHEX(std::string hexRef){
    std::vector<std::string> dummyStr;
    std::vector<CD1090::icao>::iterator it = std::find_if (icao_vector.begin (), icao_vector.end (),
                                     boost::bind ( &icao::hexcode, _1 ) == hexRef );
    if (it != icao_vector.end()) {
	    std::string acType = it->ac_type;
	    std::string airline = it->airline;
	    airline.resize(24);
	    
	    acType = formatACmodel(acType);
            dummyStr.push_back(acType);
            dummyStr.push_back(airline);
    } else {
            dummyStr.push_back("no data");
	    dummyStr.push_back("no data");
    }
    return dummyStr;
}

// format the aircraft model. Remove maufacturer text from name
std::string CD1090::formatACmodel(std::string acType) {
    boost::to_upper(acType);
    std::size_t found = acType.find("BOEING");
    if (found!=std::string::npos) {
        acType.replace(found, 6, "");
    }
    found = acType.find("AIRBUS");
    if (found!=std::string::npos) {
        acType.replace(found, 6, "");
    }
    acType.resize(7);
    return acType;
}

// reads in a csv file using google header and saves it to a struct vector
// note that the input csv file used has been modified and only has 5 colums
void CD1090::buildICAO(void) {
    io::CSVReader<5> in (ICAO_csv);
    std::cout << "\033[37mRetrieving csv ICAO database ...\033[0m" << std::endl; 
    in.read_header(io::ignore_extra_column, "hexcode","tailNumber","manuf","ac_type","operator");
    std::string col0, col1, col2, col3, col4;
    CD1090::icao d;                                                 // dummy struct to organize input
    
    Timer tmr;                                                      // initialize timer object
    while(in.read_row(col0,col1,col2,col3,col4)){
            //trim(col2);                                           // trim whitespace before and after
	    //trim(col3);
	    //trim(col4);
	    d.hexcode = col0;
            d.tailNum = col1;
            d.manuf   = col2;
            d.ac_type = col3;
            d.airline = col4;
            icao_vector.push_back(d);
    }
    std::cout << "Loading " << icao_vector.size() << " aircraft entries from csv file took "  << tmr.elapsed()
              << "sec" << std::endl;
}
// ------------------------------------ Parsing and data checking -------------------------------- //
// parse configuration data from config file
int CD1090::parseConfigData(std::string fileName) {
    int status = 0;
    // define the input arguments that will be set from the config file
    std::vector <std::string> inputArgs;
    inputArgs.push_back("IP_ADDRESS");
    inputArgs.push_back("ICAO_DATA");
    inputArgs.push_back("ITERATIONS");
    inputArgs.push_back("SORT");
    inputArgs.push_back("SAVE_TRACKS");
    inputArgs.push_back("REF_LON");
    inputArgs.push_back("REF_LAT");
    inputArgs.push_back("DATA_DIR");

    int noArgs = inputArgs.size();
    // open the file and read line by line
    std::ifstream file(fileName);
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            if (line.find(inputArgs[0]) == 0) {
                 line.replace(0,inputArgs[0].length()+1,"" );        // remove name and whitespace
                 //std::cout << "[\033[92mIP address\033[0m] : " << "\033[32m" << line << "\033[0m" << std::endl;
		 IPaddress.assign(line);
		 logFile("IP Address: " + line);
		 noArgs--;
            } else if (line.find(inputArgs[1]) == 0) {
                line.replace(0,inputArgs[1].length()+1, "");        // remove name and whitespace
                //std::cout << "[\033[92mICAO CSV\033[0m] : " << "\033[32m" << line << "\033[0m" << std::endl;
		ICAO_csv.assign(line);
		logFile("ICAO file: " + line);
		noArgs--;
            } else if (line.find(inputArgs[2]) == 0) {
                line.replace(0,inputArgs[2].length()+1, "");        // remove name and whitespace
		noIter = std::stoi(line);
		if (noIter > 0) {
                    std::cout << "[\033[92mNO ITER\033[0m] : " << "\033[32m" << line << "\033[0m"  << std::endl;
		} else {
		    std::cout << "[\033[92mNO ITER\033[0m] : \033[32mInf\033[0m" << std::endl;
		}
		logFile("ITERATIONS:" + line);
		noArgs--;		
            } else if (line.find(inputArgs[3]) == 0) {
		line.replace(0,inputArgs[3].length()+1, "");        // remove name and whitespace
                if ((line.find("BEARING") == 0) || (line.find("SEEN") == 0) 
				|| (line.find("ALTITUDE") == 0) || (line.find("SCORE") == 0)) {
		    std::cout << "[\033[92mSORT type : OK\033[0m] : " << "\033[32m" << line << "\033[0m" << std::endl;
		    sortOption.assign(line);
		} else {
		    std::cout << "[\033[91mSORT type : NOK\033[0m] : " << "\033[31m" << line << "\033[0m" << std::endl;
		    sortOption.assign("SCORE");
		}
		logFile("SORT: " + line);
		noArgs--;
	    } else if (line.find(inputArgs[4]) == 0) {
		line.replace(0,inputArgs[4].length()+1, "");        // remove name and whitespace
		saveTracks = std::stoi(line);
		logFile("SAVE_TRACKS: " + line);
		noArgs--;
            
	    } else if (line.find(inputArgs[5]) == 0) {
		line.replace(0,inputArgs[5].length()+1, "");        // remove name and whitespace
                std::cout << "[\033[92mAIRPORT LON : OK\033[0m] : " << "\033[32m" << line << "\033[0m" << std::endl;
		logFile("AIRPORT_LON: " + line);
		refLON = std::stof(line);
		noArgs--;
	    
	    } else if (line.find(inputArgs[6]) == 0) {
                line.replace(0,inputArgs[6].length()+1, "");        // remove name and whitespace
                std::cout << "[\033[92mAIRPORT LAT : OK\033[0m] : " << "\033[32m" << line << "\033[0m" << std::endl;
                refLAT = std::stof(line);
		logFile("AIRPORT_LAT: " + line);
		noArgs--;
	    } else if (line.find(inputArgs[7]) == 0) {
		line.replace(0,inputArgs[7].length()+1, "");        // remove name and whitespace
		if (check_dataDir(line) == 0) {
		    std::cout << "[\033[92mDATA DIRECTORY : OK\033[0m] : " << "\033[32m" << line << "\033[0m" << std::endl;
		    dataDir.assign(line);
		} else {
		    std::cout << "[\033[91mDATA DIRECTORY : NOK\033[0m] : " << "\033[32m" << line << "\033[0m" << std::endl;
		}
		logFile("DATA_DIR: " + line);
		noArgs--;
	    } else {
		std::cout << "line read but corresponding parameter not determined." << std::endl;
	    }
        }
        file.close();
    } else {
        std::cout << "Confiuration file not found. This input appears to be incorrect --> "
                  << fileName << std::endl;
	status = 1;

    }
    status = status || noArgs;
    return status;
}

// check if specified IP address is broadcasting data
int CD1090::cx_ip_addr(void) {
    int status = 0;
    CURL *curl;
    CURLcode RES;
    READBuffer.clear();                               // READBuffer needs to be cleared on each call
    curl = curl_easy_init();
    std::cout << "\033[37mChecking IP address ...\033[0m" << std::endl;
    if(curl){
        // read dump1090 output string
        curl_easy_setopt(curl, CURLOPT_URL, IPaddress.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &READBuffer);
        RES = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
    }
    if (READBuffer.size() < 1) {
	std::cout << "[\033[91mIP address : NOK\033[0m] : " << "\033[31m" << IPaddress << "\033[0m" << std::endl;
	status = 1;
    } else {
        std::cout << "[\033[92mIP address : OK\033[0m] : " << "\033[32m" << IPaddress << "\033[0m" << std::endl;
    }
    return status;
}

// check if specified file exists
int CD1090::cxFile_exists(std::string fileName) {
    int status = 0;
    std::ifstream file(fileName);
    if(!file) {
        std::cout << "[\033[91mFile : NOK \033[0m] : " << "\033[31m" << fileName << "\033[0m" << std::endl;
	status = 1;
    } else {
        std::cout << "[\033[92mFile : OK \033[0m] : " << "\033[32m" << fileName << "\033[0m" << std::endl;
    }
    return status;
}

// create a log file in the same directory as the executable
void CD1090::logFile(std::string msg) {
    std::time_t t = std::time(0);            // current time
    std::string dt = ctime(&t);
    //std::string fileName = logFile;
    boost::filesystem::path p{logName};
    boost::system::error_code ec;
    boost::uintmax_t filesize = file_size(p, ec);
    if (filesize > MAX_LOGFILE_SIZE) {boost::filesystem::remove(p);}   // if the logfile gets too large, delete it
    fileLog.open(logName, std::ofstream::out | std::ofstream::app);
    fileLog << msg << " " << dt << std::endl;
    fileLog.close();
}

void CD1090::writeTrack(CD1090::ADSB_brcast acTrack) {
    int noPts = acTrack.latTrk.size();
    // only save track if more than 100 data points have been recorded
    if (noPts > 100) {
        // check if directory for current date already exists
	// get current date string for daily data
        std::time_t time = std::time(nullptr);
	tm *ltm = localtime(&time);               // used to check current time & clear toff & lndg vectors at the end of the day

        char dateStr[10];
	char timeStr[10];
        std::string dirName;
        if (std::strftime(dateStr, sizeof(dateStr), "%d%b%Y", std::localtime(&time))) {
            dirName.assign("data_");
            dirName.append(dateStr);
        }
	std::strftime(timeStr, sizeof(timeStr), "%Hh%Mm", std::localtime(&time));
	boost::filesystem::current_path(baseDir);
	boost::filesystem::current_path(dataDir);
        check_dataDir(dirName);   // if directory exists do nothing, otherwise create new
	boost::filesystem::current_path(dirName);

	// clear toff & lndg vectors at the end of the day
	if ((ltm->tm_hour == 23) && (ltm->tm_min == 59) && (ltm->tm_sec > 55)) {
		TOFF_vector.erase(TOFF_vector.begin(), TOFF_vector.end());          // take off trajectory for terminated tracks
		LNDG_vector.erase(LNDG_vector.begin(), LNDG_vector.end());          // landing trajectory for terminated tracks

	}

	// Duesseldorf airport runway bearings are 053° and 233°, +/- 5° tolerance added
	bool lndg_brng = (((acTrack.bearing > 228) && (acTrack.bearing < 238)) || ((acTrack.bearing > 48) && (acTrack.bearing < 58)));
        bool lndg_alt  = acTrack.altitude < 875;
        bool toff_brng = (((acTrack.brgTrk[2] > 228) && (acTrack.brgTrk[2] < 238)) || ((acTrack.brgTrk[2] > 48) && (acTrack.brgTrk[2] < 58)));
	bool toff_alt  = acTrack.altTrk[0] < 2000;
        bool toff_dist = calc_gps_dist(refLON, refLAT, acTrack.lonTrk[1], acTrack.latTrk[1]) < 8 ;	
        
	std::string mnvr;
        flight_path tempFpath;

	// assign flight path data to tempFpath
	tempFpath.hexcode = acTrack.hexcode;
        tempFpath.latTrk = acTrack.latTrk;
        tempFpath.lonTrk = acTrack.lonTrk;
        tempFpath.altTrk = acTrack.altTrk;
        tempFpath.brgTrk = acTrack.brgTrk;

	if (lndg_brng && lndg_alt) {
		mnvr.assign("LNDG");
		LNDG_vector.push_back(tempFpath);        // this data is broadcast for plotting in GUI
	}
        else if (toff_brng && toff_alt && toff_dist) {
		mnvr.assign("TOFF");
		TOFF_vector.push_back(tempFpath);        // this data is broadcast for plotting in GUI

	}
        else {mnvr.assign("FOVR");}
        
	std::string fileName =  "TRACK_" + acTrack.hexcode + "_" + mnvr + "_" + timeStr  + ".txt";
        std::ofstream file;
        file.open(fileName, std::ofstream::out | std::ofstream::app);
        for(std::vector<float>::size_type i = 0; i !=acTrack.latTrk.size(); i++) {
	    file << acTrack.lonTrk[i] << "," << acTrack.latTrk[i] << "," << acTrack.altTrk[i] << std::endl;
        }
	file.close();
	boost::filesystem::current_path(baseDir);
    }

}

bool CD1090::check_dataDir(std::string dirVar) {
    boost::filesystem::path p (dirVar);                // assign data directory to path variable
    try {
        if (!boost::filesystem::is_directory(p)) {
	    boost::filesystem::create_directory(p);            
        }
	return 0;
    }

    catch (const boost::filesystem::filesystem_error& ex) {
        logFile(ex.what());
	return 1;
    }
}

// trim from start (in place)
inline void CD1090::ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
                                return !std::isspace(ch);
                                }));
}

// trim from end (in place)
inline void CD1090::rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
                                return !std::isspace(ch);
                                }).base(), s.end());
}

// trim from both ends (in place)
inline void CD1090::trim(std::string &s) {
        ltrim(s);
        rtrim(s);
}

