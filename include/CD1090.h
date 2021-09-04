#ifndef CD1090_H
#define CD1090_H

#include <TIMER.h>
#include <nlohmann/json.hpp>

//#include <fstream>
using json = nlohmann::json;

class CD1090 {
    private:
	std::string IPaddress;                               // IP address of the JSON broadcast
	std::string ICAO_csv;                                // location and filename of the ICAO csv file
	Timer frameTime;                                     // time used to calculate fps
	Timer clearTime;                                     // time used to refresh screen
	Timer totalTime;                                     // total run time
	std::ofstream fileLog;                               // object for error file
        bool dataReady=false;                                // check if data has been received from server
	bool saveTracks=false;                               // configuration file variable for saving tracks on termination
	std::string READBuffer;                              // read buffer for JSON stream
	std::string sortOption;                              // shall be set to either "TRACK", "SEEN" or "ALTITUDE"
	std::string msgLog;                                  // message buffer for log file
        std::string baseDir;                                 // base working directory
	std::string dataDir;                                 // data dir for saving daily data recordings
	std::string logName = "status_1090.log";             // logfile to record status and errors
	float refLAT, refLON;                                // latitude, longitude for reference airport
	void sort_by_bearing(void);                          // sort vector by track number
	void sort_by_seen(void);                             // sort vector by last seen
	void sort_by_altitude(void);                         // sort vector by altitude
        void sort_by_score(void);                            // sort vector by score
	int cx_ip_addr(void);                                // check if the ip address sends data
	int cxFile_exists(std::string fileName);             // check if the csv file exists
	void logFile(std::string err_msg);                   // write status to error log
	bool check_dataDir(std::string dirVar);              // check if data directory exists
	//void writeTrack(ADSB_brcast acTrack);                // write saved track before termination
	std::vector<std::string> findByHEX(std::string hexsearch);   // method to search icao db by hex code
        std::string formatACmodel(std::string actype);       // method to format aircraft model string
	tm *sysRef_clk;                                      // system reference time
	inline void ltrim(std::string &s);            // trim string from start (in place)
	inline void rtrim(std::string &s);            // trim string from end (in place)
	inline void trim(std::string &s);             // trim string from both ends (in place)

    public:
	// struct used to store adsb broadcast
        struct ADSB_brcast {
            int altitude;         // aircraft altitude
            std::string flight;   // flight number
            std::string hexcode; // ICAO address
            float lat;           // aircraft latitude
            float lon;           // aircraft longitude
            long messages;        // number of mode S messages received
            time_t seen;          // time at which the last packet was received
            int speed;            // speed computed from EW and NS components
            std::string squawk;
            int bearing;               // bearing is listed as 'track' in json data
            uint validposition;
            uint validtrack;
            int vert_rate;
	    std::string manuf = "";    // aircraft manufacturer
	    std::string airline = "";  // airline
	    std::string typeAC = "";   // aircraft type
	    std::vector<float> latTrk; // latitude track
	    std::vector<float> lonTrk; // longitude track
	    std::vector<float> altTrk; // altitude track
	    std::vector<int>   brgTrk; // bearing track
	    float frameCtr = 0.0;      // frame counter
	    float score = 5.0;         // default score when track is initialized
        };

	// used to save the flight path for broadcast
	struct flight_path {
	    std::string hexcode;       // icao address   
	    std::vector<float> latTrk; // latitude track
            std::vector<float> lonTrk; // longitude track
            std::vector<float> altTrk; // altitude track
            std::vector<int>   brgTrk; // bearing track
	};


	//void writeTrack(ADSB_brcast acTrack);
	// struct used to store icao database
	struct icao {
            std::string hexcode;
            std::string tailNum;
            std::string manuf;
            std::string ac_type;
            std::string airline;
        };

	std::vector<ADSB_brcast> flight_vector;      // adsb broadcast vector
	std::vector<icao> icao_vector;               // icao database vector
	std::vector<ADSB_brcast> track_vector;       // tracked objects are pushed into this vector
	std::vector<flight_path> TOFF_vector;        // vector to save take-off data on terminated tracks
	std::vector<flight_path> LNDG_vector;        // vector to save landing data on terminated tracks

	void acquireJSONblock(void);
	void JSONblock2Vector(void);
	void print2screen(void);                     // prints tracked aircraft data to the terminal
	void broadcastData(void);                    // UDP broadcast of tracked data
	void broadcastTOL(void);                     // UDP broadcast of TOL data
        void buildICAO(void);
        int parseConfigData(std::string fileName);   // read config file and set required parameters
        void runTracker(void);                       // tracker method

	std::vector<std::vector<std::string>> icao_dataList;
        int noIter;                                  // number of iterations to run
	int totalIter;                               // total iterations for reference
	bool runLoop=true;                          // condition used in main to determine loop execution 
	CD1090(std::string configFile);              // constructor declaration
	~CD1090();                                   // destructor declaration

    private:
	void writeTrack(ADSB_brcast acTrack);
	float calc_gps_bearing(ADSB_brcast acTrack, int idx);
	float calc_gps_dist(float lon1, float lat1, float lon2, float lat2);
	float deg2rad(float degVal);
};
#endif
