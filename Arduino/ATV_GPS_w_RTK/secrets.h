//ADD HERE THE WIFI NAME AND PASSWORD with witch you want to receive the GPS correction stream
const char ssid[] = "";
const char password[] =  "";

//RTK2Go works well and is free
const char casterHost[] = "rtk2go.com"; 
const uint16_t casterPort = 2101;
const char casterUser[] = ""; //ADD HERE YOUR EMAIL
const char casterUserPW[] = "";
const char mountPoint[] = "Taroniemi"; //The mount point you want to get data from

// To get correction data from the NLS, go to their website:  https://www.maanmittauslaitos.fi/en/finpos/rtk
// Register and send a request for the free RTK stream for research and education
