
#include <platform/mt_typedefs.h>


typedef struct{
    INT32 BatteryTemp;
    INT32 TemperatureR;
}BATT_TEMPERATURE;

#define RBAT_PULL_UP_R 16900
#define RBAT_PULL_DOWN_R 27000
#define RBAT_PULL_UP_VOLT 1800

#define NTC_ID_CHECK_SUPPORT 1

#ifdef NTC_ID_CHECK_SUPPORT
// for Yarisxxl BYD Battery
	BATT_TEMPERATURE Batt_Temperature_Table_1[] = {
        {-20,68237},
        {-15,53650},
        {-10,42506},
        { -5,33892},
        {  0,27219},
        {  5,22021},
        { 10,17926},
        { 15,14674},
        { 20,12081},
        { 25,10000},
        { 30,8315},
        { 35,6948},
        { 40,5834},
        { 45,4917},
        { 50,4161},
        { 55,3535},
        { 60,3014}
    };
//for Yarisxxl SCUD
	BATT_TEMPERATURE Batt_Temperature_Table_2[] = {
         {-20,67790},    
         {-15,53460},
         {-10,42450},
         { -5,33930},
         {  0,27280},
         {  5,22070},
         { 10,17960},
         { 15,14700},
         { 20,12090},
         { 25,10000},
         { 30,8312},
         { 35,6942},
         { 40,5826},
         { 45,4911},
         { 50,4158},
         { 55,3536},
         { 60,3019}
         };
//for Yarisxxl LISHEN
	BATT_TEMPERATURE Batt_Temperature_Table_3[] = {
	 {-20,63000},	 
	 {-15,50150},
	 {-10,40260},
	 { -5,32550},
	 {	0,26490},
	 {	5,21680},
	 { 10,17780},
	 { 15,14630},
	 { 20,12070},
	 { 25,10000},
	 { 30,8320},
	 { 35,6954},
	 { 40,5839},
	 { 45,4924},
	 { 50,4171},
	 { 55,3549},
	 { 60,3032}
	 };
#endif

