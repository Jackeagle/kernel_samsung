#define CAPACITY_MAX			960
#define CAPACITY_MAX_MARGIN     50
#define CAPACITY_MIN			0

#if defined(CONFIG_MACH_FORTUNA_CTC)
static sec_bat_adc_table_data_t temp_table[] = {
	{25950, 900},
	{26173, 850},
	{26424, 800},
	{26727, 750},
	{26714, 700},
	{27189, 650},
	{27985, 600},
	{28426, 550},
	{29185, 500},
	{29942, 450},
	{30607, 400},
	{31562, 350},
	{32517, 300},
	{33673, 250},
	{34629, 200},
	{35684, 150},
	{36739, 100},
	{37810, 50},
	{38760, 0},
	{39858, -50},
	{40461, -100},
	{41124, -150},
	{41510, -200},
};
#else
static sec_bat_adc_table_data_t temp_table[] = {
	{26056, 900},
	{26268, 850},
	{26520, 800},
	{26864, 750},
	{27214, 700},
	{27631, 650},
	{28183, 600},
	{28739, 550},
	{29389, 500},
	{30153, 450},
	{30965, 400},
	{31919, 350},
	{32930, 300},
	{33951, 250},
	{34957, 200},
	{36038, 150},
	{37194, 100},
	{38152, 50},
	{39002, 0},
	{39805, -50},
	{40421, -100},
	{41055, -150},
	{41593, -200},
};
#endif

#define TEMP_HIGH_THRESHOLD_EVENT  600
#define TEMP_HIGH_RECOVERY_EVENT   460
#define TEMP_LOW_THRESHOLD_EVENT   (-50)
#define TEMP_LOW_RECOVERY_EVENT    0
#define TEMP_HIGH_THRESHOLD_NORMAL 600
#define TEMP_HIGH_RECOVERY_NORMAL  460
#define TEMP_LOW_THRESHOLD_NORMAL  (-50)
#define TEMP_LOW_RECOVERY_NORMAL   0
#define TEMP_HIGH_THRESHOLD_LPM    600
#define TEMP_HIGH_RECOVERY_LPM     460
#define TEMP_LOW_THRESHOLD_LPM     (-50)
#define TEMP_LOW_RECOVERY_LPM      0
