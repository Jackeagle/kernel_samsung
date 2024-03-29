
#ifndef _TSP_TA_CALLBACK_H_
#define _TSP_TA_CALLBACK_H_

#ifdef CONFIG_EXTCON
#undef USE_TSP_TA_CALLBACKS
#else

/* Model define */
#if defined(CONFIG_SEC_O1_PROJECT)
#define USE_TSP_TA_CALLBACKS

#elif defined(CONFIG_SEC_FORTUNA_PROJECT)
#define USE_TSP_TA_CALLBACKS

#elif defined(CONFIG_SEC_A3_PROJECT) || defined(CONFIG_SEC_A3_EUR_PROJECT) \
	|| defined(CONFIG_SEC_A33G_EUR_PROJECT)
#define USE_TSP_TA_CALLBACKS

#elif defined(CONFIG_SEC_KLEOS_PROJECT)
#define USE_TSP_TA_CALLBACKS

#elif defined(CONFIG_SEC_HEAT_PROJECT)
#define USE_TSP_TA_CALLBACKS

#elif defined(CONFIG_SEC_ROSSA_PROJECT)
#define USE_TSP_TA_CALLBACKS

#elif defined(CONFIG_SEC_VIVALTO_PROJECT)
#define USE_TSP_TA_CALLBACKS

#else	/* default */
#undef USE_TSP_TA_CALLBACKS
#endif

#endif	/* CONFIG_EXTCON */

#ifdef USE_TSP_TA_CALLBACKS
extern struct tsp_callbacks *charger_callbacks;
struct tsp_callbacks {
	void (*inform_charger)(struct tsp_callbacks *, int);
};
#endif

#endif /* _TSP_TA_CALLBACK_H_ */

