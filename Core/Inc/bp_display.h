/*
 * bp_menu.h
 *
 *  Created on: 08.01.2021
 *      Author: kai
 */

#ifndef INC_BP_DISPLAY_H_
#define INC_BP_DISPLAY_H_

#include <bp_messages.h>
#include <ringbuffer.h>

#include <bp_communication.h> // TODO unsauber, muss noch hierarchisch vernünftig...

#define BP_DISABLE 0 // TODO anders oder anders benennen
#define BP_DISPLAY_TIMEOUT_DEFAULT 4000

typedef enum {
	MENU_STATE_ERROR = 0,
	MENU_STATE_OK
}bp_menu_state_dt;

typedef enum {
	MENU_CONTROL_NONE = 0,
	MENU_CONTROL_UP,
	MENU_CONTROL_DOWN,
	MENU_CONTROL_RIGHT,
	MENU_CONTROL_LEFT
	// TODO: Zahlen können hier dann auch hin...
}bp_menu_control_en;

typedef enum {
	MENU_NONE = 0, // TODO gefählrich... muss noch weg?

	MENU_DSC,
	BP_MENU_SIZE,
}bp_menu_en;

// Menüeintrag allgemein
typedef struct{
	char* name;
	char* (*getValue)(char * name); // forward the ptr of the function name
	void (*controlEntry)(bp_menu_control_en control);
	// TODO: Update und control-Funktion einzeln?
}menu_entry_dt;

// ein ganzes Menü
typedef struct{
	menu_entry_dt *menuArr;
	uint8_t menuSize;
}menu_dt;

/*enum menu_dsc_en {
	DSC_VERSION = 0,
	DSC_TEST,

	DSC_MENU_SIZE
};*/

/*enum menu_active_value {
	MENU_SHOW_NAME = 0, // we show the current menu name
	MENU_SHOW_VALUE
};*/

/*typedef struct {
	char* name;
	struct menu_entry entries[];
}menu_dt;*/

typedef struct {
	char currentText[50]; // TODO Größe festlegen

	uint8_t remainingChars; /* How many chars are left to send in further messages ? */

	uint32_t timeoutWaitMs;   /* Default time base to return back to generic value   */
	uint32_t timeoutTimestamp;

	uint32_t displayUpdateTime; // maximum speed to update the display (TODO: muss auch noch getestet werden, damit die nachrichten nicht überfluten)
	uint32_t displayUpdateTimestamp;

	uint32_t menuLeaveWaitMs;       /* The time we wait until we leave the menu after no button pushed */
	uint32_t menuLeaveTimestamp;
	uint32_t menuSwitchValueWaitMs; /* The time we wait to switch showing between menu point and value */
	uint32_t menuSwitchValueTimestamp;

	bp_menu_en currentMenu;
	uint8_t currentMenuIndex; // in welchem Menüpunktindex..
	//menutimeout
	//const menu_dt *menuDsc; // TODO: muss noch zugeordnet werden...
	//enum menu_dsc_en currentDscEntry;
	//enum menu_active_value menuActiveValue;

}bp_display_state_dt;

void bpDisplayInit(void);
void bpDisplayTasks(void);

bp_msg_dt buildTextMessage(char * text, uint32_t displayWaitMs);

bp_menu_state_dt setBpMenu(bp_menu_en menu);
bp_menu_state_dt controlBpMenu(bp_menu_control_en control);

#endif /* INC_BP_DISPLAY_H_ */
