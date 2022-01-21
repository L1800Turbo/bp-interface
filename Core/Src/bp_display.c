/*
 * bp_menu.c
 *
 *  Created on: 07.01.2021
 *      Author: kai
 */
#include <bp_display.h>

extern bp_msg_state_dt bpMsgState;
bp_display_state_dt bpDisplayState;

extern const bp_msg_dt bpMessages[]; // TODO noch auslagern... eigene Void, de das zurück gibt

/*
 * - Jedes Menü bekommt ein menu_dt
 * - Es gibt eine anzeigen und eine ändern-void:
 *   - Anzeigen: Menüname und Wert passend anzeigen, manchmal togglen, manchmal was anderes
 *     -> allgemeine Funktionen dafür definieren
 *     - als Parameter: Ptr. vom Namen des Menüpunktes geben? Dann kann die Funktion selbst entscheiden, was sie damit tut
 *   - Ändern: Anpassung durchführen, wenn der Menüpunkt bearbeitet wird, also Pfeil zu den seiten
 *   - allgemein festhalten:
 *     - in welchem Menü man sich befindet
 *     - welchen Punkt man aktuell hat
 * 	 - zurück-Taste? DSC sollte ja alles schließen
 *
 * 	 Offen:
 * 	 - wie kann man die Zeiten allgemein festhalten? Es soll ja eine Funktion den Text zurückgeben und das so, wie es für die Funktion sinnvoll ist...
 *
 */

char* dscVersionShow(char * name)
{
	return __DATE__;//"Hallo";
}

void dscVersionControl(bp_menu_control_en control)
{

}

uint8_t dscDummyVal = 0;

char* dscDummyShow(char * name)
{
	strcpy(bpDisplayState.currentText, "        ");
	uint8_t position = 6; // Stelle, wo die Zahl beginnt

	// erste Zeichen kopieren
	for(uint8_t i=0; i<5; i++)
	{
		bpDisplayState.currentText[i] = name[i];
	}

	if(dscDummyVal < 10)
	{
		position++;
	}
	itoa(dscDummyVal, &bpDisplayState.currentText[position], 10);

	return bpDisplayState.currentText;
}

void dscDummyControl(bp_menu_control_en control)
{
	if(control == MENU_CONTROL_RIGHT)
	{
		dscDummyVal++;
	}
	else if(control == MENU_CONTROL_LEFT)
	{
		dscDummyVal--;
	}
}

static const menu_entry_dt menuDsc[] = {
		//{0},
		{.name = "Version", .getValue = dscVersionShow, .controlEntry = dscVersionControl},
		{.name = "Dummy",   .getValue = dscDummyShow,   .controlEntry = dscDummyControl}
};

static const menu_dt menus[] = { // vom enum bp_menu_en
		{0,0},
		{.menuArr = menuDsc, .menuSize = 2},
	//	{.menuArr = menuVol, .menuSize = 1}
};

/*static const menu_entry_dt *menus[] = { // vom enum bp_menu_en
		0,
		menuDsc
};*/

void bpDisplayInit(void)
{
	/* Initialize display defaults */
	bpDisplayState.timeoutWaitMs = BP_DISABLE;
	bpDisplayState.displayUpdateTime = 400;

	//bpDisplayState.menuSwitchValueWaitMs = 900;

	//bpDisplayState.menuDsc = &menuDsc; // TODO: in Currentmenue öndern? Aber andere Menüs sollten ja niht mit DSC beendet werden können?

	bpDisplayState.currentMenu = MENU_NONE;
}

void bpDisplayTasks(void)
{
	/* Display message tasks for running texts */
	if(bpDisplayState.remainingChars > BP_DISABLE && (HAL_GetTick()-bpDisplayState.displayUpdateTimestamp) > bpDisplayState.displayUpdateTime)
	{
		bp_msg_dt msg;

		// Create Message with offset, which will shorten the message; add to ring
		msg = buildTextMessage(bpDisplayState.currentText+2, bpDisplayState.timeoutWaitMs);
		ringAdd(bpMsgState.writeBuf, msg);
		setSendWait();
	}

	/*if(bpDisplayState.timeoutWaitMs != BP_DISABLE)
	{
		printf("wait > 0\r\n");
	}*/

	/* If we want to go back to default value after some time */
	if((bpDisplayState.timeoutWaitMs != BP_DISABLE) && ((HAL_GetTick()-bpDisplayState.timeoutTimestamp) > bpDisplayState.timeoutWaitMs))
	{
		char * currentValue;

		bpDisplayState.timeoutWaitMs = BP_DISABLE;

		if(bpDisplayState.currentMenu == MENU_NONE)
		{
			currentValue = stateFlags.currentDisplayMessage;
		}
		else // Go back to current menu value if in menu
		{
			currentValue = menus[bpDisplayState.currentMenu].menuArr[bpDisplayState.currentMenuIndex]
											.getValue(menus[bpDisplayState.currentMenu].menuArr[bpDisplayState.currentMenuIndex].name);
		}
		ringAdd(bpMsgState.writeBuf, buildTextMessage(currentValue, BP_DISABLE));
		setSendWait();
	}
}

bp_msg_dt buildTextMessage(char * text, uint32_t displayWaitMs)
// TODO: zunächst nur einfache Texte, die den Bildschirm 1x füllen, warteZeit einfügen, die 0 ist, wenn Standard-Timeout verwendet werden soll
{
	uint8_t textLen = 0;
	char outbuf[35];

	utf2bp(text, strlen(text), outbuf, sizeof(outbuf));

	textLen = strlen(outbuf);

	if(textLen > 8)
	{
		// TODO ausrichten (zentrieren, links, ..) und so...

		strcpy(bpDisplayState.currentText, outbuf);
		bpDisplayState.remainingChars = textLen - 8;
		bpDisplayState.displayUpdateTimestamp = HAL_GetTick();

		textLen = 8;

		// Start with the first 8 bytes after this init...
	}
	else
	{
		bpDisplayState.remainingChars = BP_DISABLE; // to stop sending
	}

	/* Timestamp to go back to default view, if wanted */
	if(displayWaitMs != BP_DISABLE)
	{
		bpDisplayState.timeoutWaitMs = displayWaitMs;
		bpDisplayState.timeoutTimestamp = HAL_GetTick();
	}
	else
	{
		bpDisplayState.timeoutWaitMs = BP_DISABLE;
	}

	return buildMessage(bpMessages[BP_MSG_TEXT].address, textLen, bpMessages[BP_MSG_TEXT].command, outbuf, 10);
}

/* Change menu if possible */
bp_menu_state_dt setBpMenu(bp_menu_en menu)
{
	if(menu == MENU_NONE) // TODO: Leave silent...
	{
		bpDisplayState.currentMenu = MENU_NONE;
		//ringAdd(bpMsgState.writeBuf, buildTextMessage("bla", 200));
		//setSendWait();
	}
	/* Leave menu if current menu is already active */
	else if(bpDisplayState.currentMenu == menu) // TODO: wenn z.B. ein zweites Mal was mt VOL kommt, oder FAD... kommt er dann hier hin?!
	{
		printf("\033[1;36mMenü aus\033[0m\r\n");

		bpDisplayState.currentMenu = MENU_NONE;

		ringAdd(bpMsgState.writeBuf, buildTextMessage("Exit", 200));
		setSendWait();
	}
	else if(menu < BP_MENU_SIZE)
	{
		char * currentValue;
		menu_entry_dt * cMenu = menus[menu].menuArr; // TODO: gefährlich, wenn er eine 0 holt und an Adresse 0 springt!


		printf("\033[1;36mMenü auf: %d (%X)\033[0m\r\n", menu, (unsigned int) &menu);

		bpDisplayState.currentMenu = menu;
		bpDisplayState.currentMenuIndex = 0;

		printf("Lüm: %s\r\n", cMenu[bpDisplayState.currentMenuIndex].name);

		currentValue = cMenu[bpDisplayState.currentMenuIndex].getValue(cMenu[bpDisplayState.currentMenuIndex].name);
		ringAdd(bpMsgState.writeBuf, buildTextMessage(currentValue, BP_DISABLE));
		setSendWait();
	}

	// TODO: unbekannte Menüs ignorieren

	return MENU_STATE_OK;
}

/* Control current menu -> TODO: Komplett weg und nur über den Pointer regeln, abfragen, dass der Ptr. nie leer ist */
bp_menu_state_dt controlBpMenu(bp_menu_control_en control)
{
	char * currentValue;

	if(bpDisplayState.currentMenu != MENU_NONE)
	{
		menu_entry_dt * cMenu = menus[bpDisplayState.currentMenu].menuArr;
		uint8_t cMenuSize = menus[bpDisplayState.currentMenu].menuSize;

		if(control == MENU_CONTROL_UP)
		{
			bpDisplayState.currentMenuIndex = (bpDisplayState.currentMenuIndex-1) % cMenuSize;
			printf("Index Menü: %d\r\n", bpDisplayState.currentMenuIndex);
		}
		else if(control == MENU_CONTROL_DOWN)
		{
			bpDisplayState.currentMenuIndex = (bpDisplayState.currentMenuIndex+1) % cMenuSize;
			printf("Index Menü: %d\r\n", bpDisplayState.currentMenuIndex);
		}
		else
		{
			cMenu[bpDisplayState.currentMenuIndex].controlEntry(control);

		}

		currentValue = cMenu[bpDisplayState.currentMenuIndex].getValue(cMenu[bpDisplayState.currentMenuIndex].name);
		ringAdd(bpMsgState.writeBuf, buildTextMessage(currentValue, BP_DISABLE));
		setSendWait();
	}

	return MENU_STATE_OK;

	// ab hier tonne erstmal....
	/*switch(bpDisplayState.currentMenu)
	{
		case MENU_DSC: // TODO: evtl noch andere Menüs, bei denen man auch so blättert? -> Pointer rein bauen
			// TODO: z.B. Menüpunkt hoch und runter und so
			// array muss da noch defnizert werden

			// wenn hoch oder runter, den Eintrag wechseln
			if(control == MENU_CONTROL_UP)
			{
				bpDisplayState.currentDscEntry = (bpDisplayState.currentDscEntry+1) % (DSC_MENU_SIZE);
			}
			else if(control == MENU_CONTROL_DOWN)
			{
				bpDisplayState.currentDscEntry = (bpDisplayState.currentDscEntry-1) % (DSC_MENU_SIZE); // TODO muss noch gemacht werden!
			}

			// TODO: dieser text muss dann ja auch angegeben weren...
			bpDisplayState.menuDsc->entries[bpDisplayState.currentDscEntry].controlEntry(control); // TODO ptr menuDsc muss noch zugeordnet werden, sonst stürzt der sicher ab

			break;

		default:
			break;
	}*/
}

