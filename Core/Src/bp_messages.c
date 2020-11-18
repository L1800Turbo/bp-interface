#include "bp_messages.h"

/* Convert message into BP compatible coding */
void utf2bp(const char * inBuf, uint8_t inBufLen, char * outBuf, uint8_t outBufLen)
{
	uint32_t cUtf = 0;
	char * outPtr = outBuf;

	for(uint8_t i=0; i<inBufLen; i++)
	{
		if(inBuf[i] == 0)
		{
			break;
		}

		if(inBuf[i] <= 0x7A || inBuf[i] == 0x7C) // cannot print every char..
		{
			*outPtr = inBuf[i];
		}
		else
		{
			if((inBuf[i] & 0xE0) == 0xC0 && (i+1)<inBufLen) /* 2 byte char */
			{
				cUtf = (inBuf[i] << 8) + inBuf[i+1];

				/* Just go through BP characters as they are not many */
				switch(cUtf)
				{
					case 0xC2A1:
						*outPtr = 0x1A;	/* ¡ */
						break;

					case 0xC2A7:
						*outPtr = 0x1B;	/* § */
						break;

					case 0xC2Bf:
						*outPtr = 0x7F;	/* ¿ */
						break;

					case 0xC384:
						*outPtr = 0x08;	/* Ä */
						break;

					case 0xC386:
						*outPtr = 0x0A;	/* Æ */
						break;

					case 0xC38B:
						*outPtr = 0x09;	/* Ë */
						break;

					case 0xC391:
						*outPtr = 0x0B;	/* Ñ */
						break;

					case 0xC396:
						*outPtr = 0x0C;	/* Ö */
						break;

					case 0xC398:
						*outPtr = 0x0D;	/* Ø */
						break;

					case 0xC39C:
						*outPtr = 0x0F;	/* Ü */
						break;

					case 0xC39F:
						*outPtr = 0x7B;	/* ß */
						break;

					case 0xC3A4:
						*outPtr = 0x11;	/* ä */
						break;

					case 0xC3A6:
						*outPtr = 0x13;	/* æ */
						break;

					case 0xC3AB:
						*outPtr = 0x12;	/* ë */
						break;

					case 0xC3B1:
						*outPtr = 0x14;	/* ñ */
						break;

					case 0xC3B6:
						*outPtr = 0x15;	/* ö */
						break;

					case 0xC3B8:
						*outPtr = 0x16;	/* ø */
						break;

					case 0xC3BC:
						*outPtr = 0x18;	/* ü */
						break;

					case 0xC592:
						*outPtr = 0x0E;	/* Œ */
						break;

					case 0xC593:
						*outPtr = 0x17;	/* œ */
						break;

					case 0xCB84:
						*outPtr = 0x7E;	/* ˄ */
						break;

					case 0xCB85:
						*outPtr = 0x7D;	/* ˅ */
						break;

					default:
						*outPtr = '?';
						break;

						//TODO:     usw... sind noch nicht alle aus der Liste drin...

				}

                i++; // jump over next byte
			}
			else if((inBuf[i] & 0xF0) == 0xE0 && (i+2)<inBufLen) /* 3 byte char */
            {
				cUtf = (inBuf[i] << 16) + (inBuf[i+1] << 8) + inBuf[i+2];

				switch(cUtf) // TODO ungetestet -> funktioniert nicht, wenn ein Mehrbyteding am Ende Steht!!!!!
				// Ideen: Klappt das mit dem Ptr am Ende? / Ptr sollte gehen.. Das "?" geht ja auch / Ist der auch als 2byte Char problembehaftet?
				// wie sieht der Outptr in Zeile 587 aus, wenn man am Ende so einen Mehrbyte hat und wenn nicht?
				// vllt. WaitMS?? -> Tabelle
				{
					case 0xE28690:
						*outPtr = 0x1E;	/* ← */
						break;

					case 0xE28691:
						*outPtr = 0x1D;	/* ↑ */
						break;

					case 0xE28692:
						*outPtr = 0x1C;	/* → */
						break;

					case 0xE28693:
						*outPtr = 0x1F;	/* ↓ */
						break;

					case 0xE2AE9D: // NOTE: Doppelt von oben
						*outPtr = 0x7E;	/* ⮝ */
						break;

					case 0xE2AE9F: // NOTE: Doppelt von oben
						*outPtr = 0x7D;	/* ⮟ */
						break;

					default:
						*outPtr = '?';
						break;
				}

				//*outPtr = '?';

                i=i+2; // jump 2 bytes
            }
            else if((inBuf[i] & 0xF8) == 0xF0) /* 4 byte char */
            {
                *outPtr = '?';

                i+=3;
            }
			else
			{
				// No multibyte and not known...
				*outPtr = '?';
			}
		}

		outPtr++;
        if(--outBufLen == 0)
        {
            //*outPtr = 0;
            //return;
            break;
        }
	}
	*outPtr = 0; // Terminate C string
}
