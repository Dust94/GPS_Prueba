
#ifndef GPS_H_
#define GPS_H_

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include "uart.h"

/***************DEFINICIONES****************/
#define MAX_LENGTH	512
#define POSLLH_MSG  0x02
#define SOL_MSG     0x06
#define SBAS_MSG    0x32
#define VELNED_MSG  0x12
#define STATUS_MSG  0x03
#define DOP_MSG     0x04
#define DGPS_MSG    0x31
#define ToPC		0
#define ToGPS		2


/***************PUNTEROS****************/
#define LONG(X)    *(long*)(&data[X])
#define ULONG(X)   *(unsigned long*)(&data[X])
#define INT(X)     *(int*)(&data[X])
#define UINT(X)    *(unsigned int*)(&data[X])

unsigned char  state, lstate, code, id, chk1, chk2, ck1, ck2;
unsigned int  length, idx, cnt, len_cmdBuf;
unsigned char data[MAX_LENGTH];
long lastTime = 0;
int checkOk = 0;
unsigned char cc, flag, state;

void sendCmd (unsigned char len, uint8_t data[]) {
	uart_print(ToGPS, 0xB5);
	uart_print(ToGPS, 0x62);
	unsigned char chk1 = 0, chk2 = 0, i;
	for ( i = 0; i < len; i++) { //A traves de todo el arreglo "data[]"
		unsigned char cc = data[i];
		uart_print(ToGPS, cc); //Envio el dato al GPS
		chk1 += cc;		// 0 5 10 15 20 25
		chk2 += chk1;	// 0 5 15 30 50 75
	}//Fin for
	uart_print(ToGPS, chk1); //Envio el dato al GPS
	uart_print(ToGPS, chk2); //Envio el dato al GPS
}

void enableMsg (unsigned char id, uint8_t enable) {
	//						MSG   NAV   < length >  NAV
	if (enable == 1){
		uint8_t cmdBuf[] = {0x06, 0x01, 0x03, 0x00, 0x01, id, 0x01};
		len_cmdBuf = sizeof(cmdBuf) / sizeof(cmdBuf[0]);
		sendCmd(len_cmdBuf, cmdBuf);
	}else{
		uint8_t cmdBuf[] = {0x06, 0x01, 0x03, 0x00, 0x01, id, 0x00};
		len_cmdBuf = sizeof(cmdBuf) / sizeof(cmdBuf[0]); //Obtengo el Tamaño del arreglo
		sendCmd(len_cmdBuf, cmdBuf);
	} // Fin Else
} // Fin enableMsg(id, enable)

// Convert 1e-7 value packed into long into decimal format
void printLatLon (long val) {
	char buffer[14];
	ltoa(val,buffer,10);
	int len = sizeof(buffer) / sizeof(buffer[0]);
	unsigned char i = 0;
	while (i < (len - 7)) { uart_print(ToPC,buffer[i++]);}
	uart_print(ToPC,".");
	while (i < len) { uart_print(ToPC,buffer[i++]);}
}
  
void Setup_GPS(void){
	uart_init(ToPC); //Arduino PC. BAUD 9600. ToPC=0
	uart_init(ToGPS); //Arduino GPS. BAUD2 38400. ToGPS=2
  
	// Modify these to control which messages are sent from module
	enableMsg(POSLLH_MSG, 1);    // Enable position messages
	enableMsg(SOL_MSG, 1);       // Enable solution
	enableMsg(SBAS_MSG, 0);      // Enable SBAS messages
	enableMsg(VELNED_MSG, 0);    // Enable velocity messages
	enableMsg(STATUS_MSG, 0);    // Enable status messages
	enableMsg(DOP_MSG, 0);       // Enable DOP messages
	enableMsg(DGPS_MSG, 0);      // Disable DGPS messages 
	uart_println(ToPC, "Fin Setup GPS y Uarth");
}

void ImpresionVariables(void){
	//Impresion de las Variables
	unsigned char tempVariable[10];
	uart_print(ToPC,"chk1: "); uart_println(ToPC,itoa(chk1,tempVariable,10));
	uart_print(ToPC,"ck1: "); uart_println(ToPC,itoa(ck1,tempVariable,10));
	uart_print(ToPC,"length: "); uart_println(ToPC,itoa(length,tempVariable,10));
	uart_print(ToPC,"chk2: "); uart_println(ToPC,itoa(chk2,tempVariable,10));
	uart_print(ToPC,"ck2: "); uart_println(ToPC,itoa(ck2,tempVariable,10));
}

void Syncronization(void){
	state = 0;
	if(uart_available(ToGPS)){
		cc = uart_read(ToGPS);
		
		if(state == 0){								// wait for sync 1 (0xB5)
			ck1 = 0; ck2 = 0;
			if(cc == 0xB5) state++;
		} uart_println(ToPC,"Finish Step 0");
		
		if (state == 1){						// wait for sync 2 (0x62)
			if(cc == 0x62) state++;
			else state=0;
		} uart_println(ToPC,"Finish Step 1");

		if (state == 2){						// wait for class code
			code = cc;
			ck1 += cc;
			ck2 += ck1;
			state++;
		} uart_println(ToPC,"Finish Step 2");
		
		if (state == 3){						// wait for Id
			id = cc;
			ck1 += cc;
			ck2 += ck1;
			state++;
		} uart_println(ToPC,"Finish Step 3");
		
		if (state == 4){						// wait for length byte 1
			length = cc;
			ck1 += cc;
			ck2 += ck1;
			state++;
		} uart_println(ToPC,"Finish Step 4");
		
		if (state == 5){						// wait for length byte 2
			length |= (unsigned int) cc << 8;
			ck1 += cc;
			ck2 += ck1;
			idx = 0;
			state++;
			if (length > MAX_LENGTH) state= 0;
		} uart_println(ToPC,"Finish Step 5");
		
		if (state == 6){						// wait for <length> payload bytes
			data[idx++] = cc;
			ck1 += cc;
			ck2 += ck1;
			if (idx >= length) state++;
		} uart_println(ToPC,"Finish Step 6");
		
		if (state == 7){						// wait for checksum 1
			chk1 = cc;
			state++;
		} uart_println(ToPC,"Finish Step 7 (Final Step)");
		
		if (state == 8){						// wait for checksum 2
			chk2 = cc;
			ImpresionVariables();
			
			if((ck1 == chk1)  &&  (ck2 == chk2)){ // Only if checkOk=1
				uart_println(ToPC, "Entre! checkOk=1 ");
				unsigned char tempCode[10];
				uart_print(ToPC,"code: "); uart_println(ToPC,itoa(code,tempCode,10));
				
				if (code == 0x01){
					uart_println(ToPC,"Ingresé a code=0x01");
					if (lastTime != ULONG(0))	lastTime = ULONG(0);
					unsigned char tempId[10];
					uart_print(ToPC,"Id: "); uart_println(ToPC,itoa(id,tempId,10));
					
					if (id == POSLLH_MSG){
						uart_print(ToPC, "X: ");	printLatLon(LONG(8));
						uart_print(ToPC, "Y: ");	printLatLon(LONG(4));
					} // Fin id=POSLLH_MSG
					
					else if (id == SOL_MSG){
						uart_println(ToPC,"Ingresé a id == SOL_MSG");
						unsigned char tempData24[10];
						uart_print(ToPC,"ULONG(24): "); uart_println(ToPC,ltoa(ULONG(24),tempData24,10));
					} // Fin id=SOL_MSG
					
					/*
						switch (id) {
							case 0x02:  // NAV-POSLLH
								printLatLon(LONG(4)); uart_print(ToPC,"\n\r");
								printLatLon(LONG(8)); uart_print(ToPC,"\n\r"); break;
							case 0x03:  // NAV-STATUS
								uart_print(ToPC, F("STATUS: gpsFix = ") );
								uart_print(ToPC, data[4]);
								if (data[5] & 2) uart_print(ToPC, F(", dgpsFix") );
								break;
							case 0x04:  // NAV-DOP
								uart_print(ToPC, F("DOP:    gDOP = "));
								printBinary(UINT(4) / 100);
								uart_print(ToPC, F(", tDOP = "));
								printBinary(UINT(8) / 100);
								uart_print(ToPC, F(", vDOP = "));
								printBinary(UINT(10) / 100);
								uart_print(ToPC, F(", hDOP = "));
								printBinary(UINT(12) / 100); break;
							case 0x06:  // NAV-SOL
								uart_print(ToPC, ULONG(24)); break;
							case 0x12:  // NAV-VELNED
								uart_print(ToPC, F("VELNED: gSpeed = "));
								uart_print(ToPC, ULONG(20));
								uart_print(ToPC, F(" cm/sec, sAcc = "));
								uart_print(ToPC, ULONG(28));
								uart_print(ToPC, F(" cm/sec, heading = "));
								printBinary(LONG(24) / 100000);
								uart_print(ToPC, F(" deg, cAcc = "));
								printBinary(LONG(32) / 100000);
								uart_print(ToPC, F(" deg")); break;
							case 0x31:  // NAV-DGPS
								uart_print(ToPC, F("DGPS:   age = "));
								uart_print(ToPC, LONG(4));
								uart_print(ToPC, F(", baseId = "));
								uart_print(ToPC, INT(8));
								uart_print(ToPC, F(", numCh = "));
								uart_print(ToPC, INT(12)); break;
							case 0x32:  // NAV-SBAS
								uart_print(ToPC, F("SBAS:   geo = "));
								switch (data[4]) {
									case 133:
										uart_print(ToPC, F("Inmarsat 4F3")); break;
									case 135:
										uart_print(ToPC, F("Galaxy 15")); break;
									case 138:
										uart_print(ToPC, F("Anik F1R")); break;
									default:
										uart_print(ToPC, data[4]);	break;
								} //Fin Switch data[4]
								uart_print(ToPC, F(", mode = "));
								switch (data[5]) {
									case 0:
										uart_print(ToPC, F("disabled")); break;
									case 1:
										uart_print(ToPC, F("enabled integrity")); break;
									case 2:
										uart_print(ToPC, F("enabled test mode")); break;
									default:
										uart_print(ToPC, data[5]);
								} //Fin Switch data[5]
								uart_print(ToPC, F(", sys = "));
								switch (data[6]) {
									case 0:
										uart_print(ToPC, F("WAAS")); break;
									case 1:
										uart_print(ToPC, F("EGNOS")); break;
									case 2:
										uart_print(ToPC, F("MSAS")); break;
									case 16:
										uart_print(ToPC, F("GPS")); break;
									default:
										uart_print(ToPC, data[6]);
								} //Fin Switch data[6]
								break;
							default:
								printHex(id);
						} //Fin Switch id
						uart_print(ToPC, "\n\r"); break;
					case 0x05:      // ACK-
						uart_print(ToPC, F("ACK-"));
						switch (id) {
							case 0x00:  // ACK-NAK
								uart_print(ToPC, F("NAK: ")); break;
							case 0x01:  // ACK-ACK 
								uart_print(ToPC, F("ACK: ")); break;
						} //Fin Switch id
						printHex(data[0]); uart_print(ToPC, " ");
						printHex(data[1]); uart_print(ToPC, "\n\r"); break;
						*/
					
					
				} //Fin If(code == 0x01)
			} //Fin if(checkOk)		
			state = 0;
		} // Fin if(state == 8)	Last Step
	}//Fin If uart_available()	
}// Fin Syncronization()




#endif /* GPS_H_ */