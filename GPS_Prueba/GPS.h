
#ifndef GPS_H_
#define GPS_H_

#include <avr/io.h>
# define PSTR(s) ((const PROGMEM char *)(s))
#define F(string_literal) (reinterpret_cast<__FlashStringHelper *>(PSTR(string_literal)))

#define  MAX_LENGTH	 512
#define  POSLLH_MSG  0x02
#define  SBAS_MSG    0x32
#define  VELNED_MSG  0x12
#define  STATUS_MSG  0x03
#define  SOL_MSG     0x06
#define  DOP_MSG     0x04
#define  DGPS_MSG    0x31

#define  ToPC		0
#define  ToGPS		2

#define LONG(X)    *(long*)(&data[X])
#define ULONG(X)   *(unsigned long*)(&data[X])
#define INT(X)     *(int*)(&data[X])
#define UINT(X)    *(unsigned int*)(&data[X])

unsigned char  state, lstate, code, id, chk1, chk2 = 0, ck1 = 0, ck2;
unsigned int  length = 0, idx, cnt, len_cmdBuf;

unsigned char data[MAX_LENGTH];

long lastTime = 0;
int checkOk = 0;

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

void printHex (unsigned char val) {
	if (val < 0x10) uart_print(ToPC, "0");  // val<16
	uart_print(ToPC, val);
}

void printBinary (unsigned char val) {
	if( (val<0)) || (val>255)) uart_print(ToPC, "0");
	uart_print(ToPC, val%256);
	uart_print(ToPC, val%128);
	uart_print(ToPC, val%64);
	uart_print(ToPC, val%32);
	uart_print(ToPC, val%16);
	uart_print(ToPC, val%8);
	uart_print(ToPC, val%4);
	uart_print(ToPC, val%2);
}
  
void Setup_GPS(void){
	uart_init(0); //Arduino PC. BAUD 9600 
	uart_init(2); //Arduino GPS. BAUD2 38400
  
	  // Modify these to control which messages are sent from module
	enableMsg(POSLLH_MSG, 1);    // Enable position messages
	enableMsg(SBAS_MSG, 0);      // Enable SBAS messages
	enableMsg(VELNED_MSG, 0);    // Enable velocity messages
	enableMsg(STATUS_MSG, 0);    // Enable status messages
	enableMsg(SOL_MSG, 1);       // Enable soluton messages
	enableMsg(DOP_MSG, 0);       // Enable DOP messages
	enableMsg(DGPS_MSG, 0);     // Disable DGPS messages
}

void Init_GPS(void){
	unsigned char cc;
	Setup_GPS();
	
	if(uart_available(ToGPS)){
		cc = uart_read(ToGPS);
		unsigned char flag = 1;
		while(flag){
			length = 0;
			cc = uart_read(ToGPS);
			while( cc! = 0xB5 ) cc = uart_read(ToGPS);	// wait for sync 1 (0xB5)
			while( cc! = 0x62) cc = uart_read(ToGPS);	// wait for sync 2 (0x62)
			cc = uart_read(ToGPS);						// wait for class code
			code = cc; ck1 += cc; ck2 += ck1;
			cc = uart_read(ToGPS);						// wait for Id
			id = cc; ck1 += cc; ck2 += ck1;
			cc = uart_read(ToGPS);						// wait for length byte 1
			length = cc; ck1 += cc; ck2 += ck1;
			cc = uart_read(ToGPS);						// wait for length byte 2
			length |= ((unsigned int) cc << 8);
			ck1 += cc; ck2 += ck1; idx = 0;
			if (length > MAX_LENGTH) flag = 1;
			while(!(idx >= length)){					// wait for <length> payload bytes
				cc = uart_read(ToGPS);
				data[idx++] = cc; ck1 += cc; ck2 += ck1;
			}					
			cc = uart_read(ToGPS);	// wait for checksum 1
			chk1 = cc
			cc = uart_read(ToGPS);	// wait for checksum 2
			chk2 = cc;
			checkOk = (ck1 == chk1)  &&  (ck2 == chk2);
			flag = 0;
		} //Fin While
		if(checkOk){
			switch (code) {
				case 0x01:      // NAV-
					if (lastTime != ULONG(0)) lastTime = ULONG(0);
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
			} //Fin Switch code
		} // Fin If checkOk
	}//Fin If uart_available()	
} // Fin Init_GPS()



#endif /* GPS_H_ */