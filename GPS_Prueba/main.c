#include <avr/io.h>
#define F_CPU		16000000UL
#include "GPS.h"
#include "uart.h"

int main(void){
	Setup_GPS();
	uart_println(ToPC, "Acaba DE TERMINAR EL SETUP");
    while (1) {
		Syncronization();
    } //Fin While(1)
} // Fin main()

