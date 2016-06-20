
#ifndef UART_H_
#define UART_H_

#include <stdlib.h>

#define BAUD 9600
#define BAUD2 38400
#define BAUDRATE (F_CPU / 4 / BAUD - 1) / 2
#define BAUDRATE2 (F_CPU / 4 / BAUD2 - 1) / 2
#define SERIAL_BUFFER_SIZE 64

typedef  struct ring_buffer
{
  unsigned char buffer[SERIAL_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
} ring_buffer;

ring_buffer rx_buffer0  =  { { 0 }, 0, 0 };
ring_buffer tx_buffer0  =  { { 0 }, 0, 0 };
ring_buffer rx_buffer1  =  { { 0 }, 0, 0 };
ring_buffer tx_buffer1  =  { { 0 }, 0, 0 };
ring_buffer rx_buffer2  =  { { 0 }, 0, 0 };
ring_buffer tx_buffer2  =  { { 0 }, 0, 0 };
ring_buffer rx_buffer3  =  { { 0 }, 0, 0 };
ring_buffer tx_buffer3  =  { { 0 }, 0, 0 };

void uart_init(uint8_t);
void uart_send(uint8_t, unsigned char);
unsigned char uart_read(uint8_t);
void uart_print(uint8_t, const char *);
void uart_println(uint8_t, const char *);

inline void store_char(unsigned char c, ring_buffer *buffer)
{
  unsigned int i = (unsigned int)(buffer->head + 1) % SERIAL_BUFFER_SIZE;
  
  if (i != buffer->tail) {
    buffer->buffer[buffer->head] = c;
    buffer->head = i;
  }
}
/*
ISR(USART0_RX_vect) {
  if (is_low(UCSR0A, UPE0)) {
    unsigned char c = UDR0;
    store_char(c, &rx_buffer0);
  } else {
    unsigned char c = UDR0;
  }
}
ISR(USART1_RX_vect) {
  if (is_low(UCSR1A, UPE1)) {
    unsigned char c = UDR1;
    store_char(c, &rx_buffer1);
  } else {
    unsigned char c = UDR1;
  }
}
ISR(USART2_RX_vect) {
  if (is_low(UCSR2A, UPE2)) {
    unsigned char c = UDR2;
    store_char(c, &rx_buffer2);
  } else {
    unsigned char c = UDR2;
  }
}
ISR(USART3_RX_vect) {
  if (is_low(UCSR3A, UPE3)) {
    unsigned char c = UDR3;
    store_char(c, &rx_buffer3);
  } else {
    unsigned char c = UDR3;
  }
}
*/
/*
ISR(USART0_UDRE_vect) {
  if (tx_buffer0.head == tx_buffer0.tail) {
    cbi(UCSR0B, UDRIE0);
  } else {
    unsigned char c = tx_buffer0.buffer[tx_buffer0.tail];
    tx_buffer0.tail = (tx_buffer0.tail + 1) % SERIAL_BUFFER_SIZE;
    UDR0 = c;
  }
}
ISR(USART1_UDRE_vect) {
  if (tx_buffer1.head == tx_buffer1.tail) {
    cbi(UCSR1B, UDRIE1);
  } else {
    unsigned char c = tx_buffer1.buffer[tx_buffer1.tail];
    tx_buffer1.tail = (tx_buffer1.tail + 1) % SERIAL_BUFFER_SIZE;
    UDR1 = c;
  }
}
ISR(USART2_UDRE_vect) {
  if (tx_buffer2.head == tx_buffer2.tail) {
    cbi(UCSR2B, UDRIE2);
  } else {
    unsigned char c = tx_buffer2.buffer[tx_buffer2.tail];
    tx_buffer2.tail = (tx_buffer2.tail + 1) % SERIAL_BUFFER_SIZE;
    UDR2 = c;
  }
}
ISR(USART3_UDRE_vect) {
  if (tx_buffer3.head == tx_buffer3.tail) {
    cbi(UCSR3B, UDRIE3);
  } else {
    unsigned char c = tx_buffer3.buffer[tx_buffer3.tail];
    tx_buffer3.tail = (tx_buffer3.tail + 1) % SERIAL_BUFFER_SIZE;
    UDR3 = c;
  }
}*/


void uart_init(uint8_t num) {
  switch(num) {
  case 0:
    UCSR0A|= (1<<U2X0);
    UBRR0H = (BAUDRATE>>8);             // shift the register right by 8 bits
    UBRR0L = BAUDRATE;                  // set baud rate
    UCSR0B|= (1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);     // enable receiver and transmitter
    UCSR0C|= (1<<UCSZ00)|(1<<UCSZ01);   // 8bit data format
    break;
  case 1:
    UCSR1A|= (1<<U2X1);
    UBRR1H = (BAUDRATE>>8);             // shift the register right by 8 bits
    UBRR1L = BAUDRATE;                  // set baud rate
    UCSR1B|= (1<<TXEN1)|(1<<RXEN1)|(1<<RXCIE1);     // enable receiver and transmitter
    UCSR1C|= (1<<UCSZ10)|(1<<UCSZ11);   // 8bit data format
    break;
  case 2:
    UCSR2A|= (1<<U2X2);
    UBRR2H = (BAUDRATE2>>8);             // shift the register right by 8 bits
    UBRR2L = BAUDRATE2;                  // set baud rate
    UCSR2B|= (1<<TXEN2)|(1<<RXEN2)|(1<<RXCIE2);     // enable receiver and transmitter
    UCSR2C|= (1<<UCSZ20)|(1<<UCSZ21);   // 8bit data format
    break;
  case 3:
    UCSR3A|= (1<<U2X3);
    UBRR3H = (BAUDRATE>>8);             // shift the register right by 8 bits
    UBRR3L = BAUDRATE;                  // set baud rate
    UCSR3B|= (1<<TXEN3)|(1<<RXEN3)|(1<<RXCIE3);     // enable receiver and transmitter
    UCSR3C|= (1<<UCSZ30)|(1<<UCSZ31);   // 8bit data format
    break;
  }
}

void uart_send(uint8_t num, unsigned char data) {
  switch(num) {
  case 0:
    while (!( UCSR0A & (1<<UDRE0)));      // wait while register is free
    UDR0 = data;                          // load data in the register
    break;
  case 1:
    while (!( UCSR1A & (1<<UDRE1)));      // wait while register is free
    UDR1 = data;                          // load data in the register
    break;
  case 2:
    while (!( UCSR2A & (1<<UDRE2)));      // wait while register is free
    UDR2 = data;                          // load data in the register
    break;
  case 3:
    while (!( UCSR3A & (1<<UDRE3)));      // wait while register is free
    UDR3 = data;                          // load data in the register
    break;
  }
}
/*
unsigned char uart_read(uint8_t num) {
  switch(num) {
  case 0:
    while(!(UCSR0A) & (1<<RXC0));         // wait while data is being received
    return UDR0;                          // return 8-bit data
    break;
  case 1:
    while(!(UCSR1A) & (1<<RXC1));         // wait while data is being received
    return UDR1;                          // return 8-bit data
    break;
  case 2:
    while(!(UCSR2A) & (1<<RXC2));         // wait while data is being received
    return UDR2;                          // return 8-bit data
    break;
  case 3:
    while(!(UCSR3A) & (1<<RXC3));         // wait while data is being received
    return UDR3;                          // return 8-bit data
    break;
  }
  return 0;
}
*/

unsigned char uart_fetch(ring_buffer *buffer) {
  if(buffer->head == buffer->tail)
    return -1;
  else {
    unsigned char c = buffer->buffer[buffer->tail];
    buffer->tail = (unsigned int)(buffer->tail + 1) % SERIAL_BUFFER_SIZE;
    return c;
  }
}

unsigned char uart_read(uint8_t num) {
  ring_buffer buffer;
  
  switch(num) {
  case 0:
    return uart_fetch(&rx_buffer0);
    break;
  case 1:
    return uart_fetch(&rx_buffer1);
    break;
  case 2:
    return uart_fetch(&rx_buffer2);
    break;
  case 3:
    return uart_fetch(&rx_buffer3);
    break;
  }
  return -1;
}

uint8_t uart_available(uint8_t num) {
  ring_buffer buffer;
  
  switch(num) {
  case 0:
    buffer = rx_buffer0;
    break;
  case 1:
    buffer = rx_buffer1;
    break;
  case 2:
    buffer = rx_buffer2;
    break;
  case 3:
    buffer = rx_buffer3;
    break;
  }
  
  return (unsigned int)(SERIAL_BUFFER_SIZE + buffer.head - buffer.tail) % SERIAL_BUFFER_SIZE;
}

void uart_print(uint8_t num, const char *s) {
  register char c;
  /*
  while (*s) {
	  uart_send(num, *s);
	  s++;
  }
  */
  
  while((c = *s++)) {
    uart_send(num, c);
  }
  
}

void uart_println(uint8_t num, const char *s) {
  register char c;
  /*
  while (*s) {
	  uart_send(num, *s);
	  s++;
  }
  */
  
  while((c = *s++)) {
    uart_send(num, c);
  }
  
  uart_send(num, 13);
  uart_send(num, 10);
  
}
/*
void uart_read(uint8_t num, char *s) {
  
}
*/

#endif /* UART_H_ */