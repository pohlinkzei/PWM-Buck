/*
 * twislave.c
 *
 *  Author: uwegw
 */ 

#include <util/twi.h> 	    //enth�lt z.B. die Bezeichnungen f�r die Statuscodes in TWSR
#include <avr/interrupt.h>  //dient zur Behandlung der Interrupts
#include <stdint.h>         //definiert den Datentyp uint8_t       
#include "twi_slave.h"

//%%%%%%%% Globale Variablen, die vom Hauptprogramm genutzt werden %%%%%%%%
/*Der Buffer, in dem die Daten gespeichert werden. 
Aus Sicht des Masters l�uft der Zugrif auf den Buffer genau wie bei einem I2C-EEPROm ab.
F�r den Slave ist es eine globale Variable
*/
volatile uint8_t buffer_adr; //"Adressregister" f�r den Buffer

/*Initaliserung des TWI-Inteface. Muss zu Beginn aufgerufen werden, sowie bei einem Wechsel der Slave Adresse
Parameter adr: gew�nschte Slave-Adresse
*/
void init_twi_slave(uint8_t adr)
{
    TWAR= adr; //Adresse setzen
	TWCR &= ~(1<<TWSTA)|(1<<TWSTO);
	TWCR|= (1<<TWEA) | (1<<TWEN)|(1<<TWIE); 	
	buffer_adr=0xFF;  
	sei();
}


//Je nach Statuscode in TWSR m�ssen verschiedene Bitmuster in TWCR geschreiben werden(siehe Tabellen im Datenblatt!). 
//Makros f�r die verwendeten Bitmuster:

//ACK nach empfangenen Daten senden/ ACK nach gesendeten Daten erwarten
#define TWCR_ACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);  

//NACK nach empfangenen Daten senden/ NACK nach gesendeten Daten erwarten     
#define TWCR_NACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);

//switch to the non adressed slave mode...
#define TWCR_RESET TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC);  


/*ISR, die bei einem Ereignis auf dem Bus ausgel�st wird. Im Register TWSR befindet sich dann 
ein Statuscode, anhand dessen die Situation festgestellt werden kann.
*/
ISR (TWI_vect)  {
	uint8_t data=0;
	i2crxready = 0;
	switch (TW_STATUS) //TWI-Statusregister pr�fen und n�tige Aktion bestimmen 
	{

	// Slave Receiver 
	//case TW_GCALL_ACK:
		
	case TW_SR_SLA_ACK: // 0x60 Slave Receiver, Slave wurde adressiert	
		TWCR_ACK; // n�chstes Datenbyte empfangen, ACK danach senden
		//LEDPORT ^= (1<<LED);
		buffer_adr=0xFF; //Bufferposition ist undefiniert
	break;
	
	case TW_SR_DATA_ACK: // 0x80 Slave Receiver, ein Datenbyte wurde empfangen
		data=TWDR; //Empfangene Daten auslesen
		if (buffer_adr == 0xFF) //erster Zugriff, Bufferposition setzen
			{
				//Kontrolle ob gew�nschte Adresse im erlaubten bereich
				if(data<i2c_buffer_size+1)
					{
						buffer_adr = data; //Bufferposition wie adressiert setzen
					}
				else
					{
						buffer_adr=0xFF; //Adresse auf 0xFF setzen. Ist das sinnvoll? TO DO!
					}				
				TWCR_ACK;	// n�chstes Datenbyte empfangen, ACK danach, um n�chstes Byte anzufordern
			}
		else //weiterer Zugriff, nachdem die Position im Buffer gesetzt wurde. NUn die Daten empfangen und speichern
			{
		
				if(buffer_adr<i2c_buffer_size+1)
					{
							i2crxdata[buffer_adr]=data; //Daten in Buffer schreibe
								
					}
				buffer_adr++; //Buffer-Adresse weiterz�hlen f�r n�chsten Schreibzugriff
				
				TWCR_ACK;	
			}
	break;


	//Slave transmitter

	case TW_ST_SLA_ACK: //0xA8 Slave wurde im Lesemodus adressiert und hat ein ACK zur�ckgegeben.
		//Hier steht kein break! Es wird also der folgende Code ebenfalls ausgef�hrt!
	case TW_ST_DATA_ACK: //0xB8 Slave Transmitter, Daten wurden angefordert

		if (buffer_adr == 0xFF) //zuvor keine Leseadresse angegeben! 
			{
				buffer_adr=0;
			}	
		
		if(buffer_adr<i2c_buffer_size+1)	
			{
				TWDR = i2ctxdata[buffer_adr]; //Datenbyte senden
				buffer_adr++; //bufferadresse f�r n�chstes Byte weiterz�hlen
			}
		else
			{
				TWDR=0; //Kein Daten mehr im Buffer
				buffer_adr = 0xFF;
			}
		TWCR_ACK;
	break;
	case TW_SR_STOP:
		i2crxready = 1;
	case TW_SR_GCALL_ACK:
				TWCR_ACK;
				buffer_adr = 0xFF;
			break;
	case TW_ST_DATA_NACK: // 0xC0 Keine Daten mehr gefordert 
	case TW_SR_DATA_NACK: // 0x88 
	case TW_ST_LAST_DATA: // 0xC8  Last data byte in TWDR has been transmitted (TWEA = �0�); ACK has been received
	
	default: 	
		TWCR_RESET;
	break;
	
	} //end.switch (TW_STATUS)
} //end.ISR(TWI_vect)

////Ende von twislave.c////

