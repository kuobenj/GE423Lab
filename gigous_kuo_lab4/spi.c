
#include <tistdtypes.h>
#include <coecsl.h>
#include "28335_spi.h"
#include "user_includes.h"

long SPIbyte1,SPIbyte2,SPIbyte3,SPIbyte4,SPIbyte5;
long SPIenc1_reading = 0;
long SPIenc2_reading = 0;
long SPIenc3_reading = 0;
long SPIenc4_reading = 0;
unsigned char MCP23S08_READ = 0;
int SPIenc_state = 0;
int	SPIenc_state_errors = 0;

unsigned int dac1data = 0;
unsigned int dac2data = 0;

void init_SPI(void){
	/*****************************************************/
	/*********  SPI  *************************************/

/*
	The below code sets up several general purpose registers to outputs. It then sets the GPIOs to high, which is sent to four LS7366s, to their SS inputs (5V). This initiates the transmission cycle between the microcontroller and the LS7366s. 
*/
	EALLOW;
		
		GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
		GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
		GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;
		GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;

		GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;

		GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;
		GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
		GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;
		GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;

		GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;

		GpioDataRegs.GPASET.bit.GPIO9 = 1;
		GpioDataRegs.GPASET.bit.GPIO10 = 1;
		GpioDataRegs.GPASET.bit.GPIO11 = 1;
		GpioDataRegs.GPASET.bit.GPIO22 = 1;

	EDIS;

	InitSpiaGpio();

	EALLOW;
	// SS for DAC7564
		GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;  // use GPIO19 for SS
		GpioDataRegs.GPASET.bit.GPIO19 = 1;  // enabled
		GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;  // GPIO19 as output
	EDIS;

/*
This code gets the transmit and receive buffers ready, as well as communication between the microcontroller and the LS7366 boards (master is MC, slaves are LSs). It clears the buffers, sets them up as queues (FIFO), and clears and readies their interrupts.
*/
	SpiaRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in reset (to change settings)

	SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;  // set for LS7366
	SpiaRegs.SPICTL.bit.CLK_PHASE = 1;

	SpiaRegs.SPICCR.bit.SPICHAR = 7;   // set to transmitt 8 bits

	SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
	SpiaRegs.SPICTL.bit.TALK = 1;

	SpiaRegs.SPICTL.bit.SPIINTENA = 0;

	SpiaRegs.SPISTS.all=0x0000;

	SpiaRegs.SPIBRR = 39;   // divide by 40 2.5 Mhz

	SpiaRegs.SPIFFTX.bit.SPIRST = 1;	// SPI resume transmit or receive
	SpiaRegs.SPIFFTX.bit.SPIFFENA = 1; // FIFO enhancements
	SpiaRegs.SPIFFTX.bit.TXFIFO = 0;	// transmit FIFO reset, hold in reset
	SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;	// clear TXFFINT flag

	SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;	//Reset FIFO pointer to 0, hold in reset
	SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // clear RXFFOVF flag (which indicates overflow or not)
	SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;	// Clear RXFFINT flag (indicates interrupts from fifo)
	SpiaRegs.SPIFFRX.bit.RXFFIL = 5;		// trip interrupt after 5 things
	SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;	// comparing will be disabled

	SpiaRegs.SPIFFCT.all=0x00;

	SpiaRegs.SPIPRI.bit.FREE = 1;
	SpiaRegs.SPIPRI.bit.SOFT = 0;


	SpiaRegs.SPICCR.bit.SPISWRESET = 1;  // Pull the SPI out of reset

	SpiaRegs.SPIFFTX.bit.TXFIFO=1;		// turn reset off
	SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;	// same



/*
This section resets the encoder counts to zero, sets filter clock division factor to 2, and sets them up for x4 quadrature mode. It does this by enabling transmit by pulling the SS\ inputs low and writing the command to clear, then sending a command to write to MDR0 along with the desired command bits.
*/
	SpiaRegs.SPIFFRX.bit.RXFFIL = 1;		// generate int after 1 or more things received
	GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;	// transmit to all chip
	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	SpiaRegs.SPITXBUF = ((unsigned)0x20)<<8;  // CLR COUNT all four chips
	while (SpiaRegs.SPIFFRX.bit.RXFFST != 1) {} // wait for until it 
	GpioDataRegs.GPASET.bit.GPIO9 = 1;	// no transmit to all chips
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
	GpioDataRegs.GPASET.bit.GPIO11 = 1;
	GpioDataRegs.GPASET.bit.GPIO22 = 1;
	SPIbyte1 = SpiaRegs.SPIRXBUF;	

	SpiaRegs.SPIFFRX.bit.RXFFIL = 2;
	GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;	// trasmit
	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	SpiaRegs.SPITXBUF = ((unsigned)0x88)<<8;  // WR to MDR0 (select MDR0)
// Filter clock division factor = 2
// x4 quadrature count mode (four counts per quadrature cycle)
	SpiaRegs.SPITXBUF = ((unsigned)0x83)<<8;	

	while (SpiaRegs.SPIFFRX.bit.RXFFST != 2) {} //wait to receive ACK
	GpioDataRegs.GPASET.bit.GPIO9 = 1;	// no trasmit
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
	GpioDataRegs.GPASET.bit.GPIO11 = 1;
	GpioDataRegs.GPASET.bit.GPIO22 = 1;
	SPIbyte1 = SpiaRegs.SPIRXBUF;		// get ACKs
	SPIbyte2 = SpiaRegs.SPIRXBUF;		// clear buffer


/*
The last part of init is writing to the MDR1 register (similar to how we wrote to MDR0), clearing interrupt signals, and enabling future interrupts. Basically, it sets the chip up to receive future data.
*/
	SpiaRegs.SPIFFRX.bit.RXFFIL = 2;
	GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	SpiaRegs.SPITXBUF = ((unsigned)0x90)<<8;  // WR MDR1
	// 4 byte counter mode
	// enable counting
	// NOP
	SpiaRegs.SPITXBUF = 0x00<<8;
	while (SpiaRegs.SPIFFRX.bit.RXFFST != 2) {} // ACK
	GpioDataRegs.GPASET.bit.GPIO9 = 1;	// no transmit
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
	GpioDataRegs.GPASET.bit.GPIO11 = 1;
	GpioDataRegs.GPASET.bit.GPIO22 = 1;
	SPIbyte1 = SpiaRegs.SPIRXBUF;		// clear buffer
	SPIbyte2 = SpiaRegs.SPIRXBUF;


	SpiaRegs.SPICTL.bit.SPIINTENA = 1;	// enable inerrupt
	SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;	// clear any indication of overflow
	SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;	// clear any indication of messages received
	SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;	// enable interrupts when rx buffer gets data

/*********  SPI  *************************************/
/*****************************************************/

	// SPI
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE
	PieCtrlRegs.PIEIER6.bit.INTx1 = 1;  //Enable PIE 6.1 interrupt

}

/*
	This code is using a state machine to select and read from the individual 
	counters, one at a time. It sends the command to read from the OTR register 
	(that holds the data), then sends 4 more bytes of garbage and receives 4 
	bytes of data. It also reads from the MCP's GPIO port by sending the 
	command to read, then the register address. It receives two bytes; the 
	second is the data we desire. Then it posts the SWI to give control to the
	RobotControl function for some really cool control stuff.
*/
void SPI_RXint(void) {

	GpioDataRegs.GPASET.bit.GPIO9 = 1;		//no transmit
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
	GpioDataRegs.GPASET.bit.GPIO11 = 1;
	GpioDataRegs.GPASET.bit.GPIO22 = 1;
	GpioDataRegs.GPASET.bit.GPIO48 = 1;
	GpioDataRegs.GPASET.bit.GPIO49 = 1;
	GpioDataRegs.GPASET.bit.GPIO58 = 1;

	GpioDataRegs.GPASET.bit.GPIO19 = 1;	// something for the DAC
// state machine
	switch (SPIenc_state) {
		case 1:
			// add delay
			SPIbyte1 = SpiaRegs.SPIRXBUF;
		

			SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
			SPIenc_state = 2; // next state
			GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
			// Read from OTR, first encoder value
			SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;

			break;
		case 2:

			SPIbyte1 = SpiaRegs.SPIRXBUF;	// ACK
			SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF; // the data
			SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIenc1_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;

			SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
			SPIenc_state = 3;
			GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
			SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;

			break;
		case 3:
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIenc2_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;

			SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
			SPIenc_state = 4;
			GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
			SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;

			break;
		case 4:
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIenc3_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;

			SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
			SPIenc_state = 5;
			GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
			SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;

			break;
		case 5:
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIenc4_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;

			SpiaRegs.SPIFFRX.bit.RXFFIL = 3;
			SPIenc_state = 8;

			//pulls the selection bit low on our design to acess the MCP chip
			GpioDataRegs.GPACLEAR.bit.GPIO48 = 1;
			GpioDataRegs.GPACLEAR.bit.GPIO49 = 1;
			GpioDataRegs.GPACLEAR.bit.GPIO58 = 1;

			SpiaRegs.SPITXBUF = ((unsigned)0x41);//read
			SpiaRegs.SPITXBUF = ((unsigned)0x09);//GPIO register
			SpiaRegs.SPITXBUF = 0;//send useless byte

			break;
		case 6: // NOOOOOOOOOOOOOOOOOOOOOOOO
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIenc_state = 7;

			// Output to DAC Ch2
			SpiaRegs.SPIFFRX.bit.RXFFIL = 3;
			SPIenc_state = 7;
			GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
			SpiaRegs.SPIFFRX.bit.RXFFIL = 3;
			SpiaRegs.SPITXBUF = ((unsigned)0x12)<<8;
			SpiaRegs.SPITXBUF = (int)(dac2data << 4);
			SpiaRegs.SPITXBUF = ((int)(dac2data))<<12;

			break;
		case 7:
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte1 = SpiaRegs.SPIRXBUF;


			SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;  // set for LS7366
			SpiaRegs.SPICTL.bit.CLK_PHASE = 1;

			SPIenc_state = 0;


			// Debug to see how long 4 SPI enc read takes
//			GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;



			break;

		case 8:
			SPIbyte1 = SpiaRegs.SPIRXBUF;//manditory read to clear buffer
			SPIbyte2 = SpiaRegs.SPIRXBUF;//manditory read to clear buffer
			MCP23S08_READ = SpiaRegs.SPIRXBUF;	// our desired value from MCP

			SPIenc_state = 0;

/* 
At this point, the microcontroller has read the encoder values and is ready to 
give control to the control (pun unintended) function. The control function can
use the newly read values.
*/

			SWI_post(&SWI_control);
			break;
/* this is for errors */
		default:
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte2 = SpiaRegs.SPIRXBUF;  // these reads are not needed except to add some delay
			SPIbyte3 = SpiaRegs.SPIRXBUF;
			SPIbyte4 = SpiaRegs.SPIRXBUF;
			SPIbyte5 = SpiaRegs.SPIRXBUF;
			SPIenc_state_errors++;
			break;
	}

	SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
	SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1; 	// Clear Interrupt flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE

}


/*
	Enables SPI communication by setting SS\ to low. 
	Enables interrupt state machine to work.
*/
void start_SPI(void) {

	SpiaRegs.SPIFFRX.bit.RXFFIL = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	// Allow CNTR to dump data to OTR
	SpiaRegs.SPITXBUF = ((unsigned)0xE8)<<8; // Latch All ENCs
	SPIenc_state = 1;
}

/*
	This code initializes the MCP chip. The function takes in the values we want
	to use to initialize the chip. To send these values, we must enable the chip by
	pulling the CS\ pin low, which means giving our decoder the value 000.

	We can now send our value. 
	we send a write command, followed by the specific
	register's address, followed by the desired value to write. 
	Note that A0 and A1 are already pulled low in hardware. We continuously
	poll the RX FIFO status bits until they are set high, in which case the data
	is successfully written. We must set the decoder input bits to high so that we can
	deselect the MCP chip. We can then write to the next register after pulling the CS\
	pin low again, and the process repeats until we have written to all the registers.
*/
void Init_MCP23S08 (unsigned int IODIRvalue, unsigned int IPOLvalue, 
	unsigned int GPPUvalue, unsigned int IOCONvalue, unsigned int OLATvalue)
{
	//pulls the selection bit low on our design
	GpioDataRegs.GPACLEAR.bit.GPIO48 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO49 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO58 = 1;

	//write to IODIR
	SpiaRegs.SPITXBUF = ((unsigned)0x40);//write command to HW address 0x00
	SpiaRegs.SPITXBUF = ((unsigned)0x00);//IODIR register address
	SpiaRegs.SPITXBUF = IODIRvalue;//send the value to IODIR

	while(SpiaRegs.SPIFFRX.bit.RXFFST != 3);

	//pulls the selection bit high on our design to finish the transfer
	GpioDataRegs.GPASET.bit.GPIO48 = 1;
	GpioDataRegs.GPASET.bit.GPIO49 = 1;
	GpioDataRegs.GPASET.bit.GPIO58 = 1;

	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;  //manditory reads to clear buffer
	SPIbyte3 = SpiaRegs.SPIRXBUF;

	//pulls the selection bit low on our design so that we can write to
	//the next register
	GpioDataRegs.GPACLEAR.bit.GPIO48 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO49 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO58 = 1;

	//write to IPOL
	SpiaRegs.SPITXBUF = ((unsigned)0x40);//write command to HW address 0x00
	SpiaRegs.SPITXBUF = ((unsigned)0x01);//IPOL register address
	SpiaRegs.SPITXBUF = IPOLvalue;//send the value

	while(SpiaRegs.SPIFFRX.bit.RXFFST != 3);

	//pulls the selection bit high on our design to finish the transfer
	GpioDataRegs.GPASET.bit.GPIO48 = 1;
	GpioDataRegs.GPASET.bit.GPIO49 = 1;
	GpioDataRegs.GPASET.bit.GPIO58 = 1;

	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;  //manditory reads to clear buffer
	SPIbyte3 = SpiaRegs.SPIRXBUF;

	//pulls the selection bit low on our design
	GpioDataRegs.GPACLEAR.bit.GPIO48 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO49 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO58 = 1;

	//write to GPPU
	SpiaRegs.SPITXBUF = ((unsigned)0x40);//write command to HW address 0x00
	SpiaRegs.SPITXBUF = ((unsigned)0x06);//GPPU register address
	SpiaRegs.SPITXBUF = GPPUvalue;//send the value

	while(SpiaRegs.SPIFFRX.bit.RXFFST != 3);

	//pulls the selection bit high on our design to finish the transfer
	GpioDataRegs.GPASET.bit.GPIO48 = 1;
	GpioDataRegs.GPASET.bit.GPIO49 = 1;
	GpioDataRegs.GPASET.bit.GPIO58 = 1;

	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;  //manditory reads to clear buffer
	SPIbyte3 = SpiaRegs.SPIRXBUF;

	//pulls the selection bit low on our design
	GpioDataRegs.GPACLEAR.bit.GPIO48 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO49 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO58 = 1;

	//write to IOCON
	SpiaRegs.SPITXBUF = ((unsigned)0x40);//write command to HW address 0x00
	SpiaRegs.SPITXBUF = ((unsigned)0x05);//IOCON register address
	SpiaRegs.SPITXBUF = IOCONvalue;//send the value

	while(SpiaRegs.SPIFFRX.bit.RXFFST != 3);

	//pulls the selection bit high on our design to finish the transfer
	GpioDataRegs.GPASET.bit.GPIO48 = 1;
	GpioDataRegs.GPASET.bit.GPIO49 = 1;
	GpioDataRegs.GPASET.bit.GPIO58 = 1;

	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;  //manditory reads to clear buffer
	SPIbyte3 = SpiaRegs.SPIRXBUF;

	//pulls the selection bit low on our design
	GpioDataRegs.GPACLEAR.bit.GPIO48 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO49 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO58 = 1;

	//write to OLAT
	SpiaRegs.SPITXBUF = ((unsigned)0x40);//write command to HW address 0x00
	SpiaRegs.SPITXBUF = ((unsigned)0x0A);//OLAT register address
	SpiaRegs.SPITXBUF = OLATvalue;//send the value to set the OLAT's value

	while(SpiaRegs.SPIFFRX.bit.RXFFST != 3);

	//pulls the selection bit high on our design to finish the transfer
	GpioDataRegs.GPASET.bit.GPIO48 = 1;
	GpioDataRegs.GPASET.bit.GPIO49 = 1;
	GpioDataRegs.GPASET.bit.GPIO58 = 1;

	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;  //manditory reads to clear buffer
	SPIbyte3 = SpiaRegs.SPIRXBUF;
}

/*
	Sends a write command to the OLAT register by sending the write command byte
	then the address of the of the OLAT register. The third byte is the value 
	that we want it to have. The function returns when the data is successfully
	written to the register.
*/
void SetPortLatch(unsigned int byte)
{
	//pulls the selection bit low on our design
	GpioDataRegs.GPACLEAR.bit.GPIO48 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO49 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO58 = 1;

	//write to OLAT
	SpiaRegs.SPITXBUF = ((unsigned)0x40);//write command to HW address 0x00
	SpiaRegs.SPITXBUF = ((unsigned)0x0A);//OLAT register
	SpiaRegs.SPITXBUF = (unsigned char)byte;//send the value

	while(SpiaRegs.SPIFFRX.bit.RXFFST != 3);

	//pulls the selection bit high on our design to finish the transfer
	GpioDataRegs.GPASET.bit.GPIO48 = 1;
	GpioDataRegs.GPASET.bit.GPIO49 = 1;
	GpioDataRegs.GPASET.bit.GPIO58 = 1;

	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;  //manditory reads to clear buffer
	SPIbyte3 = SpiaRegs.SPIRXBUF;
}

/*
	The GPIO pin is read by sending a read command, followed by the GPIO
	register's address. When the command is received, the first byte read is
	garbage, then the second byte is the data we want.
*/
unsigned int ReadPort(void)
{
	//pulls the selection bit low on our design
	GpioDataRegs.GPACLEAR.bit.GPIO48 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO49 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO58 = 1;

	//write to OLAT
	SpiaRegs.SPITXBUF = ((unsigned)0x41);//read
	SpiaRegs.SPITXBUF = ((unsigned)0x09);//GPIO register
	SpiaRegs.SPITXBUF = 0;//send garbage byte for the read

	while(SpiaRegs.SPIFFRX.bit.RXFFST != 2);

	//pulls the selection bit high on our design to finish the transfer
	GpioDataRegs.GPASET.bit.GPIO48 = 1;
	GpioDataRegs.GPASET.bit.GPIO49 = 1;
	GpioDataRegs.GPASET.bit.GPIO58 = 1;

	SPIbyte1 = SpiaRegs.SPIRXBUF;//manditory read to clear buffer
	SPIbyte2 = SpiaRegs.SPIRXBUF;//manditory read to clear buffer
	return SpiaRegs.SPIRXBUF;
}

