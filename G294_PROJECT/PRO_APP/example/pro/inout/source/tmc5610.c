#include "tmc5610.h"
#include "board_g294.h"
#include "machconfig.h"
#include "stdlib.h"

unsigned short  g_spi_status = 0;

void (*spierr_callback)(U8 mot);

void spi_err_register(void (*callback)(U8))
{
 spierr_callback=callback;
}

void tmc5160_writeDatagram(U8 motor, U8 address, U8 x1, U8 x2, U8 x3, U8 x4)
{
  U8 err=0;
	motor_spi_cs(motor,SPI_CS_LOW);

	g_spi_status = SPI1_ReadWriteByte(address|0x80,&err);
	SPI1_ReadWriteByte(x1,&err);
	SPI1_ReadWriteByte(x2,&err);
	SPI1_ReadWriteByte(x3,&err);
	SPI1_ReadWriteByte(x4,&err);

	motor_spi_cs(motor,SPI_CS_HIGH);
    if(err)
    spierr_callback(motor);
}

void tmc5160_writeInt(U8 motor, U8 address, int value)
{
	tmc5160_writeDatagram(motor, address, 0xFF & (value>>24), 0xFF & (value>>16), 0xFF & (value>>8), 0xFF & (value>>0));
}

int tmc5160_readInt(U8 motor, U8 address)
{
   U8 err=0;
	int value;
	address &= 0x7F;
    if(motor >= NUM_MOTOR)
     return -1;

	// Register not readable -> shadow register copy

	motor_spi_cs(motor,SPI_CS_LOW);
	g_spi_status = SPI1_ReadWriteByte(address,&err);
	SPI1_ReadWriteByte(0,&err);
	SPI1_ReadWriteByte(0,&err);
	SPI1_ReadWriteByte(0,&err);
	SPI1_ReadWriteByte(0,&err);
	motor_spi_cs(motor,SPI_CS_HIGH);



	motor_spi_cs(motor,SPI_CS_LOW);
	g_spi_status = SPI1_ReadWriteByte(address,&err);
	value = SPI1_ReadWriteByte(0,&err);
	value <<= 8;
	value |= SPI1_ReadWriteByte(0,&err);
	value <<= 8;
	value |= SPI1_ReadWriteByte(0,&err);
	value <<= 8;
	value |= SPI1_ReadWriteByte(0,&err);
	
	//value = g_spi_status ;
	motor_spi_cs(motor,SPI_CS_HIGH);
    if(err)   
    spierr_callback(motor);
	return value;
}
// <= SPI wrapper

int rotate(int motor, int velocity)
{
	if(motor >= NUM_MOTOR)
		return -1;


	// set absolute velocity, independant from direction
	tmc5160_writeInt(motor, TMC5160_VMAX, abs(velocity));
  
	// signdedness defines velocity mode direction bit in rampmode register
	tmc5160_writeDatagram(motor, TMC5160_RAMPMODE, 0, 0, 0, (velocity >= 0)? 1 : 2);

	return 0;
}

int right(U8 motor, int velocity)
{
 #ifdef CHECKSPEED_0
  if(velocity==0)
      mot[motor].stop_0_flag=1;
 #endif
	return rotate(motor, velocity);
}

int left(U8 motor, int velocity)
{
    #ifdef CHECKSPEED_0
  if(velocity==0)
      mot[motor].stop_0_flag=1;
 #endif
    
	return rotate(motor, -velocity);
}

int stop(U8 motor)
{
	return rotate(motor, 0);
}

int moveTo(U8 motor, int position)
{
	if(motor >= NUM_MOTOR)
		return -1;

	
	// set position
	tmc5160_writeInt(motor, TMC5160_XTARGET, position);

	// change to positioning mode
	tmc5160_writeDatagram(motor, TMC5160_RAMPMODE, 0, 0, 0, 0);
    

	return 0;
}

void setPosition(U8 motor,int position)
{	
	if(motor >= NUM_MOTOR)
			return;

	tmc5160_writeInt(motor, TMC5160_XACTUAL, position);
	
	tmc5160_writeInt(motor, TMC5160_XTARGET, position);
}

void setspeed(U8 motor,int speed)
{
    if(motor >= NUM_MOTOR)
			return;
    tmc5160_writeInt(motor, TMC5160_VMAX, speed);
}

int moveBy(U8 motor, int *ticks)
{
	// determine actual position and add numbers of ticks to move
	  *ticks = tmc5160_readInt(motor, TMC5160_XACTUAL) + *ticks;
		return moveTo(motor, *ticks);
}



unsigned short tmc5160_spi_status(void)
{
		return g_spi_status;
}


