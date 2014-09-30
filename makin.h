
#define _set(_REG,_BIT) _REG |= (1<<_BIT) 			//set bit
#define _clear(_REG,_BIT) _REG &= ~(1<<_BIT) 		//clear bit
#define _togle(_REG,_BIT) _REG ^= (1<<_BIT)			//togle bit

//~ 
//~ void initpwm(void);
//~ void initadc(void);
//~ uint16_t baca(uint8_t);
//~ uint8_t cetak_bil(uint16_t, uint8_t, uint8_t,uint8_t);
//~ uint8_t isset(uint8_t,uint8_t);
//~ uint8_t isclear(uint8_t,uint8_t);
//~ void lcdhapus(uint8_t, uint8_t, uint8_t);

uint8_t isset(uint8_t _REG,uint8_t _BIT)
{
	if (bit_is_set(_REG,_BIT))
		return 1;
	else
		return 0;
}
uint8_t isclear(uint8_t _REG,uint8_t _BIT)
{
	if (bit_is_clear(_REG,_BIT))
		return 1;
	else
		return 0;
}

void initpwm(void)			//initialize PWM				
{	
	TCCR1A = _BV(WGM10)    //0xF1 11110001			 		    
		   | _BV(COM1A0)
		   | _BV(COM1A1)
		   | _BV(COM1B0)	// set OC1A/B on compare match
		   | _BV(COM1B1);		
	TCCR1B = _BV(CS11)	// 00000011 nilai OCR1 sebagai nilai fall
			|_BV(CS10);  				
}	
void initadc(void)
{
	//register adc multiplexer
	//========================
	//bit refs1=0 refs0 = 1 : adc reference di AVCC
	ADMUX |= (0<<REFS1) | (1<<REFS0);
	//register ADC Control and Status Register A
	//aktivin adc, bit ADEN di ADCSRA diisi 1
	ADCSRA |= (1<<ADEN);
	// milih prescaler = 64, 
	//kecepatan adc=FCPU/64=11059200/64=172.8KHz
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	
}

uint16_t adc_baca(uint8_t ch)
{
	//milih channel,
	//biar gak numpuk yg dipilih sama nilai ch sebelumnya di register admux, register channel dikosongke sik
	//admux = 0b11000101
	//di and sama nilai admux dengan 5 bit pertama di kasih nilai 0
	//ADMUX &= (0<<MUX4);
	_clear(ADMUX,MUX4);
	_clear(ADMUX,MUX3);
	_clear(ADMUX,MUX2);
	_clear(ADMUX,MUX1);
	_clear(ADMUX,MUX0);
	
	//ADMUX &= (0<<MUX3);
	//ADMUX &= (0<<MUX2);
	//ADMUX &= (0<<MUX1);
	//ADMUX &= (0<<MUX0);
	//trs dilebokke ning admux nilai ch
	ADMUX |= ch;
	//mulai conversi, bit ADSC di ADCSRA diisi 1
	ADCSRA |= (1<<ADSC);
	//loop terus sampe conversi selesai, nek selesai bit ADSC berubah 0 otomatis
	while(ADCSRA & (1<<ADSC))
	{
	}
	//fungsi dikasih nilai register adc 
	return(ADC);
}

uint8_t cetak_bil(uint16_t bil, uint8_t x, uint8_t y,uint8_t _c)
{
	uint8_t pjg;
	char lcdchar[30];
			LCDGotoXY(x,y);
			if (_c!=0)
			{
				
				for (uint8_t i=1; i<=_c; i++)
				{
					LCDstring(" ",1);
				}
				LCDGotoXY(x,y);
			}
			snprintf(lcdchar,15, "%d",bil);
			pjg = strlen(lcdchar);
			LCDstring(lcdchar,pjg);
	
}

uint8_t cetak_bil_lgsg(uint16_t bil, uint8_t _c)
{
	uint8_t pjg;
	char lcdchar[30];
	if (_c!=0)
	{
		
		for (uint8_t i=1; i<=_c; i++)
		{
			LCDstring(" ",1);
		}
	}
	snprintf(lcdchar,15, "%d",bil);
	pjg = strlen(lcdchar);
	LCDstring(lcdchar,pjg);
	
}

void lcdhapus(uint8_t xawal, uint8_t xakhir, uint8_t _Y)
{
	LCDGotoXY(xawal,_Y);
	for (uint8_t i=xawal;i<=xakhir;i++)
	{
		LCDstring(" ",1);
	}
}
