#include "xil_printf.h"
#include "lwip/sys.h"
#include "lwipopts.h"
#include <math.h>
#include <string.h>

/* crc16_ccitt  x16+x12+x5+1 (0x1021)
init crc= 0x0000
lower bit is front and higher bit is back ,XOR
the test website: http://www.ip33.com/crc.html
*/
u16 crc_16(u8* data, u16 len)
{
	u16 crc16=0x0000;
	while(len--)
	{
//		for(u8 i=0x80;i!=0;i>>=1)
//		{
//			if((crc16 & 0x8000)!=0)
//			{
//			crc16 = crc16<<1;
//			crc16 = crc16 ^ 0x1021;
//			}
//			else{
//				crc16 = crc16 <<1;
//			}
//			if((*data &i) !=0)
//			{
//				crc16 =crc16 ^0x1021;
//			}
//		}
//		data++;
		 crc16 ^= *data++;        // crc ^= *data; data++;

		 for (int i = 0; i < 8; ++i)
		 {
		     if (crc16 & 1)
		         crc16 = (crc16 >> 1) ^ 0x8408;        // 0x8408 = reverse 0x1021
		      else
		          crc16 = (crc16 >> 1);
		  }

	}
	return crc16;
}


//little-endian convert to big-endian
u32_t convert_end32(u32_t A)
{
	A=((A<<8)& 0xFF00FF00) |((A>>8)&0xFF00FF);
	return (A<<16) | (A>>16);
}

u16_t convert_end16(u16_t B)
{
	B=(B <<8) | (B >>8);
	return B;
}



