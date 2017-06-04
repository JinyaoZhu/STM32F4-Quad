#include "bsp.h"

#define RX_BUFFER_SIZE		1024		
			
		  
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{	 		    
	u8 *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}

u32 NMEA_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}

int NMEA_Str2num(u8 *buf,u8*dx)
{
	u8 *p=buf;
	u32 ires=0,fres=0;
	u8 ilen=0,flen=0,i;
	u8 mask=0;
	int res;
	while(1)
	{
		if(*p=='-'){mask|=0X02;p++;}
		if(*p==','||(*p=='*'))break;
		if(*p=='.'){mask|=0X01;p++;}
		else if(*p>'9'||(*p<'0'))	
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;
	for(i=0;i<ilen;i++)
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	
	*dx=flen;	 		
	for(i=0;i<flen;i++)	
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	  

//GPRMC
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;     
	u32 temp;	   
	float rs;  
	p1=(u8*)strstr((const char *)buf,"GPRMC");
	posx=NMEA_Comma_Pos(p1,1);								
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	 	 
	}	
	posx=NMEA_Comma_Pos(p1,3);								
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	
		rs=temp%NMEA_Pow(10,dx+2);			
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;
	}
	posx=NMEA_Comma_Pos(p1,4);							
	if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
 	posx=NMEA_Comma_Pos(p1,5);								
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);
		rs=temp%NMEA_Pow(10,dx+2);					 
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;
	}
	posx=NMEA_Comma_Pos(p1,6);						
	if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
	posx=NMEA_Comma_Pos(p1,9);							
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 			
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	 	 
	} 

}

//GPGSV
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p,*p1,dx;
	u8 len,i,j,slx=0;
	u8 posx;   	 
	p=buf;
	p1=(u8*)strstr((const char *)p,"$GPGSV");
	len=p1[7]-'0';								
	posx=NMEA_Comma_Pos(p1,3); 			
	if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(u8*)strstr((const char *)p,"$GPGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	
			else break;
			slx++;	   
		}   
 		p=p1+1;
	}   
}

//GPGGA
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"$GPGGA");
	posx=NMEA_Comma_Pos(p1,6);						
	if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);	
	posx=NMEA_Comma_Pos(p1,7);							
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
	posx=NMEA_Comma_Pos(p1,9);							
	if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}

//GPGSA
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx; 
	u8 i;   
	p1=(u8*)strstr((const char *)buf,"$GPGSA");
	posx=NMEA_Comma_Pos(p1,2);						
	if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
	for(i=0;i<12;i++)							
	{
		posx=NMEA_Comma_Pos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break; 
	}				  
	posx=NMEA_Comma_Pos(p1,15);							
	if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,16);								
	if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,17);								
	if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}

//GPVTG
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"$GPVTG");							 
	posx=NMEA_Comma_Pos(p1,7);							
	if(posx!=0XFF)
	{
		gpsx->speed=NMEA_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		
	}
}  

void Send_NMEA_GPRMC(nmea_msg *gpsx)
{
	BSP_Ser_Printf("year:%d ", gpsx->utc.year);
	BSP_Ser_Printf("month:%d ", gpsx->utc.month);
	BSP_Ser_Printf("date:%d ", gpsx->utc.date);
	BSP_Ser_Printf("hour:%d ", (gpsx->utc.hour+8)%24);
	BSP_Ser_Printf("min:%d ", gpsx->utc.min);
	BSP_Ser_Printf("sec:%d ", gpsx->utc.sec);
	
	BSP_Ser_Printf("%c latit:", gpsx->nshemi);
	BSP_Ser_Printf("%f ", (float)gpsx->latitude/100000);

	BSP_Ser_Printf("%c longi:", gpsx->ewhemi);
	BSP_Ser_Printf("%f\r\n", (float)gpsx->longitude/100000);
}

void Send_NMEA_GPGSV(nmea_msg *gpsx)
{
	int i, j = gpsx->svnum;
	BSP_Ser_Printf("satellites in view£º%d \r\n", gpsx->svnum);
	BSP_Ser_Printf("satellite number  elevation in degrees    azimuth in degrees to true   SNR in dB\r\n");
	for (i = 0; i < j; i++)
	{
		BSP_Ser_Printf("%d\t\t\t",gpsx->slmsg[i].num);
		BSP_Ser_Printf("%d\t\t\t",gpsx->slmsg[i].eledeg);
		BSP_Ser_Printf("%d\t\t\t",gpsx->slmsg[i].azideg);
		BSP_Ser_Printf("%d",gpsx->slmsg[i].sn);
		BSP_Ser_Printf("\r\n");
	}
}

void Send_NMEA_GPGGA(nmea_msg *gpsx)
{
	BSP_Ser_Printf("GPS quality indicator: %d    Number of satellites in view: %d    high:%.3f\r\n", \
		gpsx->gpssta, gpsx->posslnum, (float)gpsx->altitude/10);
}

void Send_NMEA_GPGSA(nmea_msg *gpsx)
{
	int i;
	BSP_Ser_Printf("mode: %d\tPDOP: %.3f\tHDOP:%.3f\tVDOP:%.3f", \
		gpsx->fixmode, (float)gpsx->pdop/100, (float)gpsx->hdop/100, (float)gpsx->vdop/100);
	BSP_Ser_Printf("\r\nNO:");
	for (i = 0 ; i < gpsx->posslnum; i++)
	{
		BSP_Ser_Printf("%d ",gpsx->possl[i]);
	}
	BSP_Ser_Printf("\r\n");
}

void Send_NMEA_GPVTG(nmea_msg *gpsx)
{
	BSP_Ser_Printf("speed: %f\r\n\r\n", (float)gpsx->speed/1000);
}

void GPSMSG_Analysis(nmea_msg *gpsx, u8 *buf)
{
		NMEA_GPRMC_Analysis(gpsx,buf);	
		NMEA_GPGSV_Analysis(gpsx,buf);	
		NMEA_GPGGA_Analysis(gpsx,buf);	
		NMEA_GPGSA_Analysis(gpsx,buf);	
		NMEA_GPVTG_Analysis(gpsx,buf);	
}

void Send_NMEA_MSG(nmea_msg *gpsx)
{
		Send_NMEA_GPRMC(gpsx);
		Send_NMEA_GPGSV(gpsx);
		Send_NMEA_GPGGA(gpsx);
		Send_NMEA_GPGSA(gpsx);
		Send_NMEA_GPVTG(gpsx);
}
