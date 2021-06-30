/* ******************************************************************************
      MEDICION Y CONTROL BOMBAS 	CHICONAUTLA

      VERSION 	1  04/Sep/2019
      RUTA 		C:/CHICONAUTLA/software/CHICONUATLA_1.C
      IDE 		DINAMIC C 10.72

      PORTC............... RS485  HMI			 					_gate
		  PORTE............... RS232  SCAIRLINK						_alter
		  PORTF............... RS232  administracion

      --------------------IMPORTANTE REVISAR TODAS LAS COSAS POR HACER (TODO:) ANTES DE PROGRAMAR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


Ver 1.0	Se anexa el numero de version en los registros 90 y 91 MB
Ver 4.0  Simulacion de las salidas indicador WIFI y ademas remapeo de I/O Rabbit
Ver 2.7 Definitiva hasta 24 feb 2021
Ver 2.8 Se Agregó reset de isntrumentacion en Dir 97, pin 7
*/

//Para debug se pone <#define PRINTFDEBUG printf> para que no haya debug se pone <#define PRINTFDEBUG //printf>

  #define PRINTFDEBUG printf
  //#define DEBUG
  #define			addr_HMI							5
  #define        Numero_sitio               1                                   // WEB // TODO: Poner el Numero_sitio correspondiente al Scairlink
  #define        dir_scairlink              100 + Numero_sitio                  // VO 1 SCAIRLINK
  #define			dir_medidor_flujo				4  // TODO: Poner la dir_medidor_flujo correspondiente
  #define 			reg_flujo              8 // TODO: Poner el registro del medidor en dodne esta el flujo inst
  #define           reg_totalizado         0	// TODO: Poner el registro del medidos en donde esta el reg_totalizado
  #define        inicio_tab_hmi             100
  #define endress_485 // TODO: Si el sensor es endressx485 dejar esta linea como está, si sera medidor 4_20 mA comentar toda esta línea
  //TODO: si es 4_20mAm,  poner en la HMI el K2  de la An 4 (0,1,2,3,4) que es el multiplicador del totalizado, por si no es cada m3 el pulso
  //CONF_ANA[4].K2_data aqui se pone el ID del medidor para LERMA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  #define			No_max_errores_instru		10

  const unsigned char Nom_Proy[]=				"Proyecto LERMA";
  const unsigned char Version[]=				"Version 28.0";
  const unsigned char Fecha_Ver[]=				"24/FEB/2021";


  #ifdef endress_485
  #define			No_med_corrien					1
  #else
  #define     No_med_corrien          2
  #endif

  #define			ver_sion		  					28.0
  #define			addr_reg_mb_version			2030
  #define			addr_reg_conta_sec_hmi		2026
  #define			addr_reg_mb_ctrl_hmi			2023
  #define			addr_reg_mb_ctrl_regio		2024
  #define			addr_reg_mb_ctrl_web			2020
  #define			U_segundos						2010													// Tiempo para UTR's
  #define			U_minutos						2011
  #define			U_horas							2012
  #define			U_dia								2013
  #define			U_mes								2014
  #define			U_año								2015
  #define			U_set_tiempo					2016

  #define			W_segundos						2000													// Tiempo desde WEB
  #define			W_minutos						2001
  #define			W_horas							2002
  #define			W_dia								2003
  #define			W_mes								2004
  #define			W_año								2005
  #define			W_set_tiempo					2006
/////////////////////////////////////////////////////////////////////////////////
//       V A R I A B L E S      D E        C O N T R O L
/////////////////////////////////////////////////////////////////////////////////

  #define				No_Med_Insta						10

  #define				Fuera_Rango_Bajo					0
  #define				Dentro_Rango						1
  #define				Fuera_Rango_Alto					2

  #define				Fuera_limite_Bajo					0
  #define				Dentro_Limite						1
  #define				Fuera_Limite_Alto					2
/////////////////////////////////////////////////////////////////////////////////
//			DEFINICION CONFIGURACION HARDWARE
/////////////////////////////////////////////////////////////////////////////////

  #define			No_Bbas_max						1
  #define			pin_estado_bba					8
  #define			pin_estado_perilla			9

  #define			pin_perilla_local				16
  #define			pin_perilla_remo				17

  #define			status_bba						115
  #define			status_perilla					116

  #define 			dio_reset_router_cel			18
  #define 			dio_reset_scairlink			19
  #define			dio_reset_rabbit				20												// RELE 2

  #define			sal_E1							29
  #define			sal_E2							30
  #define			sal_E3							31

  #define 			dio_puerta_abierta			21
  #define 			dio_mantenimiento				22
  #define 			dio_falla_ac					23

  #define			volt_bateria					120

  #define 			puerta_abierta			  		117
  #define 			falla_ac							118
  #define 			mantenimiento					119

  #define			enlace_scairlink				122
  #define			Reg_Tiempo						123

  #define			pin_totalizador				15
  #define			dio_reset_instru				7


  #define			reset_scairlink				2096
  #define			reset_instru					2097
  #define			reset_celular					2098
  #define			reset_rabbit					2099




/*******************************************************************************
   DEFINICION DRIVER MICTO SD
*******************************************************************************/

  //#define SDFLASH_DEBUG
  #use "sdflash.lib"
  char flash_buf[512];
  sd_device *dev;
  unsigned int	conta_secuencial;

  typedef struct
  {
  	float				valor;
     unsigned int	index_imagen;
     unsigned int	status_rango;
     unsigned int	status_limite;
  }val_analog;
  val_analog 	VAL_ANA[5];

  typedef struct
  {
  	float				K1_data;
     float				K2_data;
     float				K3_data;
     float				Lim_H_data;
     float				Lim_L_data;
     float				Reser_data;
  }datos_analog;
  datos_analog 	CONF_ANA[5];

  typedef struct
  {
  	float				valor;
     int				rango;
  }val_crudos;
  val_crudos 		MED_CRUDA[10];

  #define		No_max_suma				5
  //unsigned int No_max_sumaVal =No_max_suma;
  typedef struct
  {
  	float				sumando[No_max_suma];
  }val_prom;

  val_prom				promedio[No_med_corrien + 1];

  float					valor_promedio;
/////////////////////////////////////////////////////////////////////////////////
//			DEFINICION CONFIGURACION TABLA        H M I
/////////////////////////////////////////////////////////////////////////////////

  unsigned int 	conta_secuen_hmi;
  unsigned int 	cmd_control_hmi;
  unsigned int	cmd_control_regional;

  			int 	*apunta_tiempo;
  			int 	tabla_modbus[100];

  unsigned int   flag_ejecutar;
  unsigned char	ejecuta_mando;
  unsigned char	No_salida;
/*******************************************************************************
	CONFIGURACION DE LA RED ETHERNET
*******************************************************************************/

  static bbram char  new_ip[25];
  static bbram char  new_nm[25];
  static bbram char  new_gw[25];
  static bbram char  new_ns[25];

  #define TCPCONFIG 						102
  #define PORT 								502													// TCP port to use for MODBUS/TCP processing
  #define TCP_BUF_SIZE						512													// TCP buffer size, max MODBUS/TCP packet size is 256 bytes
/*******************************************************************************
	DEFINICION SOCKET ETHERNET
*******************************************************************************/

  #use "dcrtcp.lib"																					// Load TCP/IP Library
  #use MTCPS.LIB																						// Load MODBUS TCP Library
/*******************************************************************************
	DEFINICION CAPACIDAD PUERTOS SERIALES
*******************************************************************************/

  #define CINBUFSIZE  255
  #define COUTBUFSIZE 255
  #define EINBUFSIZE  255
  #define EOUTBUFSIZE 255
  #define FINBUFSIZE  255
  #define FOUTBUFSIZE 255

  unsigned char		buf_tempo_microSD[512];
///////////////////////////////////////////////////////////////////////////////
// CONFIGURACION             M O D B U S   G E N E R A L
///////////////////////////////////////////////////////////////////////////////

  #define		No_sector_datos_CONF			52601
  #define		tiempo_barrido_utrs	5 															// Tiempo de adquisicion de datos a UTR's

  #define		No_data_analogicos	5
  //#define		No_Med_Insta			5

  #define		Fuera_Rango_Bajo		0
  #define		Dentro_Rango			1
  #define		Fuera_Rango_Alto		2

  #define		Fuera_limite_Bajo		0
  #define		Dentro_Limite			1
  #define		Fuera_Limite_Alto		2

  #define		Dir_Start_table_MB	100

  #define		addr_start_table_modbus			100
  #define		addr_start_table_confi		  2200

  int				flag_new_config;
  unsigned char	flag_inicio;

  #define		cantidad					29
/**************************************************************************/
  //  Set a default of declaring all local variables "auto" (on stack)
  #class auto

  // include BLxS2xx series library
  #use "BLxS2xx.lib"
/*******************************************************************************
   CONFIGURACION DE VARIABLES GENERALES
*******************************************************************************/
  union
  {                                                                       // UNION conversion de tipo datos
  		float				conver_float;
        unsigned long  conver_long;
        unsigned int   conver_int[2];
        unsigned char  conver_byte[4];
  } conver;

  union{
  	double to_float64b;
  	unsigned char to_byte[8];
  	unsigned int to_uint16[4];
    long to_long[2];
    int to_int16[4];
  } dato64b;

  union{
    unsigned int to_uint16;
    int to_int16;
    unsigned char to_byte[2];
  }dato16b;

  union{
    unsigned int to_uint16[2];
    int to_int16[2];
    unsigned char to_byte[4];
    unsigned long to_ulong;
  }dato32b;
  float flotanteaux;
  char signoD;
  long exponenteD,mantisaD;
  char bufaux[8];
  unsigned int bufauxint[4];
  unsigned int lerma_IDmodbus;
  unsigned int lerma_float_error;
  unsigned int lerma_orden_flujo;
  unsigned char lerma_in_range;

  unsigned char in_range(float number, float min, float max){
    unsigned char resultado=0;
    if(number<=10000000000 && number>=-10000000000){
      if(number>min && number<max)
        resultado= 0x01;
      else if(number<=min && number>=max)
        resultado= 0xFF;
      else if(number<=min && number<=max)
        resultado= 0x00;
      else
        resultado= 0x02;
    }else{
      resultado= 0xFF;
    }
    return resultado;
  }
  float ajustaEnRango(float number, float min, float max){
    float resultado=0;
    char Rango=in_range(number,min,max);
    if(Rango==1){
      resultado= number;
    }else if(Rango==0){
      resultado= min;
    }else{
      resultado= max;
    }
    return resultado;
  }

  const unsigned int bin[8] = { {0x01},{0x02},{0x04},{0x08},{0x10},{0x20},{0x40},{0x80} };
/*******************************************************************************
	DEFINICION FUNCIONES PROTOTIPO
*******************************************************************************/
/*******************************************************************************
	CONFIGURACION             CARACTERISTICAS I/0 MODBUS TCP/IP
*******************************************************************************/

  #define MY_SLAVE_ADDR		1																	// MODBUS Slave Address

  #define MAX_0X_COILS 		16																	// Coils 0-15 control digital outputs 0-15, Coils 16-22 control the LEDS DS1-DS7, Coil 23 controls LCD backlight
  unsigned my0XCoils[(MAX_0X_COILS / 16)+1];
  #define MAX_1X_DISCRETES 	16																	// Discretes 0-23 report the state of digital inputs 0-23
  unsigned my1XDiscretes[(MAX_1X_DISCRETES / 16)+1];
  #define MAX_3X_INREGS 		60																	// Input Registers binary
  unsigned my3XInRegs[MAX_3X_INREGS];
  #define MAX_4X_REGS 			2500																// Holding Registerss 0-3 control the Analog Outputs 0-3, Holding Registers 4-7 are general purpose memory mapped registers
  unsigned my4XRegs[MAX_4X_REGS];																//	Holding Registers 8-27 are used to display ascii characters on the last line of the LCD display

  #define REG_AMOUNT 	MAX_4X_REGS
/*===================================================================
	WAIT_SERIAL
===================================================================*/

  void wait_485(long sd){
    unsigned long t1;
  	t1=MS_TIMER;
  	while(t1+sd>MS_TIMER){}
  }
//============================================================================*/
//  PUERTO    D E F I N I C I O N E S       G E N E R A L E S        D R I V E R       M O D B U S      *****   DATA LINK    *****   PUERTO C
//  PARA UTR'S INALAMBRICAS
//============================================================================*/

  #define  MM_BRTO_gate               700                                            // allow long (500 ms) query/response delay 500 val ant
  #define  MM_ERTO_gate               10                                            	// set short (1 mS) inter-character delay 200 val ant
  #use     "MMZ_rab_gate.LIB"
  #define  BCAST_ADDR_gate            0                                               // broadcast address
  #define  MODBUS_BAUDRATE_gate       9600                                            // Defines the Baudrate to use
  #define  MODBUS_PROTOCOL_gate       1                                               // Defines the Protocol.  0 = ASCII, 1 = RTU
  #define  MODBUS_SERIAL_gate         1                                               // Defines the Serial port type. 0 = RS485, 1 = RS232
//============================================================================
//         MODBUS FUNCTION          MASTER   MODBUS
//============================================================================

  nodebug
  void  mmCrx_gate(void){
     // Make sure all of the data has been sent by;
     // 1.) checking the write buffer for any bytes left
     // 2.) checking the status of the Write interrupt transmit bit (2).
     // 3.) checking the status of the Write interrupt data bit (3)
     while (serCwrUsed() || BitRdPortI(SCSR,2) || BitRdPortI(SCSR,3));
     // turn off the transmitter
     // Since we echo what we send, flush the read buffer, so that you are
     // ready for the next packet.
     //wait_485(2);                                                             // 10 mseg
     serCrdFlush();
     ser485Rx();
  }

  nodebug
  void mmCtx_gate(){
     ser485Tx();
  }

  nodebug
  int mmCopen_gate(unsigned long qBaud){
     if (serCopen(qBaud)){
        serCdatabits(PARAM_8BIT);
        return 1;
     }
     return 0;
  }
/*============================================================================*/
//  PUERTO    D E F I N I C I O N E S       G E N E R A L E S        D R I V E R       M O D B U S      *****   DATA LINK    *****   PUERTO E
//  PARA UTR'S INALAMBRICAS
/*============================================================================*/
  #define  MM_BRTO_alter               700                                             // allow long (500 ms) query/response delay 500 val ant
  #define  MM_ERTO_alter               10                                            	// set short (1 mS) inter-character delay 200 val ant
  #use     "MMZ_rab_alter.LIB"
  #define  BCAST_ADDR_alter            0                                               // broadcast address
  #define  MODBUS_BAUDRATE_alter       9600                                            // Defines the Baudrate to use
  #define  MODBUS_PROTOCOL_alter       1                                               // Defines the Protocol.  0 = ASCII, 1 = RTU
  #define  MODBUS_SERIAL_alter         1                                               // Defines the Serial port type. 0 = RS485, 1 = RS232
/*============================================================================*\
         MODBUS FUNCTION          MASTER   MODBUS
\*============================================================================*/

  nodebug
  void  mmErx_alter(void){
  }

  nodebug
  void mmEtx_alter(){
  }

  nodebug
  int mmEopen(unsigned long qBaud){
     if (serEopen(qBaud))
     {
        serEdatabits(PARAM_8BIT);
        return 1;
     }
     return 0;
  }
//============================================================================
//  PUERTO    D E F I N I C I O N E S       G E N E R A L E S        D R I V E R       M O D B U S      *****   DATA LINK    *****   PUERTO F
//  PARA UTR'S INALAMBRICAS
//============================================================================

  #define  MM_BRTO_clon               700                                             // allow long (500 ms) query/response delay 500 val ant
  #define  MM_ERTO_clon               10                                               // set short (1 mS) inter-character delay 200 val ant
  #use     "MMZ_rab_clon.LIB"
  #define  BCAST_ADDR_clon            0                                               // broadcast address
  #define  MODBUS_BAUDRATE_clon       9600                                            // Defines the Baudrate to use
  #define  MODBUS_PROTOCOL_clon       1                                               // Defines the Protocol.  0 = ASCII, 1 = RTU
  #define  MODBUS_SERIAL_clon         1                                               // Defines the Serial port type. 0 = RS485, 1 = RS232
//============================================================================
//         MODBUS FUNCTION          MASTER   MODBUS
//============================================================================

  nodebug
  void  mmFrx_clon(void){
  }

  nodebug
  void mmFtx_clon(){
  }

  nodebug
  int mmFopen(unsigned long qBaud){
     if (serFopen(qBaud)){
        serFdatabits(PARAM_8BIT);
        return 1;
     }
     return 0;
  }
/*******************************************************************************
   FUNCION READ DATOS DE EXTRUCTURAS EN MICRO SD   > 512 Bytes
*******************************************************************************/

  void read_micro_SD(unsigned int No_pagina, unsigned char* buffer, unsigned int No_bytes)
  {
    long pagenum;
    int  rc;

     pagenum=(long)No_pagina;

  	if(No_bytes <= 512)																			// Es menor de 512 Bytes
     {
  	   if (rc = sdspi_read_sector(dev, pagenum, buffer))
  	   {
  	      PRINTFDEBUG("\nSD read error (%d): %ls\n", rc, strerror(rc));
  	   }
     }
     else
     {
        if (rc = sdspi_read_sector(dev, pagenum, buffer))								// Graba datos en micro SD
        {
           PRINTFDEBUG("\nSD write error (%d): %ls\n", rc, strerror(rc));
        }

     	buffer += 512;
        pagenum++;																					// COPIA LOS PRIMEROS 512 BYTES

        if (rc = sdspi_read_sector(dev, pagenum, buffer))
  	   {
  	      PRINTFDEBUG("\nSD read error (%d): %ls\n", rc, strerror(rc));
  	   }
     }
  }
/*******************************************************************************
   FUNCION WRITE DATOS DE EXTRUCTURAS EN MICRO SD   > 512 Bytes
*******************************************************************************/

  void write_micro_SD(unsigned int No_pagina, unsigned char* buffer, unsigned int No_bytes)
  {
    long pagenum;
    int  rc;

       pagenum=(long)No_pagina;

    	if(No_bytes <= 512)																			// Es menor de 512 Bytes
       {
          memset (flash_buf, 0x00, 512);														// Clear buffer
          memcpy(flash_buf,buffer,No_bytes);													// Copia datos a buffer
          if (rc = sdspi_write_sector(dev, pagenum, flash_buf))							// Graba datos en micro SD
          {
             PRINTFDEBUG("\nSD write error (%d): %ls\n", rc, strerror(rc));
          }
       }
       else
       {
          memset (flash_buf, 0x00, 512);														// Clear buffer
          memcpy(flash_buf,buffer,512);															// Copia datos a buffer
          if (rc = sdspi_write_sector(dev, pagenum, flash_buf))							// Graba datos en micro SD
          {
             PRINTFDEBUG("\nSD write error (%d): %ls\n", rc, strerror(rc));
          }

          pagenum++;
          buffer += 512;                                                          // COPIA EL RESTO

          memset (flash_buf, 0x00, 512);														// Clear buffer
          memcpy(flash_buf,buffer,No_bytes-512);												// Copia datos a buffer
          if (rc = sdspi_write_sector(dev, pagenum, flash_buf))							// Graba datos en micro SD
          {
             PRINTFDEBUG("\nSD write error (%d): %ls\n", rc, strerror(rc));
          }
       }
  }
/*******************************************************************************
   FUNCION LIMPIEZA DE DIRECTORIOS, TABLAS Y CARGA CONFIGURACION CON VALORES POR DEFAULT
*******************************************************************************/

  void confi_default(void){
    unsigned int i;
    memset(CONF_ANA, 0x00, sizeof(CONF_ANA));												// Inicializa CONFIGURACION ANALOGICA  0x00
    memset(buf_tempo_microSD, 0x00, sizeof(buf_tempo_microSD));						// buf_tempo_microSD  0x00

    for(i=0; i<No_data_analogicos; i++){
        CONF_ANA[i].K1_data    = 0;															// Exponente
  	  	CONF_ANA[i].K2_data    = 6.99;														// Multiplicador
     	CONF_ANA[i].K3_data    = 0;															// Offset
        CONF_ANA[i].Lim_H_data = 75;															// Limite Alto 75%
        CONF_ANA[i].Lim_L_data = 25;															// Limite Bajo 25%
        CONF_ANA[i].Reser_data = 1;															// Reserva 0
  	}

    memcpy(buf_tempo_microSD,(char*)CONF_ANA,sizeof(CONF_ANA));
    write_micro_SD(No_sector_datos_CONF,(char*)buf_tempo_microSD, sizeof(buf_tempo_microSD));
  }
//****************************************************
//		Manda pulso digital con un ancho de 1 seg.
//****************************************************

  cofunc COF_Genera_Pulso_Digi(unsigned int No_Salida){
     digOut(No_Salida, 0);
     waitfor(DelayMs(1000));
     digOut(No_Salida, 1);
  }
//****************************************************
//    Lee ESTADO PERILLAS     AUTO, MANUAL  OFF
//****************************************************
  unsigned int read_status_Perilla(unsigned int in_dere, unsigned int in_izqui )
  {
    int   Ent_loc;
    int   Ent_remo;

     Ent_loc  = digIn(in_dere);                                                 // STATUS PERILLA
     Ent_remo = digIn(in_izqui);

     if( (Ent_loc == 1) && (Ent_remo == 1) )
     {
        return  0;                                                              // OFF
     }
     else
     {
        if( Ent_loc == 0 )                                                      // MANUAL
        {
           return 1;                                                            // REMOTO
        }
        else
        {
           return 2;                                                            // AUTO   LOCAL
        }
     }
  }
//****************************************************
//    Lee ESTADO PERILLAS     AUTO, MANUAL  OFF
//****************************************************
  unsigned int read_status_Bomba(unsigned int in_estado, unsigned int in_falla )
  {

     if( digIn(in_falla) == 0 )
     {
        return  3;                                                              // OFF
     }
     else
     {
        if( digIn(in_estado) == 0 )                                             // MANUAL
        {
           return 1;                                                            // Arrancada
        }
        else
        {
           return 2;                                                            // Parada
        }
     }
  }
/*===================================================================
   FUNCION ALMACENAR DATOS DE TIEMPO EN TABLA ( my4XRegs )
===================================================================*/

  void Almacena_Datos_Tiempo(int *apunta_t)
  {
    struct tm      rtc;

  	mktm(&rtc, read_rtc());

  	*apunta_t  = rtc.tm_sec;
  	apunta_t++;

     *apunta_t  = rtc.tm_min;
  	apunta_t++;

     *apunta_t  = rtc.tm_hour;
  	apunta_t++;

     *apunta_t  = rtc.tm_mday;
  	apunta_t++;

     *apunta_t  = rtc.tm_mon;
  	apunta_t++;

     *apunta_t  = rtc.tm_year + 1900;
  }
//------------------------------------------------------------------------------
//
//	SECCION CODIGO DE LA LIBRERIA  MTCPS.LIB    LECTURA DE REGISTROS HOLDING  FUNCION 03
//
//------------------------------------------------------------------------------

  int msRead(unsigned wReg, unsigned *pwValue)
  {
  	if( (wReg < 0) || (wReg > i4XRegs-1))
  		return MS_BADADDR;

     /*
     if( wReg == 100 )
     {
     	//actividad_modbus_RD = 0x01;
        tiempo_restante = tiempo_max_final;
     }
     */

  	*pwValue = *(p4XRegs+wReg);
  	return 0;
  }
/*===================================================================
   FUNCION MANEJA EL ESTADO DE LA COMUNICACION FRONTAL
===================================================================*/

  void drive_leds_comunicacion(unsigned estado)
  {
   switch (estado)
   {
     case 0:
          	digOut(sal_E1,0);																		// Onda Azul Punto Rojo 			0
           digOut(sal_E2,0);
           digOut(sal_E3,0);
     break;

     case 1:
           digOut(sal_E1,0);																		// Onda Roja Punto Rojo 			1
           digOut(sal_E2,1);
           digOut(sal_E3,0);
     break;

     case 2:
           digOut(sal_E1,1);																		// Onda Roja Punto Rojo 			2
           digOut(sal_E2,0);
           digOut(sal_E3,0);
     break;

     case 3:
           digOut(sal_E1,1);																		// Onda Verde Punto Verde    		3
           digOut(sal_E2,1);
           digOut(sal_E3,1);
     break;

     default: ;
   }
  }
/*===================================================================
	PROCESO DE ESCRITURA DE MEDICIONES INSTANTANEAS AL HMI
===================================================================*/

  cofunc int COF_WR_INSTA_HMI_CC(void){
    int				status_resp;
    int				k,ii;

  		waitfor(DelayMs(100));
        //                 int mmPresetRegs_gate(unsigned wAddr,       unsigned wReg, unsigned wCount,           void *pRegs)
        waitfor (status_resp = mmPresetRegs_gate(     addr_HMI,   Dir_Start_table_MB,        cantidad, (int *)&my4XRegs[100]));

        if(status_resp==0xffff)
        {
           k=0;
           PRINTFDEBUG("WR_HMI OK\n");
        }
        else
        {
        	//error_MB_C++;
           PRINTFDEBUG("error_WR_Insta_HMI\n");
        }

        conta_secuencial++;
        my4XRegs[addr_reg_conta_sec_hmi] = conta_secuencial;


        waitfor( DelayMs(100) );
        //                 int mmPresetRegs_gate(unsigned wAddr,               unsigned wReg,                              unsigned wCount,void *pRegs)
        waitfor (status_resp = mmPresetRegs_gate(     addr_HMI,       addr_reg_conta_sec_hmi,               3,(int *)&my4XRegs[addr_reg_conta_sec_hmi]));

        if(status_resp==0xffff)
        {
           k=0;
        }
        else
        {
        	//error_MB_C++;
           PRINTFDEBUG("error_WR_Counter_HMI\n");
        }

        //////////////////////////////////////////////////			Display Valores Crudos

        waitfor( DelayMs(100) );
        //                 int mmPresetRegs_gate(unsigned wAddr,          unsigned wReg,           unsigned wCount,                               void *pRegs)
        waitfor (status_resp = mmPresetRegs_gate(     addr_HMI,                    2400,        (No_med_corrien)*3,                    (int *)&my4XRegs[2400]));

        if(status_resp==0xffff)
        {
           k=0;

           			for(ii=2400; ii<=2406; ii++)
  	               {
  	                  PRINTFDEBUG("%x,",my4XRegs[ii]);
  	               }
  	               PRINTFDEBUG("\n");
        }
        else
        {
        	//error_MB_C++;
           PRINTFDEBUG("error_WR_Counter_HMI\n");
        }

     return(status_resp);
  }
/*===================================================================
	PROCESO DE ESCRITURA CONFIGURACION DATOS A TRAVES HMI
===================================================================*/

  cofunc int COF_WR_DATOS_CONFI_HMI_CC(void)
  {
    int				status_resp;
    int				k;
    int				buffersito[2];

        //  Manda set configuracion de escala y limite de las variables
        waitfor(DelayMs(100));
        //                 int mmPresetRegs_gate(unsigned wAddr,               unsigned wReg,      unsigned wCount,               void *pRegs)
        waitfor (status_resp = mmPresetRegs_gate(     addr_HMI ,  addr_start_table_confi + 4, sizeof(CONF_ANA) / 2,(int*)&CONF_ANA[0].K1_data));

        if(status_resp==0xffff)
        {
        	waitfor( DelayMs(100) );

           //  Manda set Reg=200 con valor 1

        	buffersito[0]=1;
        	//                 int mmPresetRegs_gate(unsigned wAddr,             unsigned wReg,      unsigned wCount,               void *pRegs)
        	waitfor (status_resp = mmPresetRegs_gate(     addr_HMI ,		addr_start_table_confi,                    1,                buffersito));

        	if(status_resp==0xffff)
        	{
           	k=0;
        	}
        	else
        	{
        		//error_MB_C++;
           	PRINTFDEBUG("error_WR set flag CONFI_2\n");
        	}
        }
        else
        {
        	//error_MB_C++;
           PRINTFDEBUG("error_WR set flag CONFI_1\n");
        }

     return(status_resp);
  }
/*===================================================================
	PROCESO DE ESCRITURA DE MEDICIONES INSTANTANEAS AL HMI
===================================================================*/

  cofunc int COF_RD_FLAG_CAMBIO_HMI_CC(void){
    int				status_resp;
    int				buffer_tempo[255];

  		waitfor(DelayMs(100));
        //              it  = mmRead_gate(unsigned wAddr ,               unsigned wReg, unsigned wCount , void *pwRegs )
        waitfor (status_resp = mmRead_gate(     addr_HMI ,   addr_start_table_confi +1,                1, buffer_tempo ));            // Envia CMD RD 1 o varios Registros

        if(status_resp==0xffff)
        {
           flag_new_config = buffer_tempo[0];
        }
        else
        {
        	//error_MB_C++;
           PRINTFDEBUG("error RD Flag Cambio\n");
        }
     return(status_resp);
  }
/*===================================================================
	PROCESO DE LECTURA Y ALMACENAMIENTO DATOS CONFIGURACION EN MEMORIA FLASH TABBIT
===================================================================*/

  cofunc int COF_RD_STORE_DATOS_CONF_DATOS_HMI_CC(void)
  {
    int				status_resp;
    int				buffer_tempo[255];

        waitfor(DelayMs(100));
              //         it  = mmRead_gate(unsigned wAddr,             unsigned wReg,      unsigned wCount , void *pwRegs )
        waitfor (status_resp = mmRead_gate(      addr_HMI, addr_start_table_confi + 4, sizeof(CONF_ANA) / 2, buffer_tempo ));      // Envia CMD RD 1 o varios Registros

        if(status_resp==0xffff)
        {
     			write_micro_SD(No_sector_datos_CONF,(char*)buffer_tempo, sizeof(CONF_ANA));
              memcpy((char*)CONF_ANA,(char*)buffer_tempo,sizeof(CONF_ANA));
        }
        else
        {
        	//error_MB_C++;
           PRINTFDEBUG("error_Clon RD_STORE_DATOS CONFI C=%d  ",status_resp);
        }

     return(status_resp);
  }
/*===================================================================
	PROCESO DE RESET FLAG_CAMBIO DATOS CONFIGURACION  HMI
===================================================================*/

  cofunc int COF_RESET_FLAG_DATOS_CONFI_HMI_CC(void){
    int				status_resp;
    int				k;
    int				buffer_tempo[255];

  		waitfor(DelayMs(100));
        buffer_tempo[0] = 0;
        //                 int mmPresetRegs_gate(unsigned wAddr,             unsigned wReg,      unsigned wCount,               void *pRegs)
        waitfor (status_resp = mmPresetRegs_gate(     addr_HMI ,addr_start_table_confi + 1,                    1,               buffer_tempo));

        if(status_resp==0xffff)
        {
        	k=0;
        }
        else
        {
        	//error_MB_C++;
           PRINTFDEBUG("error_RESET FLAG CONFI\n",status_resp);
        }

     return(status_resp);
  }
/*===================================================================
=====================================================================
=====================================================================

				PROGRAMA      PRINCIPAL

=====================================================================
=====================================================================
===================================================================*/

main()
{
  struct tm		rtc;
  int 	tempo_int;
  int 	channel;
  unsigned long	new_seg;
  unsigned char	conta_seg;
  unsigned char  i,j;
  			int	error,z,ii;
  unsigned int	jj;
  unsigned long	t1;
  unsigned int	new_hora;
  unsigned int	alma_tempo_int;

  unsigned int	input_local;
  unsigned int	input_remoto;
  unsigned int	cmd_control;

  unsigned char	flag_inicio_version;
  unsigned char	flag_inicio_control;

  unsigned char	flag_inicio_control_A;														// flag Control HMI
  unsigned char	flag_inicio_control_B;                                         // flag Control Scairlink

  unsigned char	nada;
  unsigned int	No_sitio;
  unsigned int	Bomba_No;

  float				current,procen_nivel_float;
  float				voltage;

  int 				rc;
  float				float_tempo;

  	unsigned long	totalizador;
	unsigned int   in_anterior;
	unsigned int   in_actual;
	unsigned int   in_cambio;

   unsigned int	conta_errores;



  /*===================================================================
  	Inicializacion del MODULO  BL4S200
  ===================================================================*/

     brdInit();																						// Required for controllers
     serMode(0);
  /*===================================================================
  	Inicializacion Salidas Digitales
  ===================================================================*/

  	for(channel = 0; channel < 8; ++channel)												// Set output to be general digital output
     {
        setDigOut(channel, 1);																	// Set output to be off                                                                              //channels[channel] = 1
     }
  /*===================================================================
  	Inicializacion Entradas Digitales
  ===================================================================*/

     for(channel = 8; channel < 32; ++channel)												// Set configurable I/O 0-15 to be general digital inputs
  	 {
  		setDigIn(channel);
     }

     setDigOut(dio_reset_router_cel, 1);
     setDigOut(dio_reset_scairlink, 1);
     setDigOut(dio_reset_rabbit,1);

     setDigOut(sal_E1,1);
     setDigOut(sal_E2,1);
     setDigOut(sal_E3,1);
  /*===================================================================
  	Inicializacion de Entradas Analogicas
  ===================================================================*/

  	 anaInConfig(0, mAMP_MODE);                                                 // Set Configuracion Entrada Analogica 0     medicion de 4 a 20 ma
     anaInConfig(1, mAMP_MODE);
     anaInConfig(2, mAMP_MODE);
     anaInConfig(3, mAMP_MODE);
     anaInConfig(4, SE0_MODE);																	// Set Medicion Voltaje
     anaInConfig(6, SE0_MODE);																	// Set Medicion Voltaje
     anaInConfig(7, SE0_MODE);																	// Set Medicion Voltaje
  //------------------------------------------------------------------------------
  //				INICIALIZACION DE VARIABLES
  //------------------------------------------------------------------------------

  	flag_inicio = 1;

    sprintf(new_ip,"192.168.1.49");
  	sprintf(new_nm,"255.255.255.0");
  	sprintf(new_gw,"192.168.1.50");
  	sprintf(new_ns,"192.168.1.81");
  	//Para pruebas locales
    /*sprintf(new_ip,"10.10.10.19");
    sprintf(new_nm,"255.255.255.0");
    sprintf(new_gw,"10.10.10.1");
    sprintf(new_ns,"10.10.10.18");*/
  /*******************************************************************************
     SECCION INICIALIZACION SOCKET TCP/IP
  *******************************************************************************/

     sock_init();

     ifconfig(IF_ETH0, IFS_DOWN, IFS_END);													// bring down interface
     tcp_tick(NULL);
     while(ifpending(IF_ETH0) % 2)          												// waitfor interface to close
  		tcp_tick(NULL);

     ifconfig(IF_ETH0,
           IFS_DOWN,
           IFS_IPADDR,         aton(new_ip),
           IFS_NETMASK,        aton(new_nm),
           IFS_ROUTER_SET,     aton(new_gw),                     					// delete all GW and add this one
           IFS_NAMESERVER_SET, aton(new_ns),                     					// delete all NS and add this one
           IFS_UP,
     IFS_END);
  /*******************************************************************************
  	SECCION INICIALIZACION DE ENTRADAS BINARIAS Y REGISTROS MODBUS
  *******************************************************************************/

  	//    Initialize local MODBUS Registers and Coils
  	// 	This is the user defined data memory which is used to pass data back and forth with the MODBUS/TCP library routines
  	//		The users program should define Coils, Discretes, Input Registers, and Holding Registers arrays large enough to
  	//		support the MODBUS/TCP references required for the application and initialize their contents.

     memset(&my3XInRegs, 0x00    , MAX_3X_INREGS);
     memset(&my4XRegs  , 0x00    , sizeof(my4XRegs));

  	// Intialize MODBUS TCP protocol Engine by passing pointers and array sizes to the library function mstInit()

  	mstInit(MY_SLAVE_ADDR, my0XCoils, MAX_0X_COILS,
  			my1XDiscretes, MAX_1X_DISCRETES,
  			my3XInRegs, MAX_3X_INREGS,
  			my4XRegs, MAX_4X_REGS);

  	//    Cooperative multitasking example using costates
  	//		This cooperative multitasking examples illustrates the use of costates to perform various tasks
  	//		Tasks are included to handle keypad I/O, update and scan digital and analog I/O, refresh the LEDS, and refresh the LCD display
  	//		The msRun() MODBUS/TCP Library function is included as a seperate costate task, this allows the handling of all
  	//		MODBUS/TCP related tasks to be cleanly seperated from the balance of the application functionality
  /*******************************************************************************
  	SECCION INICIALIZACION DATOS Y COMANDOS A ENVIAR PARA LA ADQ MODBUS TCP/IP
  *******************************************************************************/

    wMSAddr=1;
  /*===================================================================
  	Inicializacion de puertos seriales
  ===================================================================*/

  	 mmrCinit_gate(9600);                                                       // Velocidad puerto Master hacia HMI + SCAIRLINK
     mmrEinit_alter(9600);                                                       // Velocidad puerto Master hacia HMI + SCAIRLINK
     mmrFinit_clon(9600);                                                      // Velocidad puerto Master hacia HMI + SCAIRLINK
  /*===================================================================
  	Inicializa Driver micro SD
  ===================================================================*/

     dev = &SD[0];
  	 if (rc = sdspi_initDevice(0, &SD_dev0)){
     	PRINTFDEBUG("Flash init failed (%d): %ls\n\n", rc, strerror(rc));
        //exit(rc);
     }else{
     	PRINTFDEBUG("Flash init OK\n");
        PRINTFDEBUG("# of blocks: %ld\n", dev->sectors);
        PRINTFDEBUG("Size of block: 512\n\n");
     }

     //confi_default();//para guardar datos de configuracion en la SD
  /*===================================================================
  	Recupera datos   NAMES ANALOGIC  del micro SD
  ===================================================================*/

  	memset(my4XRegs, 0x00, sizeof(my4XRegs));
    memset(CONF_ANA  , 0x00    , sizeof(CONF_ANA));
  	read_micro_SD(No_sector_datos_CONF, buf_tempo_microSD, sizeof(buf_tempo_microSD));		// Load NAMES_ANALOGIC
    memcpy((char*)CONF_ANA,buf_tempo_microSD, sizeof(CONF_ANA));
    memcpy((char*)&my4XRegs[2204],CONF_ANA, sizeof(CONF_ANA));

    my4XRegs[2200] = 0;
    my4XRegs[2201] = 0;
    my4XRegs[2202] = 0;
    my4XRegs[2203] = 0;

    for(i=0; i<No_data_analogicos; i++){
       	PRINTFDEBUG("K1=%f\n",CONF_ANA[i].K1_data);												// Exponente
  	   PRINTFDEBUG("K2=%f\n",CONF_ANA[i].K2_data);												// Multiplicador
     	PRINTFDEBUG("K3=%f\n",CONF_ANA[i].K3_data);												// Offset
        PRINTFDEBUG("LH=%f\n",CONF_ANA[i].Lim_H_data);                               // Limite H
        PRINTFDEBUG("LH=%f\n",CONF_ANA[i].Lim_L_data);                               // Limite L
        PRINTFDEBUG("LH=%f\n",CONF_ANA[i].Reser_data);                               // Reservado
  	}

    PRINTFDEBUG("\n");
    PRINTFDEBUG("\n");
    //memset(promedio  , 0x00    , sizeof(promedio));
  /*===================================================================
  	Inicializacion de extructuras, variables etc.
  ===================================================================*/

   memset(tabla_modbus, 0x00, sizeof(tabla_modbus));
  /***************************************************************************************************************

  ***************************************************************************************************************/

   //  F A C T O R E S   C O N F I G U R A C I O N          N I V E L      C A R C A M O
   /*
   CONF_ANA[0].K1_data					= 0.0;
   CONF_ANA[0].K2_data					= 5;
   CONF_ANA[0].K3_data					= 0.0;
   CONF_ANA[0].Lim_L_data				= 1;
   CONF_ANA[0].Lim_H_data				= 4;
   CONF_ANA[0].Reser_data				= 5;

   //  F A C T O R E S   C O N F I G U R A C I O N          P R E S I O N

   CONF_ANA[1].K1_data					= 0.0;
   CONF_ANA[1].K2_data					= 10.0;
   CONF_ANA[1].K3_data					= 0.0;
   CONF_ANA[1].Lim_L_data				= 2;
   CONF_ANA[1].Lim_H_data				= 8;
   CONF_ANA[1].Reser_data				= 10.0;
   */

   mktm(&rtc, read_rtc());
   PRINTFDEBUG("Fecha Iinicio -> %u/%u/%u %u:%u:%u\n\r",rtc.tm_year+1900,rtc.tm_mon,rtc.tm_mday,rtc.tm_hour,rtc.tm_min,rtc.tm_sec);

   //finaliza_tiempo = read_rtc() + tiempo_max_final;

   conver.conver_float = ver_sion;															// Almacena el numero de version
   my4XRegs[addr_reg_mb_version] 		= conver.conver_int[0];
   my4XRegs[addr_reg_mb_version + 1] 	= conver.conver_int[1];

   my4XRegs[W_set_tiempo] = 0x00; 															// Flag de SET TIME MODBUS  = 0 Desabilitado WEB
   my4XRegs[U_set_tiempo] = 0x00; 															// Flag de SET TIME MODBUS  = 0 Desabilitado UTR

   flag_inicio_version		= 1;
   flag_inicio_control 		= 1;
   flag_inicio_control_A	= 1;
   flag_inicio_control_B	= 1;
   cmd_control					= 0;
   cmd_control_hmi			= 0;
   cmd_control_regional		= 0;

   totalizador					= 0;
   my4XRegs[addr_reg_mb_ctrl_regio]       = 0x00;
   my4XRegs[ addr_reg_mb_ctrl_web]			= 0;

   conta_errores									= 0;


   my4XRegs[reset_scairlink]            	= 0;
   my4XRegs[reset_instru]                 = 0;
   my4XRegs[reset_celular]                = 0;
   my4XRegs[reset_rabbit]                 = 0;

	while(1){

		costate{ // Process MODBUS TCP activity
         msRun();
		}

      costate
      {
      	in_actual = digIn( pin_totalizador );															// Status Entrada Totatizador

         in_cambio =	in_actual ^ in_anterior;

         if( in_cambio == 0x0001 && in_actual == 0x00 )
         {
         	totalizador++;
            printf("+%lu\n",totalizador);
         }
         in_anterior = in_actual;
      }

    costate{
      	if(new_seg!=read_rtc()){															// New Second ?
				  new_seg=read_rtc();
          mktm(&rtc, read_rtc());
          ////////////////////////////////////////////////////////////////////////////////////////////////////////////
          conta_seg++;
          if( conta_seg >= 4 ){
              conta_seg = 0;
              //////////////////////////////////////////////////////////////////////////
	            //     AJUSTA TIEMPO RTC SI REG 996 = 1
	            if( my4XRegs[W_set_tiempo] == 0x01 ){                              // Si flag de set_tiempo == 0x01 actualiza tiempo reloj
	               rtc.tm_sec     = my4XRegs[W_segundos];                         // change the time
	               rtc.tm_min     = my4XRegs[W_minutos];
	               rtc.tm_hour    = my4XRegs[W_horas];

	               //tm_setMDY( &rtc, 12, 31, 1999);

	               tm_setMDY( &rtc, my4XRegs[W_mes],my4XRegs[W_dia], my4XRegs[W_año]);

	               tm_wr(&rtc);                                                   // set clock
	               t1 = mktime(&rtc);
	               PRINTFDEBUG("Setting date/time to ");
	               PRINTFDEBUG("Fecha -> %u/%u/%u %u:%u:%u\n\r",rtc.tm_year+1900,rtc.tm_mon,rtc.tm_mday,rtc.tm_hour,rtc.tm_min,rtc.tm_sec);
	               my4XRegs[W_set_tiempo] = 0x00;
	            }

               ///////////////////////////////////////////////////////////////////////////////////////////////////
               ////       WR No de Version en HMI  y Borra Flag Control Start/Stop Arrancadores
               ///////////////////////////////////////////////////////////////////////////////////////////////////

              if( flag_inicio_version == 1 ){											// Write No Version Software RABBIT
                  waitfor(DelayMs(100));
                  //                      mmPresetRegs_gate(  unsigned wAddr ,          unsigned wReg,   unsigned wCount,                          void *pwRegs)
                  waitfor (      error = mmPresetRegs_gate(                5,     addr_reg_mb_version,                 2, (int *)&my4XRegs[addr_reg_mb_version]));
                  if( error == 0xffff ){
                  	PRINTFDEBUG("WR OK No_Version\n");
                  	flag_inicio_version = 0;
                  }else{
                  	PRINTFDEBUG("WR Error Version\n");
                  }
	            }

              if( flag_inicio_control_A == 1 ){
               	my4XRegs[addr_reg_mb_ctrl_regio] = 0x00;							// Reset registro control Bombas Scairlink

                  waitfor(DelayMs(100));
                                         // mmPresetReg_gate(unsigned wAddr,          unsigned wReg, unsigned wVal)
                  waitfor (      error = mmPresetReg_gate(                5,  addr_reg_mb_ctrl_hmi,             0));

                  if( error == 0xffff ){
                  	if( my4XRegs[addr_reg_mb_ctrl_hmi] == 0 ){
         	            PRINTFDEBUG("WR OK reset_cmd_control_A\n");
	                     flag_inicio_control_A = 0;
	                  }
                  }else{
                  	PRINTFDEBUG("WR Error CTRL A\n");
                  }
              }

              if( flag_inicio_control_B == 1 ){
                  waitfor(DelayMs(100));
                                         // mmPresetReg_gate(unsigned wAddr,           unsigned wReg, unsigned wVal)
                  waitfor (      error = mmPresetReg_alter(    dir_scairlink,  addr_reg_mb_ctrl_regio,             0));
                  if( error == 0xffff ){
                     PRINTFDEBUG("WR OK reset_cmd_control_B\n");
                     flag_inicio_control_B = 0;
                  }else{
                  	PRINTFDEBUG("WR Error CTRL B\n");
                  }
	            }

              ////////////////////////////////////////////////////////////////////////////////////////////////////////////
              ////			R E A D      D A T A       L O C A L       P O Z O       E N L A C E     S C A I R L I N K
              ////////////////////////////////////////////////////////////////////////////////////////////////////////////

              waitfor(DelayMs(100));
              //                     mmRead_alter(  unsigned wAddr,       unsigned wReg,   unsigned wCount,                       void *pwRegs)       // Write Status Bbas
              waitfor (      error = mmRead_alter(   dir_scairlink,   enlace_scairlink,                 1, (int *)&my4XRegs[enlace_scairlink]));
              drive_leds_comunicacion(my4XRegs[enlace_scairlink]);
              if( error == 0xffff ){
               	if( my4XRegs[enlace_scairlink] >= 2){
                  	my4XRegs[enlace_scairlink] = 1;
                  }else{
                  	my4XRegs[enlace_scairlink] = 0;
                  }
              }else{
                  PRINTFDEBUG("WR RD Enlace Scairlink\n");
                  my4XRegs[enlace_scairlink] = 0;
              }

              waitfor(DelayMs(100));
		          //                      mmPresetRegs_alter(  unsigned wAddr,   unsigned wReg,   unsigned wCount,          void *pwRegs)
		          waitfor (      error = mmPresetRegs_alter(    dir_scairlink,      100,                 22, (int *)&my4XRegs[100]));       // Se escriben 22 regs al esclavo
		          if( error != 0xffff )
		          {
		            printf("Error en ESCRITURA a SLink\n");
		          }
		          else
		          {
		         	  printf("Exito ESCRITURA Slink\n");
		          }
               //////////////////////////////////////////////////////////////////////////////////////////
	            //    R E A D     I N P U T     L E V E L    C A R C A M O    &    P R E S S U R E
	            //////////////////////////////////////////////////////////////////////////////////////////

              for(j=0; j<No_med_corrien; j++){
               	current = anaInmAmps(j);                                          		// Lee entrada del medidor de nivel
                MED_CRUDA[j].valor =  current;
                MED_CRUDA[j].rango =  0x01;

                PRINTFDEBUG("Cur_%u=%.3f\n",j,current);

                if( (current >=3.5) && (current <=4.0) )                             // Ajusta para señales >3.5 ma y <4.0 ma
                {
                     current = 4.0;                                                    // 4.0 ma
                }

                if( (current >=20) && (current <=21.0) )                             // Ajusta para señales >20.0 ma y <21.0 ma
                {
                     current = 20.0;                                                   // 20.0 ma
                }

                if( current >= 4 )                                                   // Esta fuera de Rango Bajo  ?
                {
                 if(current <= 20)                                                 // Esta fuera de Rango Alto  ?
                     {
                     	if(No_max_suma>0){
                     		//PRINTFDEBUG("A ");

	                        procen_nivel_float  = ( ((current-4)/16) );                    // Convierte en Porcentaje el Valor

	                        //PRINTFDEBUG("B ");

	                        for(i=0; i<No_max_suma-1; i++)
	                        {
	                           promedio[j].sumando[i] = promedio[j].sumando[i+1];
	                           //PRINTFDEBUG("C%d",i);
	                        }
	                        //PRINTFDEBUG("D ");

	                        promedio[j].sumando[i] = procen_nivel_float;

	                        //PRINTFDEBUG("E");

	                        valor_promedio = 0;
	                        for(i=0; i<No_max_suma; i++)
	                        {

	                        	valor_promedio += promedio[j].sumando[i]/ No_max_suma;

	                           //PRINTFDEBUG("F%d",i);
	                        }

	                        //PRINTFDEBUG("G",i);
	                        procen_nivel_float = valor_promedio;
	                    }


                        float_tempo        = procen_nivel_float;

                        procen_nivel_float  = procen_nivel_float * CONF_ANA[j].K2_data + CONF_ANA[j].K3_data;  //Convierte en Valores de Ingenieria
                        VAL_ANA[j].index_imagen  = (unsigned int)(float_tempo * 10 * CONF_ANA[j].Reser_data );


                        VAL_ANA[j].valor         = procen_nivel_float;
                        VAL_ANA[j].status_rango  = Dentro_Rango;
                        VAL_ANA[j].status_limite = Dentro_Limite;

                        if( procen_nivel_float >= CONF_ANA[j].Lim_H_data)
                        {
                           VAL_ANA[j].status_limite = Fuera_Limite_Alto;               // Rebaso Limite ALTO
                        }

                        if( procen_nivel_float <= CONF_ANA[j].Lim_L_data)
                        {
                           VAL_ANA[j].status_limite = Fuera_limite_Bajo;               // Rebaso Limite Bajo
                        }
                     }
                     else
                     {
                        VAL_ANA[j].valor         = 0;                                  // Fuera Rango Alto
                        VAL_ANA[j].index_imagen  = 0;
                        VAL_ANA[j].status_rango  = Fuera_Rango_Alto;
                        VAL_ANA[j].status_limite = 0;
                     }
                  }
                  else
                  {
                     VAL_ANA[j].valor         = 0;                                     // Fuera Rango Bajo
                     VAL_ANA[j].index_imagen  = 0;
                     VAL_ANA[j].status_rango  = Fuera_Rango_Bajo;
                     VAL_ANA[j].status_limite = 0;
                  }
                  yield;
	            }

              memcpy((char*)&my4XRegs[100],(char*)&VAL_ANA[0],sizeof(val_analog)*No_med_corrien);
              memcpy((char*)&my4XRegs[2400],(char*)&MED_CRUDA[0],sizeof(val_crudos)*No_med_corrien);
              my4XRegs[ status_bba     ] =  read_status_Bomba(pin_estado_bba, pin_estado_perilla );
              my4XRegs[ status_perilla ] =  read_status_Perilla(pin_perilla_local, pin_perilla_remo );
              //////////////////////////////////////////////////////////////////////////////////////////
	            //    R E A D     V O L T    B A T E R I A
	            //////////////////////////////////////////////////////////////////////////////////////////

              voltage = anaInVolts(7, 0);
              conver.conver_float	= voltage * 2;
              my4XRegs[ volt_bateria + 0]		= conver.conver_int[0];
              my4XRegs[ volt_bateria + 1]		= conver.conver_int[1];
              //////////////////////////////////////////////////////////////////////////////////////////
	            //    R E A D     D I G I T A L    I N P U T
	            //////////////////////////////////////////////////////////////////////////////////////////



              if( digIn(dio_puerta_abierta) == 0 ){
               	my4XRegs[ puerta_abierta ] = 1;
              }else{
               	my4XRegs[ puerta_abierta ] = 0;
              }

              if( digIn(dio_falla_ac) == 0 ){
               	my4XRegs[ falla_ac ] = 1;
              }else{
               	my4XRegs[ falla_ac ] = 0;
              }

              if( digIn(dio_mantenimiento) == 0 ){
               	my4XRegs[ mantenimiento ] = 0;
              }else{
               	my4XRegs[ mantenimiento ] = 1;
              }

              ///////////////////////      R E S E T      D I S P O S I T I V O S


               if( my4XRegs[reset_instru] == 0xaa55 )									// Reset hardware Medidor de Nivel
               {
               	digOut(dio_reset_instru, 0);
                  waitfor( DelayMs(2000) );
                  digOut(dio_reset_instru, 1);
                  my4XRegs[reset_instru] = 0x00;
               }

               if( my4XRegs[reset_celular] == 0xaa55 )
               {
               	digOut(dio_reset_router_cel, 0);
                  waitfor( DelayMs(2000) );
                  digOut(dio_reset_router_cel, 1);
                  my4XRegs[reset_celular] = 0x00;
               }

               if( my4XRegs[reset_rabbit] == 0xaa55 )
               {
               	digOut(dio_reset_rabbit, 0);
                  waitfor( DelayMs(2000) );
                  digOut(dio_reset_rabbit, 1);
                  my4XRegs[reset_rabbit] = 0x00;
               }

               if( my4XRegs[reset_scairlink] == 0xaa55 )
               {
               	digOut(dio_reset_scairlink, 0);
                  waitfor( DelayMs(2000) );
                  digOut(dio_reset_scairlink, 1);
                  my4XRegs[reset_scairlink] = 0x00;
               }


              ///////////////////////////////////////////////////////////////////////////////////////////////////
              ////       C O N T R O L      B O M B A S       H M I
              ///////////////////////////////////////////////////////////////////////////////////////////////////

              if( flag_inicio_control_A == 0 ){
               	waitfor(DelayMs(100));
	               //               it  = mmRead_gate(  unsigned wAddr,         unsigned wReg,   unsigned wCount,             void *pwRegs)
	               waitfor (      error = mmRead_gate(               5,  addr_reg_mb_ctrl_hmi,                 1,  (int *)&cmd_control_hmi));           // Read Registros
	               if( error == 0xffff ){
                  	if( cmd_control_hmi != 0){
	                     waitfor(DelayMs(100));
	                                         // mmPresetReg_gate(  unsigned wAddr,          unsigned wReg, unsigned wVal)
	                     waitfor (      error = mmPresetReg_gate(              5 ,   addr_reg_mb_ctrl_hmi,            0));
	                     if( error == 0xffff ){
	                        nada = 0x00;
	                     }else{
	                        cmd_control_hmi = 0x00;
	                     }
                     }
                  }else{
                  	cmd_control_hmi = 0x00;
                  }
               }

               if( flag_inicio_control_B == 0 ){
                  waitfor(DelayMs(100));
                  //               it  = mmRead_alter(    unsigned wAddr,           unsigned wReg,   unsigned wCount,                  void *pwRegs)
	               waitfor (      error = mmRead_alter(     dir_scairlink,  addr_reg_mb_ctrl_regio,                 1,  (int *)&cmd_control_regional));           // Read Registros
	               if( error == 0xffff ){
                  	if( cmd_control_regional != 0 ){
	                     waitfor(DelayMs(100));
	                                         // mmPresetReg_alter(   unsigned wAddr,            unsigned wReg, unsigned wVal)
	                     waitfor (      error = mmPresetReg_alter(    dir_scairlink,   addr_reg_mb_ctrl_regio,            0));

	                     if( error == 0xffff ){
	                        PRINTFDEBUG("--%x\n",cmd_control_regional);
	                        nada = 0x00;
	                     }else{
	                        cmd_control_regional = 0x00;
	                     }
                     }
                  }else{
                  	cmd_control_regional = 0x00;
                  }
               }

               if( cmd_control_hmi !=0 ){
                  cmd_control = cmd_control_hmi;
               }else{
                  if( cmd_control_regional != 0 ){
                     cmd_control = cmd_control_regional;
                     cmd_control_regional = 0x00;
                  }else{
                  	if( my4XRegs[ addr_reg_mb_ctrl_web] != 0 ){
	                      cmd_control                        = my4XRegs[ addr_reg_mb_ctrl_web];
	                      my4XRegs[ addr_reg_mb_ctrl_web]    = 0;
                     }
                  }
               }
               //////////////////////////////////////////////////////////////////////////
               //     PROCESA CONTROL REGIONAL
               //////////////////////////////////////////////////////////////////////////

               if( (cmd_control != 0) )   // Hay un comando control y perilla en automatico
                  //if( (cmd_control != 0) && (my4XRegs[dir_modo_control] == 1) )   // Hay un comando control y perilla en automatico
               {
                  if( my4XRegs[status_perilla] == 1 ){   // Hay un comando control y perilla en automatico
                    PRINTFDEBUG("Reg cmd_control_regional = %x\n",cmd_control);

	                  No_sitio = cmd_control & 0xFF00;
	                  No_sitio = No_sitio >> 8;

	                  if( No_sitio == Numero_sitio )
	                  {
	                     Bomba_No = cmd_control & 0x00FF;

	                     switch (Bomba_No)
	                     {
	                       case 0x0001:
	                        No_salida      = 0;
	                        ejecuta_mando  = 0x01;
	                        PRINTFDEBUG("START Bba 1\n");
	                       break;

	                       case 0x0002:
	                        No_salida      = 1;
	                        ejecuta_mando  = 0x01;
	                        PRINTFDEBUG("STOP Bba 1\n");
	                       break;

	                       default:
	                        nada = 0;                        //my4XRegs[266] = 0x0000;
	                        ejecuta_mando  = 0x00;
	                        PRINTFDEBUG("DEFAULT Bba %u\n",Bomba_No);
	                     }
	                  }
                  }
                  cmd_control                            = 0x00;
                  cmd_control_hmi                        = 0x00;
                  my4XRegs[addr_reg_mb_ctrl_regio]       = 0x00;
               }
               ///////////////////////      S E T       T I E M P O 				// PONE  TIEMPO

               apunta_tiempo = (int *)&my4XRegs[Reg_Tiempo];
         		   Almacena_Datos_Tiempo( apunta_tiempo );

               if(flag_inicio == 1){
               	  waitfor(DelayMs(1000));
                  wfd error=COF_WR_DATOS_CONFI_HMI_CC();                                  // Escribe la Configuracion de Escalas y Rangos de los instrumentos
                  if(error == 0xffff)                                                     // Una sola vez
                  {
                     flag_inicio = 0;
                     PRINTFDEBUG("WR_data_HMI\n");
                  }
                  else
                  {
                  	PRINTFDEBUG("ERROR WR_data_HMI\n");
                  }
               }

              if(flag_inicio == 0){
               	  waitfor(DelayMs(1000));
                  wfd error=COF_WR_INSTA_HMI_CC();
                  waitfor(DelayMs(1000));
                  wfd error=COF_RD_FLAG_CAMBIO_HMI_CC();

                  if(error == 0xffff){                                                     //
                     if( flag_new_config == 0x01 ){
                     	  waitfor(DelayMs(1000));
                        wfd error=COF_RD_STORE_DATOS_CONF_DATOS_HMI_CC();

                        if(error == 0xffff)                                              //
                        {
                        	PRINTFDEBUG("RD_data_HMI\n");
                           waitfor( DelayMs(100) );
                           wfd error=COF_RESET_FLAG_DATOS_CONFI_HMI_CC();
                           if(error == 0xffff)                                            //
                           {
                           	PRINTFDEBUG("ERASE_FLAG_HMI\n");
                              flag_new_config = 0;
                              memcpy((char*)&my4XRegs[2204],CONF_ANA, sizeof(CONF_ANA));
                           }
                        }
                     }
              }else{
                  	PRINTFDEBUG("ERROR RD_FLAG_CAMBIO_HMI_CC\n");
              }

              if( my4XRegs[ addr_start_table_confi + 1 ] == 1){
	                  memcpy(CONF_ANA, (char*)&my4XRegs[ addr_start_table_confi +4 ], sizeof(CONF_ANA));
	                  memcpy(buf_tempo_microSD,(char*)CONF_ANA,sizeof(CONF_ANA));
	                  write_micro_SD(No_sector_datos_CONF,(char*)buf_tempo_microSD, sizeof(buf_tempo_microSD));
	                  my4XRegs[ addr_start_table_confi +1 ] = 0;
                    flag_inicio = 1;
   						      flag_new_config = 0;
                    PRINTFDEBUG("REINICIA_copy_data_HMI\n");
              }
          }

          /////////////////////////////////////////////////////////////////////////////////////////
          //
          //		SET TIME        W E B
          /////////////////////////////////////////////////////////////////////////////////////////

          if( my4XRegs[W_set_tiempo] == 0x01 ){                              // Si flag de set_tiempo == 0x01 actualiza tiempo reloj
            rtc.tm_sec     = my4XRegs[W_segundos];                         // change the time
            rtc.tm_min     = my4XRegs[W_minutos];
            rtc.tm_hour    = my4XRegs[W_horas];
            //tm_setMDY( &rtc, 12, 31, 1999);
            tm_setMDY( &rtc, my4XRegs[W_mes],my4XRegs[W_dia], my4XRegs[W_año]);
            tm_wr(&rtc);                                                   // set clock
            t1 = mktime(&rtc);
            PRINTFDEBUG("Setting date/time to ");
            PRINTFDEBUG("Fecha -> %u/%u/%u %u:%u:%u\n\r",rtc.tm_year+1900,rtc.tm_mon,rtc.tm_mday,rtc.tm_hour,rtc.tm_min,rtc.tm_sec);
            my4XRegs[W_set_tiempo] = 0;
          }

          /////////////////////////////////////////////////////////////////////////////////////////
          //
          //		SET TIME        S C A I R L I N K
          /////////////////////////////////////////////////////////////////////////////////////////

          waitfor(DelayMs(100));
	        //               it  = mmRead_alter(    unsigned wAddr,       unsigned wReg,   unsigned wCount,                  void *pwRegs)
	        waitfor (      error = mmRead_alter(     dir_scairlink,          U_segundos,                 7,  (int *)&my4XRegs[U_segundos]));           // Read Registros
	        if( error == 0xffff ){
	              if( my4XRegs[U_set_tiempo] == 0x01 ){                              // Si flag de set_tiempo == 0x01 actualiza tiempo reloj
	                  rtc.tm_sec     = my4XRegs[U_segundos];                         // change the time
	                  rtc.tm_min     = my4XRegs[U_minutos];
	                  rtc.tm_hour    = my4XRegs[U_horas];
	                  //tm_setMDY( &rtc, 12, 31, 1999);
	                  tm_setMDY( &rtc, my4XRegs[U_mes],my4XRegs[U_dia], my4XRegs[U_año]);
	                  tm_wr(&rtc);                                                   // set clock
	                  t1 = mktime(&rtc);
	                  PRINTFDEBUG("Setting date/time to ");
	                  PRINTFDEBUG("Fecha -> %u/%u/%u %u:%u:%u\n\r",rtc.tm_year+1900,rtc.tm_mon,rtc.tm_mday,rtc.tm_hour,rtc.tm_min,rtc.tm_sec);
	                  waitfor(DelayMs(100));
	                                      // mmPresetReg_alter(   unsigned wAddr,       unsigned wReg, unsigned wVal)
	                  waitfor (      error = mmPresetReg_alter(    dir_scairlink,        U_set_tiempo,             0));
	                  if( error == 0xffff ){
	                     PRINTFDEBUG("OK Reset Sincronia Tiempo\n");
	                  }else{
	                     PRINTFDEBUG("Error Sincronia Tiempo\n");
	                  }
	              }
	        }else{
	               PRINTFDEBUG("Error Read Datos Tiempo\n");
	        }

          #ifdef endress_485

	        lerma_IDmodbus=(unsigned int) CONF_ANA[4].K2_data;
	        lerma_float_error=0;

	        PRINTFDEBUG("\nID medidor: %d", lerma_IDmodbus);

            waitfor(DelayMs(100));
                          // it  = mmRead_clon(     unsigned wAddr,      unsigned wReg,   unsigned wCount,                  void *pwRegs)
            waitfor (      error = mmRead_clon(  lerma_IDmodbus,   						reg_flujo,                 2,                   (int *)&bufauxint[0]));

            if(error != -1 )
            {
            	conta_errores++;
               if( conta_errores == No_max_errores_instru )
               {
               	conta_errores = No_max_errores_instru;
                  digOut(7, 0);
     					waitfor(DelayMs(3000));
     					digOut(7, 1);
               }

               if( conta_errores >= No_max_errores_instru + 200  )
               {
               	conta_errores = No_max_errores_instru;
                  digOut(7, 0);
     					waitfor(DelayMs(3000));
     					digOut(7, 1);
                  conta_errores = 0;
               }





            }
            else
            {
            	conta_errores = 0;
            }



            memcpy(&bufaux[0], &bufauxint[0],4);

            conver.conver_byte[0]=bufaux[0];
            conver.conver_byte[1]=bufaux[1];
            conver.conver_byte[2]=bufaux[2];
            conver.conver_byte[3]=bufaux[3];

            flotanteaux=(float) conver.conver_float;
            printf("\nINSTANTANEO l/h: %f\n",flotanteaux );
            conver.conver_float=(float)flotanteaux/3600;
            printf("\nINSTANTANEO l/s: %f\n",conver.conver_float );

            my4XRegs[105]=conver.conver_int[0];
            my4XRegs[106]=conver.conver_int[1];


            if( error == 0xffff ){
                    my4XRegs[108]=1; // se pone 1 en rango si respondio
                    my4XRegs[109]=1;
            }else{
                    PRINTFDEBUG("Error WR Med Flujo\n");
                    my4XRegs[108]=0; // se pone 0 en rango
                    my4XRegs[109]=0;
            }
            waitfor(DelayMs(100));
                          // it  = mmRead_clon(     unsigned wAddr,      unsigned wReg,   unsigned wCount,                  void *pwRegs)
            waitfor (      error = mmRead_clon(  lerma_IDmodbus,   						reg_totalizado,                 4,                   (int *)&bufauxint[0]));

            memcpy(&bufaux[0], &bufauxint[0],8);

            dato64b.to_byte[0]=bufaux[7];
            dato64b.to_byte[1]=bufaux[6];
            dato64b.to_byte[2]=bufaux[5];
            dato64b.to_byte[3]=bufaux[4];
            dato64b.to_byte[4]=bufaux[3];
            dato64b.to_byte[5]=bufaux[2];
            dato64b.to_byte[6]=bufaux[1];
            dato64b.to_byte[7]=bufaux[0];

            //conversion de double a float
            signoD=dato64b.to_byte[0]&0X80;
            //PRINTFDEBUG("\n to_byte[0]:%02X ",dato64b.to_byte[0]);
            if(signoD!=0){
              PRINTFDEBUG("\nSigno:1");
              lerma_float_error=0;
            }else{
              PRINTFDEBUG("\nSigno:0");
              lerma_float_error=0;
            }
            dato16b.to_byte[0]=dato64b.to_byte[1];
            dato16b.to_byte[1]=dato64b.to_byte[0];
            //PRINTFDEBUG("\n int0:%u ",dato16b.to_uint16);
            exponenteD=(dato16b.to_uint16>>4)&0X07FF;
            //PRINTFDEBUG("\n Exponente:%u",exponenteD);
            if(exponenteD==0){
            	exponenteD=0;
            	PRINTFDEBUG("\nExponente: 0!");
            }else{
            	exponenteD=exponenteD-896;
            }

            //PRINTFDEBUG("\n Exponente:%i",exponenteD);
            if(exponenteD<0 || exponenteD>255){
            	lerma_float_error=1;
            }

            dato32b.to_byte[1]=dato64b.to_byte[3];
            dato32b.to_byte[0]=dato64b.to_byte[4];
            dato32b.to_byte[3]=dato64b.to_byte[1];
            dato32b.to_byte[2]=dato64b.to_byte[2];
            //PRINTFDEBUG("\n MantisaP: %lu", dato32b.to_ulong);
            mantisaD=(dato32b.to_ulong>>5)& 0X007FFFFF;
            //PRINTFDEBUG("\n Mantisa: %lu", mantisaD);
            dato32b.to_ulong=mantisaD;

            if(signoD!=0){
              dato32b.to_ulong=(exponenteD<<23) | mantisaD | 0X80000000;
            }else{
              dato32b.to_ulong=(exponenteD<<23) | mantisaD;
            }

            conver.conver_byte[0]=dato32b.to_byte[0];
            conver.conver_byte[1]=dato32b.to_byte[1];
            conver.conver_byte[2]=dato32b.to_byte[2];
            conver.conver_byte[3]=dato32b.to_byte[3];

            flotanteaux=(float) conver.conver_float;

            lerma_in_range=in_range(flotanteaux, -777777777, 777777777);

            if(lerma_in_range==1){
            	lerma_in_range=1;
            }else{
            	lerma_float_error=1;
              flotanteaux=777777777;
            }

            printf("\nTOTALIZADO l: %f\n",flotanteaux );
            conver.conver_float=(float) flotanteaux/1000;
            printf("\nTOTALIZADO m3: %f\n",conver.conver_float );
            /*PRINTFDEBUG("\nconverfloat=%f \n", conver.conver_float);
            PRINTFDEBUG("\nconver[0]:%02X ",conver.conver_byte[0]);
            PRINTFDEBUG("\nconver[1]:%02X ",conver.conver_byte[1]);
            PRINTFDEBUG("\nconver[2]:%02X ",conver.conver_byte[2]);
            PRINTFDEBUG("\nconver[3]:%02X ",conver.conver_byte[3]); */
            my4XRegs[110]=conver.conver_int[0];
            my4XRegs[111]=conver.conver_int[1];


            if( error == 0xffff && lerma_float_error==0 ){
                    my4XRegs[113]=1; // se pone 1 en rango si respondio y no hubo error en float
                    my4XRegs[114]=1;
            }else{
                    PRINTFDEBUG("Error RD Med Flujo\n");
                    my4XRegs[113]=0; // se pone 0 en rango
                    my4XRegs[114]=0;
            }
          #else
            conver.conver_float 	= 	totalizador*CONF_ANA[4].K2_data;
            my4XRegs[110]         = conver.conver_int[0];                // Parte Alta Totalizador
            my4XRegs[111]         = conver.conver_int[1];                // Parte Baja Totalizador
            my4XRegs[113]         = 1;                                   // Rango

          #endif
        }
      }
    }

      /*===================================================================
	      MODULO QUE A TRAVES MODBUS MANDA EJECUTAR MANDO
	   ===================================================================*/

    costate{
      	if(ejecuta_mando == 0x01){
         	wfd COF_Genera_Pulso_Digi(No_salida);
            ejecuta_mando=0x00;
        }
    }
  }
}

               	//PRINTFDEBUG("%x,\n",my4XRegs[74]);
                  /*
                  for(ii=50; ii<=74; ii++)
	               {
	                  PRINTFDEBUG("%x,",my4XRegs[ii]);
	               }
	               PRINTFDEBUG("\n");
                  */

