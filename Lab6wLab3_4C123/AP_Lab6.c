// AP_Lab6.c
// Runs on either MSP432 or TM4C123
// see GPIO.c file for hardware connections 

// Daniel Valvano and Jonathan Valvano
// November 20, 2016
// CC2650 booster or CC2650 LaunchPad, CC2650 needs to be running SimpleNP 2.2 (POWERSAVE)

#include <stdint.h>
#include "../inc/UART0.h"
#include "../inc/UART1.h"
#include "../inc/AP.h"
#include "AP_Lab6.h"
//**debug macros**APDEBUG defined in AP.h********
#ifdef APDEBUG
#define OutString(STRING) UART0_OutString(STRING)
#define OutUHex(NUM) UART0_OutUHex(NUM)
#define OutUHex2(NUM) UART0_OutUHex2(NUM)
#define OutChar(N) UART0_OutChar(N)
#else
#define OutString(STRING)
#define OutUHex(NUM)
#define OutUHex2(NUM)
#define OutChar(N)
#endif

//****links into AP.c**************
extern const uint32_t RECVSIZE;
extern uint8_t RecvBuf[];
typedef struct characteristics{
  uint16_t theHandle;          // each object has an ID
  uint16_t size;               // number of bytes in user data (1,2,4,8)
  uint8_t *pt;                 // pointer to user data, stored little endian
  void (*callBackRead)(void);  // action if SNP Characteristic Read Indication
  void (*callBackWrite)(void); // action if SNP Characteristic Write Indication
}characteristic_t;
extern const uint32_t MAXCHARACTERISTICS;
extern uint32_t CharacteristicCount;
extern characteristic_t CharacteristicList[];
typedef struct NotifyCharacteristics{
  uint16_t uuid;               // user defined 
  uint16_t theHandle;          // each object has an ID (used to notify)
  uint16_t CCCDhandle;         // generated/assigned by SNP
  uint16_t CCCDvalue;          // sent by phone to this object
  uint16_t size;               // number of bytes in user data (1,2,4,8)
  uint8_t *pt;                 // pointer to user data array, stored little endian
  void (*callBackCCCD)(void);  // action if SNP CCCD Updated Indication
}NotifyCharacteristic_t;
extern const uint32_t NOTIFYMAXCHARACTERISTICS;
extern uint32_t NotifyCharacteristicCount;
extern NotifyCharacteristic_t NotifyCharacteristicList[];
//**************Lab 6 routines*******************
// **********SetFCS**************
// helper function, add check byte to message
// assumes every byte in the message has been set except the FCS
// used the length field, assumes less than 256 bytes
// FCS = 8-bit EOR(all bytes except SOF and the FCS itself)
// Inputs: pointer to message
//         stores the FCS into message itself
// Outputs: none
void SetFCS(uint8_t *msg){
//****You implement this function as part of Lab 6*****
	uint32_t size; uint8_t fcs; uint8_t data;
	size = AP_GetSize(msg);
	fcs = 0;
	msg++;
	data = *msg; fcs = fcs^data; msg++;
	data = *msg; fcs = fcs^data; msg++;
	data = *msg; fcs = fcs^data; msg++;
	data = *msg; fcs = fcs^data; msg++;
	for(int i=0;i<size;i++){
		data = *msg; fcs = fcs^data; msg++;
	}
	*msg = fcs;
}
//*************BuildGetStatusMsg**************
// Create a Get Status message, used in Lab 6
// Inputs pointer to empty buffer of at least 6 bytes
// Output none
// build the necessary NPI message that will Get Status
void BuildGetStatusMsg(uint8_t *msg){
// hint: see NPI_GetStatus in AP.c
//****You implement this function as part of Lab 6*****

 uint8_t *GetStatus;
	GetStatus = msg;
	*GetStatus = SOF;  GetStatus++;
	*GetStatus = 0x00; GetStatus++; //length 
	*GetStatus = 0x00; GetStatus++;
	*GetStatus = 0x55; GetStatus++;
	*GetStatus = 0x06; GetStatus++;
	SetFCS(msg); 
}
//*************Lab6_GetStatus**************
// Get status of connection, used in Lab 6
// Input:  none
// Output: status 0xAABBCCDD
// AA is GAPRole Status
// BB is Advertising Status
// CC is ATT Status
// DD is ATT method in progress
uint32_t Lab6_GetStatus(void){volatile int r; uint8_t sendMsg[8];
  OutString("\n\rGet Status");
  BuildGetStatusMsg(sendMsg);
  r = AP_SendMessageResponse(sendMsg,RecvBuf,RECVSIZE);
  return (RecvBuf[4]<<24)+(RecvBuf[5]<<16)+(RecvBuf[6]<<8)+(RecvBuf[7]);
}

//*************BuildGetVersionMsg**************
// Create a Get Version message, used in Lab 6
// Inputs pointer to empty buffer of at least 6 bytes
// Output none
// build the necessary NPI message that will Get Status
void BuildGetVersionMsg(uint8_t *msg){
// hint: see NPI_GetVersion in AP.c
//****You implement this function as part of Lab 6*****
  uint8_t *GetVersion;
	*GetVersion = SOF; GetVersion++;
	*GetVersion = 0x00; GetVersion++; //length
	*GetVersion = 0x00; GetVersion++; 
	*GetVersion = 0x35; GetVersion++; 
	*GetVersion = 0x03; GetVersion++;
	SetFCS(msg); 
}
//*************Lab6_GetVersion**************
// Get version of the SNP application running on the CC2650, used in Lab 6
// Input:  none
// Output: version
uint32_t Lab6_GetVersion(void){volatile int r;uint8_t sendMsg[8];
  OutString("\n\rGet Version");
  BuildGetVersionMsg(sendMsg);
  r = AP_SendMessageResponse(sendMsg,RecvBuf,RECVSIZE); 
  return (RecvBuf[5]<<8)+(RecvBuf[6]);
}

//*************BuildAddServiceMsg**************
// Create an Add service message, used in Lab 6
// Inputs uuid is 0xFFF0, 0xFFF1, ...
//        pointer to empty buffer of at least 9 bytes
// Output none
// build the necessary NPI message that will add a service
void BuildAddServiceMsg(uint16_t uuid, uint8_t *msg){
//****You implement this function as part of Lab 6*****
    uint8_t *GetService;
		 GetService = msg;
		*GetService = SOF; GetService++; 
		*GetService = 0x03; GetService++;  //Length
		*GetService = 0x00; GetService++;
		*GetService = 0x35; GetService++; //SNP add service
		*GetService = 0x81; GetService++; //SNP add service
		*GetService = 0x01; GetService++; // Primary Service
		*GetService = 0xFF&uuid; GetService++; // uuid
		*GetService = uuid>>8; GetService++;
		 SetFCS(msg); 
}
//*************Lab6_AddService**************
// Add a service, used in Lab 6
// Inputs uuid is 0xFFF0, 0xFFF1, ...
// Output APOK if successful,
//        APFAIL if SNP failure
int Lab6_AddService(uint16_t uuid){ int r; uint8_t sendMsg[12];
  OutString("\n\rAdd service");
  BuildAddServiceMsg(uuid,sendMsg);
  r = AP_SendMessageResponse(sendMsg,RecvBuf,RECVSIZE);  
  return r;
}
//*************AP_BuildRegisterServiceMsg**************
// Create a register service message, used in Lab 6
// Inputs pointer to empty buffer of at least 6 bytes
// Output none
// build the necessary NPI message that will register a service
void BuildRegisterServiceMsg(uint8_t *msg){
//****You implement this function as part of Lab 6*****
	 uint8_t *RegService;
	 RegService = msg;
	*RegService= SOF; RegService++; 
	*RegService= 0x00;	RegService++;
	*RegService= 0x00; RegService++;
	*RegService= 0x35; RegService++; 	*RegService= 0x84; RegService++; // SNP Register Service
	 SetFCS(msg); 
}
//*************Lab6_RegisterService**************
// Register a service, used in Lab 6
// Inputs none
// Output APOK if successful,
//        APFAIL if SNP failure
int Lab6_RegisterService(void){ int r; uint8_t sendMsg[8];
  OutString("\n\rRegister service");
  BuildRegisterServiceMsg(sendMsg);
  r = AP_SendMessageResponse(sendMsg,RecvBuf,RECVSIZE);
  return r;
}

//*************BuildAddCharValueMsg**************
// Create a Add Characteristic Value Declaration message, used in Lab 6
// Inputs uuid is 0xFFF0, 0xFFF1, ...
//        permission is GATT Permission, 0=none,1=read,2=write, 3=Read+write 
//        properties is GATT Properties, 2=read,8=write,0x0A=read+write, 0x10=notify
//        pointer to empty buffer of at least 14 bytes
// Output none
// build the necessary NPI message that will add a characteristic value
void BuildAddCharValueMsg(uint16_t uuid,  
  uint8_t permission, uint8_t properties, uint8_t *msg){
// set RFU to 0 and
// set the maximum length of the attribute value=512
// for a hint see NPI_AddCharValue in AP.c
// for a hint see first half of AP_AddCharacteristic and first half of AP_AddNotifyCharacteristic
//****You implement this function as part of Lab 6*****
 
	uint8_t *AddCharValue;
	AddCharValue = msg;		
	*AddCharValue = SOF; AddCharValue++;
	*AddCharValue = 0x08; AddCharValue++;
	*AddCharValue = 0x00; AddCharValue++;	
	*AddCharValue = 0x35; AddCharValue++;   // SNP Add Characteristic Value Declaration
  *AddCharValue = 0x82; AddCharValue++; 
  *AddCharValue = permission; AddCharValue++;// 0=none,1=read,2=write, 3=Read+write, GATT Permission
  *AddCharValue = properties; AddCharValue++;// 2=read,8=write,0x0A=read+write,0x10=notify, GATT Properties
  *AddCharValue = 0x00; AddCharValue++; 
	*AddCharValue = 0x00; AddCharValue++; //RFU
	*AddCharValue = 0x00; AddCharValue++; 
	*AddCharValue = 0x02; AddCharValue++;	// Maximum length
	*AddCharValue = 0xFF&uuid;AddCharValue++; // UUID 
	*AddCharValue = uuid>>8; AddCharValue++; 
	SetFCS(msg);
}

//*************BuildAddCharDescriptorMsg**************
// Create a Add Characteristic Descriptor Declaration message, used in Lab 6
// Inputs name is a null-terminated string, maximum length of name is 20 bytes
//        pointer to empty buffer of at least 32 bytes
// Output none
// build the necessary NPI message that will add a Descriptor Declaration
void BuildAddCharDescriptorMsg(char name[], uint8_t *msg){
// set length and maxlength to the string length
// set the permissions on the string to read
// for a hint see NPI_AddCharDescriptor in AP.c
// for a hint see second half of AP_AddCharacteristic
//****You implement this function as part of Lab 6*****
  uint8_t *AddCharDescriptor;
	AddCharDescriptor = msg;
	*AddCharDescriptor = SOF; AddCharDescriptor++;
  *AddCharDescriptor = 0x17; AddCharDescriptor++; // frame length
  *AddCharDescriptor = 0x00; AddCharDescriptor++; // SNP Add Characteristic Descriptor Declaration
  *AddCharDescriptor = 0x35; AddCharDescriptor++;
  *AddCharDescriptor = 0x83; AddCharDescriptor++; // User Description String
  *AddCharDescriptor = 0x80; AddCharDescriptor++; // GATT Read Permissions
  *AddCharDescriptor = 0x01; AddCharDescriptor++; // string length
  *AddCharDescriptor = 0x11; AddCharDescriptor++; // string length
  *AddCharDescriptor = 0x00; AddCharDescriptor++; 
  *AddCharDescriptor = 0x11; AddCharDescriptor++;
  *AddCharDescriptor = 0x00; AddCharDescriptor++;
  	int i=0;
	while((name[i])&&(i<20)){
		*AddCharDescriptor = name[i];	i++; AddCharDescriptor++; //Description String
	}
	*AddCharDescriptor = 0x00; AddCharDescriptor++;	i++;
	AddCharDescriptor = msg;
	for(int j=0;j<11;j++){
		if(j==1){
			*AddCharDescriptor = 6+i; 
		}
		if(j==7 || j==9){
			*AddCharDescriptor = i;
		}
		AddCharDescriptor++;
	}
	SetFCS(msg);
}

//*************Lab6_AddCharacteristic**************
// Add a read, write, or read/write characteristic, used in Lab 6
//        for notify properties, call AP_AddNotifyCharacteristic 
// Inputs uuid is 0xFFF0, 0xFFF1, ...
//        thesize is the number of bytes in the user data 1,2,4, or 8 
//        pt is a pointer to the user data, stored little endian
//        permission is GATT Permission, 0=none,1=read,2=write, 3=Read+write 
//        properties is GATT Properties, 2=read,8=write,0x0A=read+write
//        name is a null-terminated string, maximum length of name is 20 bytes
//        (*ReadFunc) called before it responses with data from internal structure
//        (*WriteFunc) called after it accepts data into internal structure
// Output APOK if successful,
//        APFAIL if name is empty, more than 8 characteristics, or if SNP failure
int Lab6_AddCharacteristic(uint16_t uuid, uint16_t thesize, void *pt, uint8_t permission,
  uint8_t properties, char name[], void(*ReadFunc)(void), void(*WriteFunc)(void)){
  int r; uint16_t handle; 
  uint8_t sendMsg[32];  
  if(thesize>8) return APFAIL;
  if(name[0]==0) return APFAIL;       // empty name
  if(CharacteristicCount>=MAXCHARACTERISTICS) return APFAIL; // error
  BuildAddCharValueMsg(uuid,permission,properties,sendMsg);
  OutString("\n\rAdd CharValue");
  r=AP_SendMessageResponse(sendMsg,RecvBuf,RECVSIZE);
  if(r == APFAIL) return APFAIL;
  handle = (RecvBuf[7]<<8)+RecvBuf[6]; // handle for this characteristic
  OutString("\n\rAdd CharDescriptor");
  BuildAddCharDescriptorMsg(name,sendMsg);
  r=AP_SendMessageResponse(sendMsg,RecvBuf,RECVSIZE);
  if(r == APFAIL) return APFAIL;
  CharacteristicList[CharacteristicCount].theHandle = handle;
  CharacteristicList[CharacteristicCount].size = thesize;
  CharacteristicList[CharacteristicCount].pt = (uint8_t *) pt;
  CharacteristicList[CharacteristicCount].callBackRead = ReadFunc;
  CharacteristicList[CharacteristicCount].callBackWrite = WriteFunc;
  CharacteristicCount++;
  return APOK; // OK
} 
  

//*************BuildAddNotifyCharDescriptorMsg**************
// Create a Add Notify Characteristic Descriptor Declaration message, used in Lab 6
// Inputs name is a null-terminated string, maximum length of name is 20 bytes
//        pointer to empty buffer of at least bytes
// Output none
// build the necessary NPI message that will add a Descriptor Declaration
void BuildAddNotifyCharDescriptorMsg(char name[], uint8_t *msg){
// set length and maxlength to the string length
// set the permissions on the string to read
// set User Description String
// set CCCD parameters read+write
// for a hint see NPI_AddCharDescriptor4 in VerySimpleApplicationProcessor.c
// for a hint see second half of AP_AddNotifyCharacteristic
//****You implement this function as part of Lab 6*****
  uint8_t *AddCharDescriptor;
		AddCharDescriptor = msg;
	int i = 0;
	for(i=0; i<12; i++)
 {
	 AddCharDescriptor++;
 }
	i = 0;
	while((name[i]))
	{
		*AddCharDescriptor = name[i]; i++; AddCharDescriptor++; // Add string 
  }
  *AddCharDescriptor =0; i++; AddCharDescriptor++;
 	AddCharDescriptor = msg;
	*AddCharDescriptor = SOF; AddCharDescriptor++; 
	*AddCharDescriptor = 7+i; AddCharDescriptor++;
	*AddCharDescriptor = 0x00; AddCharDescriptor++;
	*AddCharDescriptor = 0x35; AddCharDescriptor++;
	*AddCharDescriptor = 0x83; AddCharDescriptor++; //SNP Add Characteristic Descriptor Declaration
	*AddCharDescriptor = 0x84; AddCharDescriptor++; //User Description String
	*AddCharDescriptor = 0x03; AddCharDescriptor++; // Read + Write CCCD parameters
	*AddCharDescriptor = 0x01; AddCharDescriptor++; //GATT Read Write Permissions
	*AddCharDescriptor = i; AddCharDescriptor++; // Maximum Length
	*AddCharDescriptor = 0x00; AddCharDescriptor++; // Initial Length
	*AddCharDescriptor = i; AddCharDescriptor++;
	*AddCharDescriptor = 0x00; AddCharDescriptor++;
	SetFCS(msg);
}
//*************Lab6_AddNotifyCharacteristic**************
// Add a notify characteristic, used in Lab 6
//        for read, write, or read/write characteristic, call AP_AddCharacteristic 
// Inputs uuid is 0xFFF0, 0xFFF1, ...
//        thesize is the number of bytes in the user data 1,2,4, or 8 
//        pt is a pointer to the user data, stored little endian
//        name is a null-terminated string, maximum length of name is 20 bytes
//        (*CCCDfunc) called after it accepts , changing CCCDvalue
// Output APOK if successful,
//        APFAIL if name is empty, more than 4 notify characteristics, or if SNP failure
int Lab6_AddNotifyCharacteristic(uint16_t uuid, uint16_t thesize, void *pt,   
  char name[], void(*CCCDfunc)(void)){
  int r; uint16_t handle; 
  uint8_t sendMsg[36];  
  if(thesize>8) return APFAIL;
  if(NotifyCharacteristicCount>=NOTIFYMAXCHARACTERISTICS) return APFAIL; // error
  BuildAddCharValueMsg(uuid,0,0x10,sendMsg);
  OutString("\n\rAdd Notify CharValue");
  r=AP_SendMessageResponse(sendMsg,RecvBuf,RECVSIZE);
  if(r == APFAIL) return APFAIL;
  handle = (RecvBuf[7]<<8)+RecvBuf[6]; // handle for this characteristic
  OutString("\n\rAdd CharDescriptor");
  BuildAddNotifyCharDescriptorMsg(name,sendMsg);
  r=AP_SendMessageResponse(sendMsg,RecvBuf,RECVSIZE);
  if(r == APFAIL) return APFAIL;
  NotifyCharacteristicList[NotifyCharacteristicCount].uuid = uuid;
  NotifyCharacteristicList[NotifyCharacteristicCount].theHandle = handle;
  NotifyCharacteristicList[NotifyCharacteristicCount].CCCDhandle = (RecvBuf[8]<<8)+RecvBuf[7]; // handle for this CCCD
  NotifyCharacteristicList[NotifyCharacteristicCount].CCCDvalue = 0; // notify initially off
  NotifyCharacteristicList[NotifyCharacteristicCount].size = thesize;
  NotifyCharacteristicList[NotifyCharacteristicCount].pt = (uint8_t *) pt;
  NotifyCharacteristicList[NotifyCharacteristicCount].callBackCCCD = CCCDfunc;
  NotifyCharacteristicCount++;
  return APOK; // OK
}

//*************BuildSetDeviceNameMsg**************
// Create a Set GATT Parameter message, used in Lab 6
// Inputs name is a null-terminated string, maximum length of name is 24 bytes
//        pointer to empty buffer of at least 36 bytes
// Output none
// build the necessary NPI message to set Device name
void BuildSetDeviceNameMsg(char name[], uint8_t *msg){
// for a hint see NPI_GATTSetDeviceNameMsg in VerySimpleApplicationProcessor.c
// for a hint see NPI_GATTSetDeviceName in AP.c
//****You implement this function as part of Lab 6*****
	int i = 0;
	uint8_t *GATTSetDeviceNameMsg; 
  GATTSetDeviceNameMsg = msg;
	*GATTSetDeviceNameMsg = SOF; GATTSetDeviceNameMsg++;
	*GATTSetDeviceNameMsg = 0x00; GATTSetDeviceNameMsg++;
	*GATTSetDeviceNameMsg = 0x00; GATTSetDeviceNameMsg++; 
	*GATTSetDeviceNameMsg = 0x35; GATTSetDeviceNameMsg++; // Set GATT Parameter 
	*GATTSetDeviceNameMsg = 0x8C; GATTSetDeviceNameMsg++; // Set GATT Parameter
	*GATTSetDeviceNameMsg = 0x01; GATTSetDeviceNameMsg++; // Generic Access Service 
	*GATTSetDeviceNameMsg = 0x00; GATTSetDeviceNameMsg++;
	*GATTSetDeviceNameMsg = 0x00; GATTSetDeviceNameMsg++;
   while((name[i]))
	 {
		 *GATTSetDeviceNameMsg = name[i];	 GATTSetDeviceNameMsg++; i++;
	 }
	 GATTSetDeviceNameMsg = msg; GATTSetDeviceNameMsg++; 
	 *GATTSetDeviceNameMsg = i+3; // Length
	 SetFCS(msg);
}
//*************BuildSetAdvertisementData1Msg**************
// Create a Set Advertisement Data message, used in Lab 6
// Inputs pointer to empty buffer of at least 16 bytes
// Output none
// build the necessary NPI message for Non-connectable Advertisement Data
void BuildSetAdvertisementData1Msg(uint8_t *msg){
// for a hint see NPI_SetAdvertisementMsg in VerySimpleApplicationProcessor.c
// for a hint see NPI_SetAdvertisement1 in AP.c
// Non-connectable Advertisement Data
// GAP_ADTYPE_FLAGS,DISCOVERABLE | no BREDR  
// Texas Instruments Company ID 0x000D
// TI_ST_DEVICE_ID = 3
// TI_ST_KEY_DATA_ID
// Key state=0
//****You implement this function as part of Lab 6*****
  uint8_t *SetAdvertisementData1Msg;
   SetAdvertisementData1Msg = msg;
	*SetAdvertisementData1Msg = SOF; SetAdvertisementData1Msg++;
	*SetAdvertisementData1Msg = 11; SetAdvertisementData1Msg++; // Length
	*SetAdvertisementData1Msg = 0x00; SetAdvertisementData1Msg++; 
	*SetAdvertisementData1Msg = 0x55; SetAdvertisementData1Msg++; 	*SetAdvertisementData1Msg = 0x43; SetAdvertisementData1Msg++; //SNP Set Advertisement Data
	*SetAdvertisementData1Msg = 0x01; SetAdvertisementData1Msg++; //Disconnected Advertisement Data
		*SetAdvertisementData1Msg = 0x02; SetAdvertisementData1Msg++;  
	*SetAdvertisementData1Msg = 0x01; SetAdvertisementData1Msg++;
	*SetAdvertisementData1Msg = 0x06; SetAdvertisementData1Msg++;
	*SetAdvertisementData1Msg = 0x06; SetAdvertisementData1Msg++;
	*SetAdvertisementData1Msg = 0xFF; SetAdvertisementData1Msg++;
	*SetAdvertisementData1Msg = 0x0D; SetAdvertisementData1Msg++;
	*SetAdvertisementData1Msg = 0x00; SetAdvertisementData1Msg++;
	*SetAdvertisementData1Msg = 0x03; SetAdvertisementData1Msg++;
	*SetAdvertisementData1Msg = 0x00; SetAdvertisementData1Msg++;
	*SetAdvertisementData1Msg = 0x00; SetAdvertisementData1Msg++;
	SetFCS(msg);
}

//*************BuildSetAdvertisementDataMsg**************
// Create a Set Advertisement Data message, used in Lab 6
// Inputs name is a null-terminated string, maximum length of name is 24 bytes
//        pointer to empty buffer of at least 36 bytes
// Output none
// build the necessary NPI message for Scan Response Data
void BuildSetAdvertisementDataMsg(char name[], uint8_t *msg){
// for a hint see NPI_SetAdvertisementDataMsg in VerySimpleApplicationProcessor.c
// for a hint see NPI_SetAdvertisementData in AP.c
//****You implement this function as part of Lab 6*****
  uint8_t *SetAdvertisementDataMsg; int i = 0;
	SetAdvertisementDataMsg = msg;
	*SetAdvertisementDataMsg = SOF; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x00; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x00; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x55; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x43; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x00; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x00; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x09; SetAdvertisementDataMsg++;
	while(name[i])
	{
		*SetAdvertisementDataMsg = name[i]; i++; SetAdvertisementDataMsg++;
	}
	*SetAdvertisementDataMsg = 0x05; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x12; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x50; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x00; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x20; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x03; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x02; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x0A; SetAdvertisementDataMsg++;
	*SetAdvertisementDataMsg = 0x00; SetAdvertisementDataMsg++;
	SetAdvertisementDataMsg = msg;

	SetAdvertisementDataMsg[1] = i+12; // Length
	SetAdvertisementDataMsg[6] = i+1; // Length of string
	
	SetFCS(msg);  
}
//*************BuildStartAdvertisementMsg**************
// Create a Start Advertisement Data message, used in Lab 6
// Inputs advertising interval
//        pointer to empty buffer of at least 20 bytes
// Output none
// build the necessary NPI message to start advertisement
void BuildStartAdvertisementMsg(uint16_t interval, uint8_t *msg ){
// for a hint see NPI_StartAdvertisementMsg in VerySimpleApplicationProcessor.c
// for a hint see NPI_StartAdvertisement in AP.c
//****You implement this function as part of Lab 6*****
uint8_t *StartAdvertisementMsg;  
  StartAdvertisementMsg = msg;
	*StartAdvertisementMsg = SOF; StartAdvertisementMsg++;
	*StartAdvertisementMsg = 14; StartAdvertisementMsg++; // Length
	*StartAdvertisementMsg = 0x00; StartAdvertisementMsg ++;
	*StartAdvertisementMsg = 0x55; StartAdvertisementMsg ++; // Start advertisement command
	*StartAdvertisementMsg = 0x42; StartAdvertisementMsg ++;
	*StartAdvertisementMsg = 0x00; StartAdvertisementMsg ++;
	*StartAdvertisementMsg = 0x00; StartAdvertisementMsg ++;
	*StartAdvertisementMsg = 0x00; StartAdvertisementMsg ++; // Advertise Continuously
	*StartAdvertisementMsg = interval; StartAdvertisementMsg ++; 
	*StartAdvertisementMsg = 0x00; StartAdvertisementMsg++;
	*StartAdvertisementMsg = 0x00; StartAdvertisementMsg++;
	*StartAdvertisementMsg = 0x00; StartAdvertisementMsg++;
	*StartAdvertisementMsg = 0x00; StartAdvertisementMsg++;
	*StartAdvertisementMsg = 0x01; StartAdvertisementMsg++;
	*StartAdvertisementMsg = 0x00; StartAdvertisementMsg++;
	*StartAdvertisementMsg = 0x00; StartAdvertisementMsg++;
	*StartAdvertisementMsg = 0x00; StartAdvertisementMsg++;
	*StartAdvertisementMsg = 0xC5; StartAdvertisementMsg++;
	*StartAdvertisementMsg = 0x02; StartAdvertisementMsg++; // To restart advertising 
	SetFCS(msg);
	
}

//*************Lab6_StartAdvertisement**************
// Start advertisement, used in Lab 6
// Input:  none
// Output: APOK if successful,
//         APFAIL if notification not configured, or if SNP failure
int Lab6_StartAdvertisement(void){volatile int r; uint8_t sendMsg[50];
  OutString("\n\rSet Device name");
  BuildSetDeviceNameMsg("Shape the World",sendMsg);
  r =AP_SendMessageResponse(sendMsg,RecvBuf,RECVSIZE);
  OutString("\n\rSetAdvertisement1");
  BuildSetAdvertisementData1Msg(sendMsg);
  r =AP_SendMessageResponse(sendMsg,RecvBuf,RECVSIZE);
  OutString("\n\rSetAdvertisement Data");
  BuildSetAdvertisementDataMsg("Shape the World",sendMsg);
  r =AP_SendMessageResponse(sendMsg,RecvBuf,RECVSIZE);
  OutString("\n\rStartAdvertisement");
  BuildStartAdvertisementMsg(100,sendMsg);
  r =AP_SendMessageResponse(sendMsg,RecvBuf,RECVSIZE);
  return r;
}

