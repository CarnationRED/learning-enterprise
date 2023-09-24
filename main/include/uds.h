#include "stdint.h"
#include "msg.h"

typedef struct 
{
    u8 dlc;
    u8* data;
}FrameData;

typedef struct 
{
    bool success;
    u8 nrc;
    u8 errorMessage[32];
}UDSResult;

u8 NRCs[46]={
	0x00,//PR(PositiveResponse)                               
	0x10,//GR(GeneralReject)                                  
	0x11,//SNS(ServiceNotSupported).                          
	0x12,//SFNS(SubFunctionNotSupported)                      
	0x13,//IMLOIF(IncorrectMessageLengthOrInvalidFormat)      
	0x14,//RTL(ResponseTooLong)                               
	0x21,//BRR(BusyRepeatReques)                              
	0x22,//CNC(ConditionsNotCorrect)                          
	0x23,//ISOSAERESRVD                                       
	0x24,//RSE(RequestSequenceError)                          
	0x25,//NRFSC(NoResponseFromSubnetComponent)               
	0x26,//FPEORA(FailurePreventsExecutionOfRequestedAction)  
	0x31,//ROOR(RequestOutOfRange)                            
	0x32,//ISOSAERESRVD                                       
	0x33,//SAD(SecurityAccessDenied)                          
	0x34,//ISOSAERESRVD                                       
	0x35,//IK(InvalidKey)                                     
	0x36,//ENOA(ExceedNumberOfAttempts)                       
	0x37,//RTDNE(RequiredTimeDelayNotExpired)                 
	0x70,//UDNA(UploadDownloadNotAccepted)                    
	0x71,//TDS(TransferDataSuspended)                         
	0x72,//GPF(GeneralProgrammingFailure)                     
	0x73,//WBSC(WrongBlockSequenceCounter)                    
	0x78,//RCRRP(RequestCorrectlyReceived-ResponsePending)    
	0x7E,//SFNSIAS(SubFunctionNotSupportedInActiveSession)    
	0x7F,//SNSIAS(ServiceNotSupportedInActiveSession)         
	0x80,//ISOSAERESRVD                                       
	0x81,//RPMTH(RpmTooHigh)                                  
	0x82,//RPMTL(RpmTooLow)                                   
	0x83,//EIR(EngineIsRunning)                               
	0x84,//EINR(EngineIsNotRunning)                           
	0x85,//ERTTL(EngineRunTimeTooLow)                         
	0x86,//TEMPTH(TemperatureTooHigh)                         
	0x87,//TEMPTL(TemperatureTooLow)                          
	0x88,//VSTH(VehicleSpeedTooHigh)                          
	0x89,//VSTL(VehicleSpeedTooLow)                           
	0x8A,//TPTH(Throttle/PedalTooHigh)                        
	0x8B,//TPTL(Throttle/PedalTooLow)                         
	0x8C,//TRNIG(TransmissionRangeNotInNeutral)               
	0x8D,//TRNIG(TransmissionRangeNotInGear)                  
	0x8E,//ISOSAERESRVD                                       
	0x8F,//BSNC(BrakeSwitch(es)NotClosed)                     
	0x90,//SLNIP(ShifterLeverNotInPark)                       
	0x91,//TCCL(TorqueConverterClutchLocked)                  
	0x92,//VTH(VoltageTooHigh)                                
	0x93,//VTL(VoltageTooLow)                                 
};
