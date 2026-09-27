#pragma once

namespace xpcapi {

typedef struct {
    int    mode;
    double incrementvalue;
} lgmode;

typedef struct {
    int    number;
    int    type;
    int    state;
    int    signals[20];
    int    numsamples;
    int    decimation;
    int    triggermode;
    int    numprepostsamples;
    int    triggersignal;
    int    triggerscope;
    int    triggerscopesample;
    double triggerlevel;
    int    triggerslope;
} scopedata;

typedef struct {
    char          Label[12];
    char          DriveLetter;
    char          Reserved[3];
    unsigned int  SerialNumber;
    unsigned int  FirstPhysicalSector;
    unsigned int  FATType;            // 12 or 16
    unsigned int  FATCount;
    unsigned int  MaxDirEntries;
    unsigned int  BytesPerSector;
    unsigned int  SectorsPerCluster;
    unsigned int  TotalClusters;
    unsigned int  BadClusters;
    unsigned int  FreeClusters;
    unsigned int  Files;
    unsigned int  FileChains;
    unsigned int  FreeChains;
    unsigned int  LargestFreeChain;
} diskinfo;

typedef struct {
    char	    Name[8];
    char	    Ext[3];
    int		    Day;
    int             Month;
    int             Year;
    int             Hour;
    int             Min;
    int             isDir;
    unsigned long   Size;
}dirStruct;

typedef struct {
    int            FilePos;
    int            AllocatedSize;
    int            ClusterChains;
    int            VolumeSerialNumber;
    char           FullName[255];
}fileinfo;

int xPCReOpenPort(int port);
int xPCOpenSerialPort(int comport, int baudRate);
void xPCClosePort(int port);
int xPCGetLastError(void);
void xPCSetLastError(int error);
double xPCGetExecTime(int port);
void xPCSetStopTime(int port, double tfinal);
double xPCGetStopTime(int port);
void xPCSetSampleTime(int port, double ts);
double xPCGetSampleTime(int port);
void xPCSetEcho(int port, int mode);
int xPCGetEcho(int port);
void xPCSetHiddenScopeEcho(int port, int mode);
int xPCGetHiddenScopeEcho(int port);
double xPCAverageTET(int port);
int xPCGetNumParams(int port);
int xPCGetNumSignals(int port);
char * xPCGetAppName(int port, char *modelname);
void xPCUnloadApp(int port);
void xPCStartApp(int port);
void xPCStopApp(int port);
int xPCIsAppRunning(int port);
int xPCIsOverloaded(int port);
int xPCGetNumOutputs(int port);
int xPCGetNumStates(int port);
void xPCGetParam(int port, int parIdx, double *paramValue);
void xPCSetLogMode(int port, lgmode lgdata);
void xPCSetParam(int port, int parIdx, const double *paramValue);
lgmode xPCGetLogMode(int port);
int xPCNumLogSamples(int port);
int xPCMaxLogSamples(int port);
int xPCNumLogWraps(int port);
void xPCReboot(int port);
void xPCGetOutputLog(int port, int start, int numsamples,
                                  int decimation, int output_id, double *data);
void xPCGetStateLog(int port, int start, int numsamples,
                                 int decimation, int state_id, double *data);
void xPCGetTimeLog(int port, int start, int numsamples,
                                int decimation, double *data);
void xPCGetTETLog(int port, int start, int numsamples,
                               int decimation, double *data);
void xPCScGetData(int port, int scNum , int signal_id, int start,
                               int numsamples, int decimation, double *data);
void xPCMinimumTET(int port, double *data);
void xPCMaximumTET(int port, double *data);
int xPCGetSignals(int port, int numSignals, const int *signals,
                               double *values);
double xPCGetSignal(int port,  int sigNum);
void xPCAddScope(int port, int type, int scNum);
void xPCRemScope(int port, int scNum);
void xPCScAddSignal(int port, int scNum, int sigNum);
void xPCScRemSignal(int port, int scNum, int sigNum);
void xPCScSetAutoRestart(int port, int scNum, int autorestart);
int xPCScGetAutoRestart(int port, int scNum);
void xPCGetScopes(int port, int *data);
void xPCGetHiddenScopes(int port, int *data);
void xPCScGetSignals(int port, int scNum, int *data);
void xPCScSetDecimation(int port, int scNum, int decimation);
int xPCScGetNumSignals(int port, int scNum);
int xPCScGetDecimation(int port, int scNum);
void xPCScSetNumSamples(int port, int scNum, int samples);
int xPCScGetNumSamples(int port, int scNum);
double xPCScGetStartTime(int port, int scNum);
int xPCScGetState(int port, int scNum);
void xPCScSetTriggerLevel(int port, int scNum, double level);
double xPCScGetTriggerLevel(int port, int scNum);
void xPCScSetTriggerMode(int port, int scNum, int mode);
int xPCScGetTriggerMode(int port, int scNum);
void xPCScSetTriggerScope(int port, int scNum, int trigMode);
int xPCScGetTriggerScope(int port, int scNum);
void xPCScSetTriggerScopeSample(int port, int scNum, int trigScSamp);
int xPCScGetTriggerScopeSample(int port, int scNum);
void xPCScSetTriggerSignal(int port, int scNum, int trigSig);
int xPCScGetTriggerSignal(int port, int scNum);
void xPCScSetTriggerSlope(int port, int scNum, int trigSlope);
int xPCScGetTriggerSlope(int port, int scNum);
void xPCScSoftwareTrigger(int port, int scNum);
void xPCScStart(int port, int scNum);
void xPCScStop(int port, int scNum);
int xPCIsScFinished(int port, int scNum);
int xPCScGetNumPrePostSamples(int port, int scNum);
void xPCScSetNumPrePostSamples(int port, int scNum, int prepost);
scopedata xPCGetScope(int port, int scNum);
void xPCSetScope(int port, scopedata state);
void xPCLoadApp(int port, const char* pathstr,
                             const char* filename);
void xPCGetParamDims(int port, int parIdx, int *dims);
int  xPCGetParamDimsSize(int port, int parIdx);
int xPCGetSignalWidth(int port, int sigIdx);
int xPCGetSignalIdx(int port, const char *sigName );

int xPCGetSigLabelWidth(int port, const char *sigName);
int xPCGetSigIdxfromLabel(int port, const char *sigName, int *sigIds);
char * xPCGetSignalLabel(int port, int sigIdx, char *sigLabel);

int xPCGetParamIdx(int port, const char *block,
                                const char *parameter);
void xPCGetParamName(int port, int parIdx, char *block, char *param);
void xPCGetParamType(int port, int parIdx, char *paramType);
char * xPCGetSignalName(int port, int sigIdx, char *sigName);
int xPCTgScGetGrid(int port, int scNum);
int xPCTgScGetMode(int port, int scNum);
int xPCTgScGetViewMode(int port);
void xPCTgScGetYLimits(int port, int scNum, double *limits);
void xPCTgScSetGrid(int port, int scNum, int flag);
void xPCTgScSetMode(int port, int scNum, int flag);
void xPCTgScSetViewMode(int port, int scNum);
void xPCTgScSetYLimits(int port, int scNum, const double *limits);
void xPCTgScSetSignalFormat(int port, int scNum, int signalNo, const char *signalFormat);
char* xPCTgScGetSignalFormat(int port,int scNum, int signalNo, char *signalFormat);
void xPCSetLoadTimeOut(int port, int timeOut);
const char * xPCErrorMsg(int errorno, char *errmsg);
int xPCScGetType(int port, int scNum);
int xPCGetLoadTimeOut(int port);
int xPCOpenTcpIpPort(const char* address, const char* port);
void xPCOpenConnection(int port);
void xPCCloseConnection(int port);
int xPCRegisterTarget(int commType, const char *ipAddress,
                                   const char *ipPort,
                                   int comPort, int baudRate);
void xPCDeRegisterTarget(int port);
const char * xPCGetAPIVersion(void);
void xPCGetTargetVersion(int port, char *ver);
int xPCTargetPing(int port);
void xPCFSReadFile(int port, int fileHandle, int start,
                               int numsamples, unsigned char *data);
int xPCFSRead(int port, int fileHandle, int start,
                           int numsamples, unsigned char *data);
void xPCFSWriteFile(int port, int fileHandle, int numbytes,
                                 const unsigned char *data);
void xPCFSBufferInfo(int port, char *data);
int xPCFSGetFileSize(int port, int fileHandle);
int xPCFSOpenFile(int port, const char *filename,
                               const char *attrib);
void xPCFSCloseFile(int port, int fileHandle);
void xPCFSGetPWD(int port, char *data);
void xPCFTPGet(int port, int fileHandle, int numbytes, char *filename);
void xPCFTPPut(int port, int fileHandle, char *filename);
void xPCFSRemoveFile(int port, char *filename);
void xPCFSCD(int port, char *filename);
void xPCFSMKDIR(int port, const char *dirname);
void xPCFSRMDIR(int port, const char *dirname);
void xPCFSDir(int port, const char *path, char *listing, int numbytes);
int xPCFSDirSize(int port, const char *path);
void xPCFSGetError(int            port,
                                unsigned int   errCode,
                                unsigned char *message);
void xPCSaveParamSet(int port, const char *filename);
void xPCLoadParamSet(int port, const char *filename);

void xPCFSScSetFilename(int port, int scopeId,
                                     const char *filename);
const char * xPCFSScGetFilename(int port, int scopeId,
                                             char *filename);
void xPCFSScSetWriteMode(int port, int scopeId,
                                      int writeMode);
int xPCFSScGetWriteMode(int port, int scopeId);

void xPCFSScSetWriteSize(int port, int scopeId,
                                     unsigned int writeSize);
unsigned int xPCFSScGetWriteSize(int port, int scopeId);
void xPCReadXML(int port, int numbytes, unsigned char *data);
diskinfo xPCFSDiskInfo(int port, const char *driveLetter);
const char * xPCFSFileTable(int port, char *tableBuffer);
void xPCFSDirItems(int port, const char *path, dirStruct *dirs, int numDirItems);
int xPCFSDirStructSize(int port, const char *path);
int xPCGetNumScopes(int port);
int xPCGetNumHiddenScopes(int port);
void xPCGetScopeList(int port, int *data);
void xPCGetHiddenList(int port, int *data);
void xPCScGetSignalList(int port, int scNum, int *data);
int xPCGetSimMode(int port);
void xPCGetPCIInfo(int port, char *buf);
double xPCGetSessionTime(int port);
void xPCGetLogStatus(int port, int *logArray);
fileinfo xPCFSFileInfo(int port, int fileHandle);
void xPCSetDefaultStopTime(int port);
int xPCGetXMLSize(int port);
int xPCIsTargetScope(int port);
void  xPCSetTargetScopeUpdate(int port,int value);
void xPCFSReNameFile(int port, const char *fsName, const char *newName);
void xPCFSScSetDynamicMode(int port, int scopeId, int onoff);
int xPCFSScGetDynamicMode(int port, int scopeId);
void xPCFSScSetMaxWriteFileSize(int port, int scopeId,
                                     unsigned int maxWriteFileSize);
unsigned int xPCFSScGetMaxWriteFileSize(int port, int scopeId);


};