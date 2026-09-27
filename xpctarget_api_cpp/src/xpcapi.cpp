#include "xpcapi.hpp"

#include "xpc_msg.hpp"
#include "xpc_comms.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <filesystem>

using namespace xpcapi;

Connection* con_ = new Connection();
Message* msg_ = new Message();

int err_ = 0;

std::string get_app_name() {
    msg_->reset(0xC8, 0x3C);
    msg_->send(*con_);

    std::string res;
    msg_->read(res);
    return res;
} 

int xpcapi::xPCReOpenPort(int port) {
    return 0;
}
int xpcapi::xPCOpenSerialPort(int comport, int baudRate) {
    return 0;
}
void xpcapi::xPCClosePort(int port) {
    // Should Close socket attached to port
    return;
}
int xpcapi::xPCGetLastError(void) {
    // Intentionally left unimplemented
    return 0;
}
void xpcapi::xPCSetLastError(int error) {
    // Intentionally left unimplemented
    return;
}
double xpcapi::xPCGetExecTime(int port) {
    msg_->reset(0x14, 0x1E);
    msg_->send(*con_);

    double time;
    msg_->read(time);
    return time;
}
void xpcapi::xPCSetStopTime(int port, double tfinal) {
    return;
}
double xpcapi::xPCGetStopTime(int port) {
    msg_->reset(0x14, 0x0A);
    msg_->send(*con_);

    double time;
    msg_->read(time);
    return time;
}
void xpcapi::xPCSetSampleTime(int port, double ts) {
    return;
}
double xpcapi::xPCGetSampleTime(int port) {
    msg_->reset(0x14, 0x28);
    msg_->send(*con_);

    double time;
    msg_->read(time);
    return time;
}
void xpcapi::xPCSetEcho(int port, int mode) {
    return;
}
int xpcapi::xPCGetEcho(int port) {
    // Unsure what this does?
    // Sends 0xC8, 0x14, gets a byte, does some manipulation to it then returns it
    return 0;
}
void xpcapi::xPCSetHiddenScopeEcho(int port, int mode) {
    return;
}
int xpcapi::xPCGetHiddenScopeEcho(int port) {
    return 0;
}
double xpcapi::xPCAverageTET(int port) {
    return 0.0;
}
int xpcapi::xPCGetNumParams(int port) {
    msg_->reset(0x14, 0x96);
    msg_->send(*con_);
    uint32_t val;
    msg_->read(val);
    return val;
}
int xpcapi::xPCGetNumSignals(int port) {
    msg_->reset(0x14, 0x97);
    msg_->send(*con_);
    uint32_t val;
    msg_->read(val);
    return val;
}
char * xpcapi::xPCGetAppName(int port, char *modelname) {
    if (modelname == nullptr) {
        return nullptr;
    }
    std::string name = get_app_name();
    std::sprintf(modelname, "%s", name.c_str());
    return modelname;
}
void xpcapi::xPCUnloadApp(int port) {
    // Uses CheckPort
    std::string res = get_app_name();

    if (res.compare("loader") != 0) {
        int status = xPCIsAppRunning(port);

        if (status != 0) {
            xPCStopApp(port);
            return;
        }

        msg_->reset(0x14, 0xAC);
        msg_->send(*con_);
    }
}
void xpcapi::xPCStartApp(int port) {
    msg_->reset(0x14, 0x9b);
    uint32_t data = 5;
    msg_->push(&data, 1);
    msg_->send(*con_);
}
void xpcapi::xPCStopApp(int port) {
    msg_->reset(0x14, 0x9b);
    uint32_t data = 1;
    msg_->push(&data, 1);
    msg_->send(*con_);
}
int xpcapi::xPCIsAppRunning(int port) {
    msg_->reset(0x14, 0xA1);
    msg_->send(*con_);

    uint32_t status;
    msg_->read(status);
    return status;
}
int xpcapi::xPCIsOverloaded(int port) {
    return 0;
}
int xpcapi::xPCGetNumOutputs(int port) {
    msg_->reset(0x14, 0x9E);
    msg_->send(*con_);

    uint32_t outputs;
    msg_->read(outputs);
    return outputs;
}
int xpcapi::xPCGetNumStates(int port) {
    msg_->reset(0x14, 0x9C);
    msg_->send(*con_);
    uint32_t val;
    msg_->read(val);
    return val;
}
void xpcapi::xPCGetParam(int port, int parIdx, double *paramValue) {
    // Not sure why xpcapi sends this packet but I wont change it
    int dims[4];
    xPCGetParamDims(port, parIdx, dims); 

    msg_->reset(0x28, 0x14);
    msg_->push<uint32_t>(reinterpret_cast<uint32_t*>(&parIdx), 1);
    msg_->send(*con_);

    std::vector<double> values;
    msg_->read(values);
    std::move(values.begin(), values.end(), paramValue);
}
void xpcapi::xPCSetLogMode(int port, lgmode lgdata) {
    return;
}
void xpcapi::xPCSetParam(int port, int parIdx, const double *paramValue) {
    int dims_size = xPCGetParamDimsSize(port, parIdx);

    std::vector<int> dims(dims_size, 0);
    xPCGetParamDims(port, parIdx, dims.data());

    int param_value_size = 1;
    for (int i = 0; i < dims.size(); i++) {
        param_value_size *= dims[i];
    }

    msg_->reset(0x28, 0x1E);
    msg_->push<uint32_t>(reinterpret_cast<uint32_t*>(&parIdx), 1);
    msg_->push(paramValue, dims_size);
    msg_->send(*con_);

    return;
}
lgmode xpcapi::xPCGetLogMode(int port) {
    return {};
}
int xpcapi::xPCNumLogSamples(int port) {
    return 0;
}
int xpcapi::xPCMaxLogSamples(int port) {
    return 0;
}
int xpcapi::xPCNumLogWraps(int port) {
    return 0;
}
void xpcapi::xPCReboot(int port) {
    msg_->reset(0xC8, 0x46);
    msg_->send(*con_);
}
void xpcapi::xPCGetOutputLog(int port, int start, int numsamples,
                                  int decimation, int output_id, double *data) {
    return;
}
void xpcapi::xPCGetStateLog(int port, int start, int numsamples,
                                 int decimation, int state_id, double *data) {
    return;
}
void xpcapi::xPCGetTimeLog(int port, int start, int numsamples,
                                int decimation, double *data) {
    return;
}
void xpcapi::xPCGetTETLog(int port, int start, int numsamples,
                               int decimation, double *data) {
    return;
}
void xpcapi::xPCScGetData(int port, int scNum , int signal_id, int start,
                               int numsamples, int decimation, double *data) {
    return;
}
void xpcapi::xPCMinimumTET(int port, double *data) {
    return;
}
void xpcapi::xPCMaximumTET(int port, double *data) {
    return;
}
int xpcapi::xPCGetSignals(int port, int numSignals, const int *signals, double *values) {
    if (numSignals >= 1000) {
        return -1;
    }
    
    msg_->reset(0x28, 0x0A);
    msg_->push<uint32_t>(reinterpret_cast<const uint32_t*>(signals), numSignals);
    msg_->send(*con_);

    msg_->read<double>(values);

    return 0;
}
double xpcapi::xPCGetSignal(int port, int sigNum) {
    double value;
    xPCGetSignals(port, 1, &sigNum, &value);
    return value;
}
void xpcapi::xPCAddScope(int port, int type, int scNum) {
    return;
}
void xpcapi::xPCRemScope(int port, int scNum) {
    return;
}
void xpcapi::xPCScAddSignal(int port, int scNum, int sigNum) {
    return;
}
void xpcapi::xPCScRemSignal(int port, int scNum, int sigNum) {
    return;
}
void xpcapi::xPCScSetAutoRestart(int port, int scNum, int autorestart) {
    return;
}
int xpcapi::xPCScGetAutoRestart(int port, int scNum) {
    return 0;
}
void xpcapi::xPCGetScopes(int port, int *data) {
    return;
}
void xpcapi::xPCGetHiddenScopes(int port, int *data) {
    return;
}
void xpcapi::xPCScGetSignals(int port, int scNum, int *data) {
    return;
}
void xpcapi::xPCScSetDecimation(int port, int scNum, int decimation) {
    return;
}
int xpcapi::xPCScGetNumSignals(int port, int scNum) {
    return 0;
}
int xpcapi::xPCScGetDecimation(int port, int scNum) {
    return 0;
}
void xpcapi::xPCScSetNumSamples(int port, int scNum, int samples) {
    return;
}
int xpcapi::xPCScGetNumSamples(int port, int scNum) {
    return 0;
}
double xpcapi::xPCScGetStartTime(int port, int scNum) {
    return 0.0;
}
int xpcapi::xPCScGetState(int port, int scNum) {
    return 0;
}
void xpcapi::xPCScSetTriggerLevel(int port, int scNum, double level) {
    return;
}
double xpcapi::xPCScGetTriggerLevel(int port, int scNum) {
    return 0.0;
}
void xpcapi::xPCScSetTriggerMode(int port, int scNum, int mode) {
    return;
}
int xpcapi::xPCScGetTriggerMode(int port, int scNum) {
    return 0;
}
void xpcapi::xPCScSetTriggerScope(int port, int scNum, int trigMode) {
    return;
}
int xpcapi::xPCScGetTriggerScope(int port, int scNum) {
    return 0;
}
void xpcapi::xPCScSetTriggerScopeSample(int port, int scNum, int trigScSamp) {
    return;
}
int xpcapi::xPCScGetTriggerScopeSample(int port, int scNum) {
    return 0;
}
void xpcapi::xPCScSetTriggerSignal(int port, int scNum, int trigSig) {
    return;
}
int xpcapi::xPCScGetTriggerSignal(int port, int scNum) {
    return 0;
}
void xpcapi::xPCScSetTriggerSlope(int port, int scNum, int trigSlope) {
    return;
}
int xpcapi::xPCScGetTriggerSlope(int port, int scNum) {
    return 0;
}
void xpcapi::xPCScSoftwareTrigger(int port, int scNum) {
    return;
}
void xpcapi::xPCScStart(int port, int scNum) {
    return;
}
void xpcapi::xPCScStop(int port, int scNum) {
    return;
}
int xpcapi::xPCIsScFinished(int port, int scNum) {
    return 0;
}
int xpcapi::xPCScGetNumPrePostSamples(int port, int scNum) {
    return 0;
}
void xpcapi::xPCScSetNumPrePostSamples(int port, int scNum, int prepost) {
    return;
}
scopedata xpcapi::xPCGetScope(int port, int scNum) {
    return {};
}
void xpcapi::xPCSetScope(int port, scopedata state) {
    return;
}
void xpcapi::xPCLoadApp(int port, const char* pathstr, const char* filename) {
    xPCUnloadApp(port);

    std::filesystem::path base;
    if (!pathstr || (strcmp(pathstr, "nopath") == 0)) {
        base = "./";
    }
    else {
        base = pathstr;
    }
    std::filesystem::path path = base / filename;
    if (!path.has_extension()) {
        path = path.replace_extension(".dlm");
    }
    std::ifstream file(path, std::ios::binary | std::ios::ate);

    if (!file.is_open()) {
        std::stringstream ss;
        ss << "Failed to open file " << path.string();
        throw std::runtime_error(ss.str());
    }
    std::streamsize size = file.tellg();
    std::vector<char> buf(size);

    file.seekg(0, std::ios::beg);
    file.read(buf.data(), size);

    msg_->reset(0xc8, 0x8c);
    msg_->push(buf.data(), buf.size());
    msg_->send(*con_);

}
void xpcapi::xPCGetParamDims(int port, int parIdx, int *dims) {
    msg_->reset(0x28, 0x78);
    msg_->push<uint32_t>(reinterpret_cast<uint32_t*>(&parIdx), 1);
    msg_->send(*con_);

    std::vector<int32_t> values;
    msg_->read(values);
    std::move(values.begin(), values.end(), dims);
}
int xpcapi::xPCGetParamDimsSize(int port, int parIdx) {
    msg_->reset(0x28, 0x6E);
    msg_->push<uint32_t>(reinterpret_cast<uint32_t*>(&parIdx), 1);
    msg_->send(*con_);

    int32_t val;
    msg_->read(val);
    return val;
}
int xpcapi::xPCGetSignalWidth(int port, int sigIdx) {
    msg_->reset(0x28, 0xA0);
    msg_->push<uint32_t>(reinterpret_cast<uint32_t*>(&sigIdx), 1);
    msg_->send(*con_);

    int32_t val;
    msg_->read(val);
    return val;
}
int xpcapi::xPCGetSignalIdx(int port, const char *sigName ) {
    msg_->reset(0x28, 0x46);
    msg_->push_str(sigName);
    msg_->send(*con_);
    uint32_t val;
    msg_->read<uint32_t>(val);
    return val;
}

int xpcapi::xPCGetSigLabelWidth(int port, const char *sigName) {
    msg_->reset(0x28, 0x5A);
    msg_->push_str(sigName);
    msg_->send(*con_);

    int32_t idx_start;
    msg_->read(idx_start);

    int32_t idx_size;
    msg_->read(idx_size);

    return idx_size;
}
int xpcapi::xPCGetSigIdxfromLabel(int port, const char *sigName, int *sigIds) {
    msg_->reset(0x28, 0x5A);
    msg_->push_str(sigName);
    msg_->send(*con_);

    int32_t idx_start;
    msg_->read(idx_start);
    if (idx_start == -1) {
        return -1;
    }

    int32_t idx_size;
    msg_->read(idx_size);
    if (idx_size == 0) {
        return 0;
    }

    for (int i = 0; i < idx_size; i++) {
        sigIds[i] = idx_start;
        idx_start++;
    }

    return 0;
}
char * xpcapi::xPCGetSignalLabel(int port, int sigIdx, char *sigLabel) {
    if (sigLabel == nullptr) {
        return nullptr;
    }

    if (sigIdx < 0) {
        return nullptr;
    }

    msg_->reset(0x28, 0x64);
    msg_->push<uint32_t>(reinterpret_cast<uint32_t*>(&sigIdx), 1);
    msg_->send(*con_);

    std::string str;
    msg_->read(str);
    std::strcpy(sigLabel, str.c_str());

    return sigLabel;
}

int xpcapi::xPCGetParamIdx(int port, const char *block, const char *parameter) {
    msg_->reset(0x28, 0x82);
    msg_->push_str(block);
    msg_->push_str(parameter);
    msg_->send(*con_);

    uint32_t val;
    msg_->read(val);
    return val;
}
void xpcapi::xPCGetParamName(int port, int parIdx, char *block, char *param) {
    msg_->reset(0x28, 0x8C);
    msg_->push<uint32_t>(reinterpret_cast<uint32_t*>(&parIdx), 1);
    msg_->send(*con_);

    std::string str;
    msg_->read(str);
    std::sprintf(block, "%s", str.c_str());
    msg_->read(str);
    std::sprintf(param, "%s", str.c_str());
}
void xpcapi::xPCGetParamType(int port, int parIdx, char *paramType) {
    msg_->reset(0x28, 0x96);
    msg_->push<uint32_t>(reinterpret_cast<uint32_t*>(&parIdx), 1);
    msg_->send(*con_);

    std::string str;
    msg_->read(str);
    std::sprintf(paramType, "%s", str.c_str());
}
char * xpcapi::xPCGetSignalName(int port, int sigIdx, char *sigName) {
    if (sigName == nullptr) {
        return nullptr;
    }

    if (sigIdx < 0) {
        return nullptr;
    }

    msg_->reset(0x28, 0x50);
    msg_->push<uint32_t>(reinterpret_cast<uint32_t*>(&sigIdx), 1);
    msg_->send(*con_);

    std::string str;
    msg_->read(str);
    std::strcpy(sigName, str.c_str());

    return sigName;
}
int xpcapi::xPCTgScGetGrid(int port, int scNum) {
    return 0;
}
int xpcapi::xPCTgScGetMode(int port, int scNum) {
    return 0;
}
int xpcapi::xPCTgScGetViewMode(int port) {
    return 0;
}
void xpcapi::xPCTgScGetYLimits(int port, int scNum, double *limits) {
    return;
}
void xpcapi::xPCTgScSetGrid(int port, int scNum, int flag) {
    return;
}
void xpcapi::xPCTgScSetMode(int port, int scNum, int flag) {
    return;
}
void xpcapi::xPCTgScSetViewMode(int port, int scNum) {
    return;
}
void xpcapi::xPCTgScSetYLimits(int port, int scNum, const double *limits) {
    return;
}
void xpcapi::xPCTgScSetSignalFormat(int port, int scNum, int signalNo, const char *signalFormat) {
    return;
}
char* xpcapi::xPCTgScGetSignalFormat(int port,int scNum, int signalNo, char *signalFormat) {
    return nullptr;
}
void xpcapi::xPCSetLoadTimeOut(int port, int timeOut) {
    return;
}
const char * xpcapi::xPCErrorMsg(int errorno, char *errmsg) {
    return nullptr;
}
int xpcapi::xPCScGetType(int port, int scNum) {
    return 0;
}
int xpcapi::xPCGetLoadTimeOut(int port) {
    return 0;
}
int xpcapi::xPCOpenTcpIpPort(const char* address, const char* port) {
    con_->con_connect(address, std::stoi(port));

    return 0;
}
void xpcapi::xPCOpenConnection(int port) {
    return;
}
void xpcapi::xPCCloseConnection(int port) {
    xPCClosePort(port);
    return;
}
int xpcapi::xPCRegisterTarget(int commType, const char *ipAddress,
                                   const char *ipPort,
                                   int comPort, int baudRate) {
    return 0;
}
void xpcapi::xPCDeRegisterTarget(int port) {
    return;
}
const char * xpcapi::xPCGetAPIVersion(void) {
    return "cpp_0.1";
}
void xpcapi::xPCGetTargetVersion(int port, char *ver) {
    msg_->reset(0xC8, 0x0A);
    msg_->send(*con_);

    std::string ver_str;
    msg_->read(ver_str);

    if (ver_str.compare(0, 9, "xPCTarget") != 0) {
        return;
    }

    ver_str = ver_str.substr(9);
    std::strcpy(ver, ver_str.c_str());
}
int xpcapi::xPCTargetPing(int port) {
    try {
        char ver[112];
        xPCGetTargetVersion(port, ver);
    }
    catch (const std::runtime_error& e) {
        return -1;
    }

    return 0;
}
void xpcapi::xPCFSReadFile(int port, int fileHandle, int start,
                               int numsamples, unsigned char *data) {
    return;
}
int xpcapi::xPCFSRead(int port, int fileHandle, int start,
                           int numsamples, unsigned char *data) {
    return 0;
}
void xpcapi::xPCFSWriteFile(int port, int fileHandle, int numbytes,
                                 const unsigned char *data) {
    return;
}
void xpcapi::xPCFSBufferInfo(int port, char *data) {
    return;
}
int xpcapi::xPCFSGetFileSize(int port, int fileHandle) {
    return 0;
}
int xpcapi::xPCFSOpenFile(int port, const char *filename,
                               const char *attrib) {
    return 0;
}
void xpcapi::xPCFSCloseFile(int port, int fileHandle) {
    return;
}
void xpcapi::xPCFSGetPWD(int port, char *data) {
    return;
}
void xpcapi::xPCFTPGet(int port, int fileHandle, int numbytes, char *filename) {
    return;
}
void xpcapi::xPCFTPPut(int port, int fileHandle, char *filename) {
    return;
}
void xpcapi::xPCFSRemoveFile(int port, char *filename) {
    return;
}
void xpcapi::xPCFSCD(int port, char *filename) {
    return;
}
void xpcapi::xPCFSMKDIR(int port, const char *dirname) {
    return;
}
void xpcapi::xPCFSRMDIR(int port, const char *dirname) {
    return;
}
void xpcapi::xPCFSDir(int port, const char *path, char *listing, int numbytes) {
    return;
}
int xpcapi::xPCFSDirSize(int port, const char *path) {
    return 0;
}
void xpcapi::xPCFSGetError(int            port,
                                unsigned int   errCode,
                                unsigned char *message) {
    return;
}
void xpcapi::xPCSaveParamSet(int port, const char *filename) {
    return;
}
void xpcapi::xPCLoadParamSet(int port, const char *filename) {
    return;
}

void xpcapi::xPCFSScSetFilename(int port, int scopeId,
                                     const char *filename) {
    return;
}
const char * xpcapi::xPCFSScGetFilename(int port, int scopeId,
                                             char *filename) {
    return nullptr;
}
void xpcapi::xPCFSScSetWriteMode(int port, int scopeId,
                                      int writeMode) {
    return;
}
int xpcapi::xPCFSScGetWriteMode(int port, int scopeId) {
    return 0;
}

void xpcapi::xPCFSScSetWriteSize(int port, int scopeId,
                                     unsigned int writeSize) {
    return;
}
unsigned int xpcapi::xPCFSScGetWriteSize(int port, int scopeId) {
    return 0;
}
void xpcapi::xPCReadXML(int port, int numbytes, unsigned char *data) {
    return;
}
diskinfo xpcapi::xPCFSDiskInfo(int port, const char *driveLetter) {
    return {};
}
const char * xpcapi::xPCFSFileTable(int port, char *tableBuffer) {
    return nullptr;
}
void xpcapi::xPCFSDirItems(int port, const char *path, dirStruct *dirs, int numDirItems) {
    return;
}
int xpcapi::xPCFSDirStructSize(int port, const char *path) {
    return 0;
}
int xpcapi::xPCGetNumScopes(int port) {
    msg_->reset(0x1E, 0x99);
    msg_->send(*con_);
    uint32_t val;
    msg_->read(val);
    return val;
}
int xpcapi::xPCGetNumHiddenScopes(int port) {
    msg_->reset(0x1E, 0x9A);
    msg_->send(*con_);
    uint32_t val;
    msg_->read(val);
    return val;
}
void xpcapi::xPCGetScopeList(int port, int *data) {
    return;
}
void xpcapi::xPCGetHiddenList(int port, int *data) {
    if (data == nullptr) {
        return;
    }

    msg_->reset(0x1E, 0x9D);
    msg_->send(*con_);

    std::vector<int32_t> data_vec;
    msg_->read(data_vec);

    std::move(data_vec.begin(), data_vec.end(), data);
}
void xpcapi::xPCScGetSignalList(int port, int scNum, int *data) {
    return;
}
int xpcapi::xPCGetSimMode(int port) {
    msg_->reset(0x14, 0xA6);
    msg_->send(*con_);

    uint32_t val;
    msg_->read(val);
    return val;
}
void xpcapi::xPCGetPCIInfo(int port, char *buf) {
    msg_->reset(0xC8, 0x50);
    msg_->send(*con_);

    std::vector<uint8_t> values;
    msg_->read(values);
    std::move(values.begin(), values.end(), buf);
}
double xpcapi::xPCGetSessionTime(int port) {
    msg_->reset(0xC8, 0x96);
    msg_->send(*con_);

    double val;
    msg_->read(val);
    return val;
}
void xpcapi::xPCGetLogStatus(int port, int *logArray) {
    return;
}
fileinfo xpcapi::xPCFSFileInfo(int port, int fileHandle) {
    return {};
}
void xpcapi::xPCSetDefaultStopTime(int port) {
    return;
}
int xpcapi::xPCGetXMLSize(int port) {
    return 0;
}
int xpcapi::xPCIsTargetScope(int port) {
    return 0;
}
void  xpcapi::xPCSetTargetScopeUpdate(int port,int value) {
    return;
}
void xpcapi::xPCFSReNameFile(int port, const char *fsName, const char *newName) {
    return;
}
void xpcapi::xPCFSScSetDynamicMode(int port, int scopeId, int onoff) {
    return;
}
int xpcapi::xPCFSScGetDynamicMode(int port, int scopeId) {
    return 0;
}
void xpcapi::xPCFSScSetMaxWriteFileSize(int port, int scopeId,
                                     unsigned int maxWriteFileSize) {
    return;
}
unsigned int xpcapi::xPCFSScGetMaxWriteFileSize(int port, int scopeId) {
    return 0;
}