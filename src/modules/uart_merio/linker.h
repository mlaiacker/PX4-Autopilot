#ifndef LINKER_H
#define LINKER_H

#include <stdint.h>
//  Class GEN
#define GENPAYLOAD          0x00
//Associated blocs -v-
//**Commands
#define B_MODE              0x00
#define B_VELOCITY          0x01
#define B_POSITION          0x02
#define B_GEOTRACKING       0x03
#define B_NUC               0x05
#define B_VELOFFSET         0x06
#define B_ALIM              0x07
#define B_LASER             0x10
#define B_DZOOMVEL          0x15
#define B_DZOOMPOS          0x16
#define B_AUTOFOCUS         0x17
#define B_MANUALFOCUS       0x18
#define B_ICR               0x19
#define B_DSONYFEATURES     0x1A
#define B_VIDEOSWITCH       0x26
#define B_NAVCTRL           0x27
#define B_IRPOLARITY        0x30
#define B_IZOOMVEL          0x31
#define B_NAVINFO1          0x36
#define B_NAVINFO2          0x37
#define B_GPSTIME           0x3A
#define B_GROUNDMLS         0x3B
#define B_REQCMD            0x40
//**Status
#define B_STATUS            0x80
#define B_QUATERNION        0x81
#define B_TEMPERATURE       0x82
#define B_EULERUAV          0x83
#define B_LLHUAV            0x84
#define B_LLHTARGET         0x85
#define B_LLHTARGETREF      0x86
#define B_UTCDATE           0x87
#define B_UTCTIME           0x88
#define B_SOFTMAIN          0x91
#define B_PN                0x94
#define B_SN                0x95
#define B_MODELCAMDAY       0x96
#define B_MODELCAMIR        0x97
#define B_MODELLASER        0x98
#define B_QUARKSTATUS       0x99
#define B_ACK               0x9A
#define B_GIMBALCONFIG      0x9B


//  Class LRF
#define LRFPAYLOAD          0x04
//Associated blocs -v-
//**Commands
#define B_TRIGLRF           0x00
//**Status
#define B_LRFRESULT         0x80






//Linker class is handeling the receiving of messages from the gimbal and the sending of messages to the gimbal.
class Linker
{

public:
    explicit Linker();
    /**
     * @brief sendProtocol
     * This method is to be called each time a message need to be send to the gimbal
     */
    void sendProtocol();

    /**
     * @brief addBloc allow to add a bloc to the sending list
     * @param blocClass  -- Id of the Bloc's class (see list of those IDs communiaction protocol)
     * @param blocId    -- Id of the Bloc(see list of those IDs in communiaction protocol)
     */
    void addBloc(uint8_t blocClass, uint8_t blocId);
    uint8_t addLEN();
    bool connectWith(int fd);
    bool getConnectStatus();

    void blocReceived(uint8_t);

public:
    /**
     * @brief receive
     * @param data
     * This method received Bytes one by one, it translate the start bytes
     * (see document TSC-B004 for more precision).
     * And it send them to the decode method.
     */
    void receive(uint8_t data);
    /**
     * @brief handleReadyRead
     * This slot is called by the readyRead() signal from QSerialPort
     * It get the whole bytes received and send them to the receive method
     */
    //void handleReadyRead();
    //void handleError(QSerialPort::SerialPortError serialPortError);
    //void handleBytesWritten(qint64 bytes);
    //void disconnectWith();

    //-Write Byte
        void fillUInt8(uint8_t data);
        void fillInt16(int16_t data);
        void fillUInt16(uint16_t data);
        void fillInt32(int32_t data);
        void fillUInt32(uint32_t data);
        void fillX32(float data);

        void X32ToInt8(double data, double scale);
        void X32ToUInt8(double data, double scale);
        void X32ToInt16(double data, double scale);
        void X32ToUInt16(double data, double scale);
        void X32ToInt32(double data, double scale);
        void X32ToUInt32(double data, double scale);
private:
    enum State {
        START = 0,
        LEN = 1,
        DATA = 2,
        CHECK = 3
    };
    int _serialPort = -1;
    //std::array<uint8_t>  readData;
    //std::array<uint8_t>  writeData;

    State _state = START;
    int16_t  _x5Adetected = 0;
    int16_t  _len, _lenPos;
    int16_t  _ptr;
    int16_t  _sendPtr = 0;
    uint8_t  _buffer[257];
    uint8_t  _sendBuffer[300];
    uint16_t _CRCtab[256];
    uint32_t  _bytesWritten;
    int16_t _checkFlag = 0;
    uint16_t _checksum;
    bool _isReading = false ;
    bool _isConnected = false;
    bool _ICRReq = true;

    double _posRefAzi = 0;
    double _posRefEle = 10;

    void addBlocGENPAYLOAD(uint8_t blocId);
    void decode(uint8_t data, int16_t status);
    /**
     * @brief CRC16
     * @param buffer
     * @param len
     * @param crc
     * @return
     * This method compute the checksum given received message and return it
     * initCRCtab must be called once before to be able to compute the checksum
     * correctly.
     */
    uint16_t CRC16(uint8_t *buffer, uint16_t len, uint16_t crc);
    void initCRCtab();
    /**
     * @brief reader
     * @param data
     * @param len
     * This method identify blocs in the message and read them one by one using
     * unbloc method
     */
    void reader(uint8_t *data, int16_t len);
    /**
     * @brief unbloc
     * @param buffer
     * This method read the content of a message bloc
     */
    void unbloc(uint8_t *buffer);
    /**
     * @brief sendByte
     * @param data
     * sendByte is the byteStuffer
     */
    void sendByte(uint8_t data);
    /**
     * @brief send
     * @param data
     * This method uses the write method of QSerialPort to send bytes to the gimbal
     */
    void send(uint8_t data);

//- Read Byte
    bool    readBitfromByte(uint8_t *data, int bit);
    void readString(uint8_t *data, char* str);
    int8_t  readInt8(uint8_t *data);
    int16_t readInt16(uint8_t *data);
    uint16_t readUInt16(uint8_t *data);
    int32_t readInt32(uint8_t *data);
    uint32_t readUInt32(uint8_t *data);
    float readX32(uint8_t *data);
};

#endif // LINKER_H
