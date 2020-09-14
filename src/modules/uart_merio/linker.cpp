#include <unistd.h>
#include <string.h>
#include "linker.h"

#define CRCkey	0x1021

Linker::Linker()
{
    initCRCtab();
}
void Linker::initCRCtab(void)
{
    int16_t i, j;
    uint16_t crc, c;
    for ( i = 0 ; i < 256 ; i++)
    {
        crc = 0;
        c = i << 8;
        for ( j = 0 ; j < 8 ; j++)
        {
            if ((crc ^ c) & 0x8000)
            {
                crc = (crc << 1) ^ CRCkey;
            }
            else
            {
                crc = crc << 1;
            }
            c = c << 1;
        }
        _CRCtab[i] = crc;
    }
}
uint16_t Linker::CRC16(uint8_t* buffer, uint16_t len, uint16_t crc)
{
    uint16_t tmp;
    uint16_t i;
    for(i = 0 ; i < len ; i++)
    {
        tmp = (crc >> 8) ^ buffer[i];
        crc = (crc << 8) ^ _CRCtab[tmp];
    }
    return crc;
}

bool Linker::connectWith(int fd)
{
	_serialPort = fd;
    return true;
}
/*
void Linker::disconnectWith()
{
    if(_serialPort->isOpen())
    {
        disconnect(_serialPort, SIGNAL(readyRead()),
                   this, SLOT(handleReadyRead()));
        disconnect(_serialPort, SIGNAL(error(QSerialPort::SerialPortError)),
                   this, SLOT(handleError(QSerialPort::SerialPortError)));
        disconnect(_serialPort, SIGNAL(bytesWritten(qint64)),
                   this, SLOT(handleBytesWritten(qint64)));
        _serialPort->close();
        delete _serialPort;
    }
}
*/
bool Linker::getConnectStatus()
{
	/*
    std::cout<<"Connection ";
    if(_isConnected)
        std::cout << "True\n";
    else
        std::cout << "False\n";
*/
    return _isConnected;
}

void Linker::addBlocGENPAYLOAD(uint8_t blocId)
{
    _sendBuffer[_sendPtr++]=2; /* set LEN */
    _sendBuffer[_sendPtr++]=GENPAYLOAD; /* CLASS */
    _sendBuffer[_sendPtr++]=blocId; /* ID */
    _lenPos = _sendPtr -3;
    switch(blocId)
    {
    case B_REQCMD:
        _sendBuffer[_sendPtr++] = B_STATUS;
        if(!_isConnected)
        {
            _sendBuffer[_sendPtr++] = B_SN;
            _sendBuffer[_sendPtr++] = B_PN;
            _sendBuffer[_sendPtr++] = B_SOFTMAIN;
        }
        addLEN();
        break;

    case B_POSITION:
        _sendBuffer[_lenPos] = 8;
        X32ToInt16(_posRefAzi,0.005729577951300823); //scalling modified to convert degree to rad
        X32ToInt16(_posRefEle,0.005729577951300823);
        X32ToInt16(0.5, 6e-5);
        addLEN(); // sould be 8
        break;
    case B_ICR:
        _sendBuffer[_lenPos] = 3;
        if(_ICRReq)
        {
            _sendBuffer[_sendPtr++] = 0x01;
            _ICRReq = false;
        }
        else
        {
            _sendBuffer[_sendPtr++] = 0x00;
            _ICRReq = true;
        }
        addLEN(); // should be 3
        break;
    case B_NUC: /* no payload (LEN=2)*/
    	addLEN();
        break;
    case B_IRPOLARITY:/* no payload (LEN=2)*/
    	addLEN();
        break;
    case B_ACK:/* no payload (LEN=2)*/
    	addLEN();
        break;
    }
}

void Linker::addLEN()
{
	if(_lenPos!=0) {
		_sendBuffer[_lenPos] = _sendPtr-_lenPos-1;//LEN
		_lenPos = 0;
	}
}
/*
// ========= Reading Blocs
void Linker::handleReadyRead()
{

    isReading = true;
    readData.append(_serialPort->readAll());
    for (int i = 0; i < readData.size() ; i++)
    {
        receive(readData.at(i));
    }
    readData.resize(0);
    isReading = false;
}
*/
void Linker::receive(uint8_t data)
{
    _isReading = true;
    data &= 0xff;
//    qDebug() << "received : " << QString::asprintf("0x%02X (%u)",data, data);
    if (data == 0xAA)
    {
//        qDebug() << "0xAA received";
        decode(data, 1);
        _x5Adetected = 0;
    }
    else
    {
        if (_x5Adetected == 1)
        {
            _x5Adetected = 0;
            if(data == 0x00)
            {
                decode(0xAA, 0);
            }
            else
            {
                decode(0x5A, 0);
            }
        }
        else
        {
            if(data == 0x5A)
            {
                _x5Adetected = 1;
            }
            else
            {
                decode(data, 0);
            }
        }
    }
    _isReading = false;
}
void Linker::decode(uint8_t data, int16_t status)
{
    if(status == 1)
    {
        _ptr = 0;
        _checkFlag = 0;
        _state = LEN;
    }
    else if(_state == LEN)
    {
        _len = (int16_t)data;
        _buffer[_ptr++] = _len;
        if ((_len == 0) || (_len > 253))
        {
            _state = START;
        }
        else
        {
            _state = DATA;
        }
    }
    else if (_state == DATA)
    {
        _buffer[_ptr++] = data;
        if (_ptr == (_len+1))
        {
            _state = CHECK;
        }
    }
    else if (_state == CHECK)
    {
        if (_checkFlag == 0)
        {
            _checksum = data;
            _checkFlag = 1;
        }
        else
        {
            _state = START;
            _checksum += data << 8;
            if (_checksum == CRC16(_buffer, _len + 1, 0x00))
            {
//                qDebug() << "CHECKSUM OK";
                reader(&_buffer[1], _len);
            }
            else
            {
//                qDebug() << "CHECKSUM ERROR";
            }
        }
    }
}
void Linker::reader(uint8_t* data, int16_t len)
{
    int16_t ptr = 0;
    uint8_t id;
    id = data[0];
//    qDebug() << "reader ID " << QString::asprintf("0x%02X (%u)",id, id);
//    qDebug() << "len : " << len;
    if (id == 0)
    {
        _sendPtr = 2;
        data++;
        while (ptr < (len-1))
        {
//            qDebug() << "ptr : " << ptr;
//            qDebug() << "len bloc : " << QString::asprintf("%u",data[ptr]) ;
            int16_t add = ptr + data[ptr] + 1;
//            qDebug() << "next : " << add;
            if(add <= len)
            {
                unbloc(&data[ptr]);
            }
            else
            {
//                qDebug() << "bloc trop long";
            }
            ptr = add;
        }
    }
}
void Linker::unbloc(uint8_t* buffer)
{
    uint8_t len = buffer[0];
    uint8_t classe = buffer[1];
    uint8_t id = buffer[2];
    uint8_t *payload = &buffer[3];

    std::cout << "unbloc class-id ";
    printf("0x%02X- 0x%02X\n",classe, id);
    switch(id)
    {
    case B_SN:
        _isConnected = true;
        std::cout <<"Serial Number: "<<readString(&payload[0])<<std::endl;
        break;
    case B_PN:
        std::cout <<"Part Number : "<<readString(&payload[0])<<std::endl;
        break;
    case B_SOFTMAIN:
        std::cout <<"Software : "<<readString(&payload[0])<<std::endl;
        break;
    case B_STATUS:
    {
        int gimbalModeStatus = (payload[9] >> 5)&0x07;
        std::cout <<"Mode Status : "<<gimbalModeStatus<<std::endl;
        bool icrOnOff = readBitfromByte(&payload[11],5);
        if(icrOnOff)
        {
            std::cout <<"ICR Status : ON"<<std::endl;
        }
        else
        {
            std::cout <<"ICR Status : OFF"<<std::endl;
        }
    }
        break;
    default:
        std::cout<<"Received Unhandled Bloc of ID :"<<id
                <<" and size:"<<len<<std::endl;
        break;
    }

}

// ========= Writing Blocs
void Linker::addBloc(uint8_t blocClass, uint8_t blocId)
{
    switch(blocClass)
    {
    case GENPAYLOAD:
        addBlocGENPAYLOAD(blocId);
        break;
    case LRFPAYLOAD:
        /*addBlockLRFPAYLOAD(blockId);*/
        break;
    default:
        std::cout<<"Classe non implémentée"<<std::endl;
        break;
    }
}
void Linker::sendProtocol()
{
    int16_t i;
    uint16_t checksum;
    send(0xAA);
    _sendBuffer[0] = _sendPtr - 1;
    _sendBuffer[1] = 0x00;	// payload id, fixed to 0x00
    for ( i = 0 ; i < _sendPtr ; i++)
    {
        sendByte(_sendBuffer[i]);
    }
    checksum = CRC16(_sendBuffer, _sendPtr, 0x00);
    sendByte(checksum & 0xff);
    sendByte(checksum >> 8);
    _sendPtr=2;
}
void Linker::sendByte(uint8_t data)
{
    data &= 0xff;
    if (data == 0xAA)
    {
        send(0x5A);
        send(0x00);
    }
    else if (data == 0x5A)
    {
        send(0x5A);
        send(0x5A);
    }
    else
    {
        send(data);
    }
}
void Linker::send(uint8_t data)
{
    if (_serialPort != -1)
    {
        size_t len = ::write(_serialPort, &data, 1);
        if(len!=1) {
        	_isConnected = false;
        }
    }
}


// ========= Data to Bytes/ Bytes to data
//-Write Byte
void Linker::fillUInt8(uint8_t data)
{
    _sendBuffer[_sendPtr++] = (int8_t)(data &0xff);
}
void Linker::fillInt16(int16_t data)
{
    _sendBuffer[_sendPtr++] = (int8_t)(data &0xff);
    _sendBuffer[_sendPtr++] = (int8_t)((data >> 8)&0xff);
}
void Linker::fillUInt16(uint16_t data)
{
    _sendBuffer[_sendPtr++] = (uint8_t)(data &0xff);
    _sendBuffer[_sendPtr++] = (uint8_t)((data >> 8)&0xff);
}
void Linker::fillInt32(int32_t data)
{
    _sendBuffer[_sendPtr++] = (int8_t)(data &0xff);
    _sendBuffer[_sendPtr++] = (int8_t)((data >> 8)&0xff);
    _sendBuffer[_sendPtr++] = (int8_t)((data >> 16)&0xff);
    _sendBuffer[_sendPtr++] = (int8_t)((data >> 24)&0xff);
}
void Linker::fillUInt32(uint32_t data)
{
    _sendBuffer[_sendPtr++] = (uint8_t)(data &0xff);
    _sendBuffer[_sendPtr++] = (uint8_t)((data >> 8)&0xff);
    _sendBuffer[_sendPtr++] = (uint8_t)((data >> 16)&0xff);
    _sendBuffer[_sendPtr++] = (uint8_t)((data >> 24)&0xff);
}
void Linker::fillX32(float data)
{
    static unsigned char buffer[4];
    static float* tmp;
    tmp = (float*)&data;
    memcpy(buffer, tmp, 4);

    _sendBuffer[_sendPtr++] = buffer[0];
    _sendBuffer[_sendPtr++] = buffer[1];
    _sendBuffer[_sendPtr++] = buffer[2];
    _sendBuffer[_sendPtr++] = buffer[3];
}

void Linker::X32ToInt8(double data, double scale)
{
    int8_t tmp;
    data /= scale;
    if(data > 255)
    {
        data = 255;
    }
    else if( data < 0.0)
    {
        data = 0.0f;
    }
    tmp = (int8_t) data;
    _sendBuffer[_sendPtr++] = tmp;
}
void Linker::X32ToUInt8(double data, double scale)
{
    uint8_t tmp;
    data /= scale;
    if(data > 255)
    {
        data = 255;
    }
    else if( data < 0.0)
    {
        data = 0.0f;
    }
    tmp = (uint8_t) data;
    _sendBuffer[_sendPtr++] = tmp;
}
void Linker::X32ToInt16(double data, double scale)
{
    int16_t tmp;
    data /= scale;
    if(data > 32767.0)
    {
        data = 32767.0;
    }
    else if(data < -32767.0)
    {
        data = -32767.0;
    }
    tmp = (int)data;
    fillInt16(tmp);

}
void Linker::X32ToUInt16(double data, double scale)
{
    uint16_t tmp;
    data /= scale;
    if(data > 65535.0)
        data = 65535.0;
    else if(data < 0.0)
        data = 0.0;
    tmp = (uint16_t)data;
    fillInt16(tmp);
}
void Linker::X32ToInt32(double data, double scale)
{
    int32_t tmp;
    data /= scale;
    if(data > 2147483647.0)
    {
        data = 2147483647.0;
    }
    else if(data < -2147483648.0)
    {
        data = -2147483648.0;
    }
    tmp = (int32_t)data;
    fillInt32(tmp);
}
void Linker::X32ToUInt32(double data, double scale)
{
    uint32_t tmp;
        data /= scale;
        if(data > 4294967295.0)
        {
            data = 4294967295.0;
        }
        else if(data < 0.0)
        {
            data = 0.0;
        }
        tmp = (uint32_t)data;
        fillUInt32(tmp);
}

//- Read Byte
bool Linker::readBitfromByte(uint8_t *data, int bit)
{
    bool tmp = false;
      switch (bit)
      {
      case 0:
      {
          tmp = (data[0] & 0x01)!=0;
      }
          break;
      case 1:
      {
          tmp = (data[0] & 0x02)!=0;
      }
          break;
      case 2:
      {
          tmp = (data[0] & 0x04)!=0;
      }
          break;
      case 3:
      {
          tmp = (data[0] & 0x08)!=0;
      }
          break;
      case 4:
      {
          tmp = (data[0] & 0x10)!=0;
      }
          break;
      case 5:
      {
          tmp = (data[0] & 0x20)!=0;
      }
          break;
      case 6:
      {
          tmp = (data[0] & 0x40)!=0;
      }
          break;
      case 7:
      {
          tmp = (data[0] & 0x80)!=0;
      }
          break;
      default:
          return false;
          break;
      }
      return tmp;
}
std::string Linker::readString(uint8_t *data)
{
    std::string tmp = "";
    int i = 0;
    while(data[i] != 0)
    {
        tmp += data[i];
        i++;
    }
    return tmp;
}
int8_t Linker::readInt8(uint8_t *data)
{
    int8_t tmp = data[0];
    return tmp;
}
int16_t Linker::readInt16(uint8_t *data)
{
    return data[0] + (data[1] << 8);
}
uint16_t Linker::readUInt16(uint8_t *data)
{
    return data[0] + (data[1] << 8);
}
int32_t Linker::readInt32(uint8_t *data)
{
    int32_t value = 0;
    for(int i = 0; i<4; i++)
    {
        int32_t tmp = data[i];
        tmp = tmp<<i*8;
        value += tmp;
    }
    return value;
}
uint32_t Linker::readUInt32(uint8_t *data)
{
    uint32_t value = 0;
    for(int i = 0; i<4; i++)
    {
        uint32_t tmp = data[i];
        tmp = tmp<<i*8;
        value += tmp;
    }
    return value;
}
float Linker::readX32(uint8_t *data)
{
    unsigned char tmp[4];
    tmp[0] = data[0];
    tmp[1] = data[1];
    tmp[2] = data[2];
    tmp[3] = data[3];
    float floatReaded;
    memcpy(&floatReaded, tmp, 4);

    return floatReaded;
}
